use colored::Colorize;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::{
    AprioriPosition, Candidate, Carrier, Config, Duration, Epoch, InterpolationResult,
    IonosphereBias, Method, Observation, Solver, TroposphereBias, Vector3,
};
use map_3d::{Ellipsoid, ecef2geodetic};
use once_cell::sync::Lazy;
use std::sync::{Arc, Mutex};

use crate::{
    constants::{EARTH_MU_GPS, EARTH_ROTATION_RATE, SPEED_OF_LIGHT},
    ephemeris::Ephemeris,
    state::GnssState,
};

const PI: f64 = std::f64::consts::PI;

fn get_eccentric_anomaly(eph: &Ephemeris, t_k: f64) -> f64 {
    // computed mean motion
    let n0 = (EARTH_MU_GPS / eph.a.powi(3)).sqrt();
    // corrected mean motion
    let n = n0 + eph.deln;
    // mean anomaly
    let mk = eph.m0 + n * t_k;

    let mut e = mk;
    let mut e_k = 0.0;
    let mut n_iter = 0;

    while (e - e_k).abs() > 1e-14 && n_iter < 30 {
        e_k = e;
        e = e + (mk - e + eph.ecc * e.sin()) / (1.0 - eph.ecc * e.cos());
        n_iter += 1;
    }
    assert!(n_iter < 20);

    e
}

fn compute_sv_position_ecef(eph: &Ephemeris, t: Epoch) -> (f64, f64, f64) {
    let mut dte = (t - eph.toe_gpst).to_seconds();

    log::warn!("{}: ---- now={t:?}", eph.sv);
    log::warn!("{}: ---- toe={:?} delta-t={dte} ", eph.sv, eph.toe_gpst);

    if dte > 302400.0 {
        dte -= 604800.0;
    }
    if dte < -302400.0 {
        dte += 604800.0;
    }

    let ecc_anomaly = get_eccentric_anomaly(eph, dte);
    let v_k =
        ((1.0 - eph.ecc.powi(2)).sqrt() * ecc_anomaly.sin()).atan2(ecc_anomaly.cos() - eph.ecc);

    let phi_k = v_k + eph.omg;
    let duk = eph.cus * (2.0 * phi_k).sin() + eph.cuc * (2.0 * phi_k).cos();
    let drk = eph.crs * (2.0 * phi_k).sin() + eph.crc * (2.0 * phi_k).cos();
    let dik = eph.cis * (2.0 * phi_k).sin() + eph.cic * (2.0 * phi_k).cos();

    let uk = phi_k + duk;
    let rk = eph.a * (1.0 - eph.ecc * ecc_anomaly.cos()) + drk;
    let ik = eph.i0 + eph.i_dot * dte + dik;

    let orb_plane_x = rk * uk.cos();
    let orb_plane_y = rk * uk.sin();

    let omega =
        eph.omg0 + (eph.omg_dot - EARTH_ROTATION_RATE) * dte - EARTH_ROTATION_RATE * eph.toe as f64;

    let ecef_x = orb_plane_x * omega.cos() - orb_plane_y * ik.cos() * omega.sin();
    let ecef_y = orb_plane_x * omega.sin() + orb_plane_y * ik.cos() * omega.cos();
    let ecef_z = orb_plane_y * ik.sin();

    log::warn!(
        "{}: position: x={:8.1} y={:8.1} z={:8.1} h={:.1}",
        eph.sv,
        ecef_x / 1000.0,
        ecef_y / 1000.0,
        ecef_z / 1000.0,
        (ecef_x.powi(2) + ecef_y.powi(2) + ecef_z.powi(2)).sqrt() / 1000.0
    );
    let (lat_rad, lon_rad, h) = ecef2geodetic(ecef_x, ecef_y, ecef_z, Ellipsoid::WGS84);
    log::warn!(
        "{}: position: lat/lon: {:.6},{:.6} h={:.1}",
        eph.sv,
        lat_rad * 180.0 / PI,
        lon_rad * 180.0 / PI,
        h / 1000.0
    );
    (ecef_x, ecef_y, ecef_z)
}

fn get_tropo_iono_bias() -> (TroposphereBias, IonosphereBias) {
    let iono_bias = IonosphereBias {
        kb_model: None,
        bd_model: None,
        ng_model: None,
        stec_meas: None,
    };
    let tropo_bias = TroposphereBias {
        total: None,
        zwd_zdd: None,
    };
    (tropo_bias, iono_bias)
}

pub type I = fn(Epoch, SV, usize) -> Option<InterpolationResult>;
pub struct PositionSolver {
    solver: Solver<I>,
    pub_state: Arc<Mutex<GnssState>>,
}

static SOLVER_EPHEMERIS: Lazy<Mutex<Vec<Ephemeris>>> =
    Lazy::new(|| Mutex::new(Vec::<Ephemeris>::new()));

fn sv_interp(t: Epoch, sv: SV, _size: usize) -> Option<InterpolationResult> {
    let ephs = SOLVER_EPHEMERIS.lock().unwrap();
    let eph = ephs.iter().find(|&&e| e.sv == sv).unwrap();
    let pos = compute_sv_position_ecef(eph, t);

    Some(InterpolationResult::from_apc_position(pos))
}

impl PositionSolver {
    #[allow(clippy::new_without_default)]
    pub fn new(pub_state: Arc<Mutex<GnssState>>) -> Self {
        let apriori = AprioriPosition::from_geo(Vector3::new(46.5, 6.6, 0.0));
        let mut cfg = Config::static_preset(Method::SPP);
        cfg.min_sv_elev = Some(0.0);

        let solver = Solver::new(&cfg, apriori, sv_interp as I).expect("Solver issue");

        Self { solver, pub_state }
    }

    pub fn compute_position(&mut self, ts_sec: f64, ephs: &Vec<Ephemeris>) {
        {
            let mut glob_ephs = SOLVER_EPHEMERIS.lock().unwrap();
            *glob_ephs = ephs.clone();
        }

        /*
         * https://www.insidegnss.com/auto/IGM_janfeb12-Solutions.pdf
         *
         * sat0 is the closest. sat2 is the furthest.
         *          tow
         * sat0 -----+[....][....][....][....][....][....][...| obs   <-- reference
         *                 tow                                |
         * sat1 ------------+[....][....][....][....][....][..| obs
         *                        tow                         |
         * sat2 -------------------+[....][....][....][....][.| obs
         *
         *  sat0      []                   ~0
         *  sat1      [------]
         *  sat2      [-------------]
         */
        let mut pool = vec![];

        let min_gpst = ephs
            .iter()
            .map(|&eph| eph.tow_gpst + Duration::from_seconds(ts_sec - eph.ts_sec))
            .min()
            .unwrap();

        let now_gpst = min_gpst + 0.01;
        log::warn!("----- now_gpst={now_gpst:?}");
        for eph in ephs {
            let e_gpst = eph.tow_gpst + Duration::from_seconds(ts_sec - eph.ts_sec);
            let pseudo_range_sec = (e_gpst - min_gpst).to_seconds() + eph.code_off_sec;
            let pseudo_range = pseudo_range_sec * SPEED_OF_LIGHT;
            let dt = (now_gpst - eph.tow_gpst).to_seconds();
            let clock_corr = eph.f0 + eph.f1 * dt + eph.f2 * dt.powi(2);
            assert!(dt >= 0.0);

            log::warn!("{} - e_gpst={:?} eph.ts={}", eph.sv, e_gpst, eph.ts_sec);
            log::warn!(
                "{} - prng={pseudo_range_sec:+e}sec/{pseudo_range:.1}m tgd={:+e} clock_corr={clock_corr}",
                eph.sv,
                eph.tgd,
            );

            let candidate = Candidate::new(
                eph.sv,
                now_gpst,
                Duration::from_seconds(0.0),
                Some(Duration::from_seconds(eph.tgd)),
                vec![Observation {
                    carrier: Carrier::L1,
                    value: pseudo_range,
                    snr: Some(eph.cn0),
                }],
                vec![],
                vec![],
            );

            pool.push(candidate);
        }

        let (tropo_bias, iono_bias) = get_tropo_iono_bias();
        let res = self
            .solver
            .resolve(now_gpst, &pool, &iono_bias, &tropo_bias);

        match res {
            Err(err) => log::warn!("Failed to get a position: {err}"),
            Ok(solution) => {
                let pos = solution.1.position;
                let (lat_rad, lon_rad, h) = ecef2geodetic(pos[0], pos[1], pos[2], Ellipsoid::WGS84);
                let lat = lat_rad * 180.0 / PI;
                let lon = lon_rad * 180.0 / PI;
                let height = h / 1000.0;

                self.pub_state.lock().unwrap().latitude = lat;
                self.pub_state.lock().unwrap().longitude = lon;
                self.pub_state.lock().unwrap().height = height;

                log::warn!(
                    "{}",
                    format!("XXX: lat/lon: {:.4},{:.4} h={:.1}", lat, lon, height).red(),
                );
            }
        }
    }
}
