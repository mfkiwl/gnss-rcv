use gnss_rs::sv::SV;
use gnss_rtk::prelude::{
    AprioriPosition, Candidate, Carrier, Config, Duration, Epoch, InterpolationResult,
    IonosphereBias, Method, Observation, Solver, TroposphereBias, Vector3,
};

use crate::ephemeris::Ephemeris;

const SPEED_OF_LIGHT: f64 = 299_792_458.0;
const EARTH_MU_GPS: f64 = 3.9860058e14; // earth gravitational constant
const EARTH_ROTATION_RATE: f64 = 7.2921151467e-5;

pub struct PositionSolver {
    cfg: Config,
}

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
    log::warn!("n_iter={n_iter} e={e}");
    assert!(n_iter < 20);

    e
}

pub fn compute_sv_position_ecef(eph: &Ephemeris, t: Epoch) -> (f64, f64, f64) {
    let mut dte = (t - eph.toe_gpst).to_seconds();

    log::warn!("{}: now={t}", eph.sv);
    log::warn!("{}: toe={} delta-t={dte} ", eph.sv, eph.toe_gpst);

    if dte > 302400.0 {
        dte -= 604800.0;
    }
    if dte < -302400.0 {
        dte += 604800.0;
    }

    let ecc_anomaly = get_eccentric_anomaly(eph, dte);

    let v_k =
        ((1.0 - (eph.ecc * eph.ecc)).sqrt() * ecc_anomaly.sin()).atan2(ecc_anomaly.cos() - eph.ecc);

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
        "{}: position: x={ecef_x:.1} y={ecef_y:.1} z={ecef_z:.1} h={:.0}km:",
        eph.sv,
        (ecef_x * ecef_x + ecef_y * ecef_y + ecef_z * ecef_z).sqrt() / 1000.0
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

impl PositionSolver {
    pub fn new() -> Self {
        Self {
            cfg: Config::static_preset(Method::SPP),
        }
    }

    pub fn get_position(&mut self, ts_sec: f64, ephs: &Vec<Ephemeris>) -> bool {
        // somewhere in the middle of Lake Leman
        let initial = AprioriPosition::from_geo(Vector3::new(46.5, 6.6, 0.0));
        let mut pool: Vec<Candidate> = vec![];

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

        let ts_ref = ephs.iter().map(|e| e.ts_sec).reduce(f64::min).unwrap();
        let now_gpst = ephs[0].tow_gpst + Duration::from_seconds(ts_sec - ephs[0].ts_sec);

        for eph in ephs {
            let pseudo_range_sec = eph.ts_sec - ts_ref;

            log::warn!(
                "{} - tow={} eph.ts={} pseudo_range_sec={pseudo_range_sec}",
                eph.sv,
                eph.tow,
                eph.ts_sec,
            );
            assert!(pseudo_range_sec >= 0.0);

            let candidate = Candidate::new(
                eph.sv,
                now_gpst,
                Duration::from_seconds(0.0), // TBD
                Some(Duration::from_seconds(eph.tgd)),
                vec![Observation {
                    carrier: Carrier::L1,
                    value: SPEED_OF_LIGHT * pseudo_range_sec,
                    snr: Some(35.0),
                }],
                vec![], // not required for SPP
                vec![], // not used
            );

            pool.push(candidate);
        }

        let (tropo_bias, iono_bias) = get_tropo_iono_bias();
        let sv_interp = |t: Epoch, sv: SV, _size: usize| {
            let eph = ephs.iter().find(|&&e| e.sv == sv).unwrap();
            let pos = compute_sv_position_ecef(eph, t);

            Some(InterpolationResult::from_apc_position(pos))
        };

        let mut solver = Solver::new(&self.cfg, initial, sv_interp).expect("Solver issue");
        let solutions = solver.resolve(now_gpst, &pool, &iono_bias, &tropo_bias);
        let res = solutions.is_ok();
        if res {
            log::warn!("POSITION: {solutions:?}");
        } else {
            log::warn!("Failed to get a fix: {}", solutions.err().unwrap());
        }
        res
    }
}
