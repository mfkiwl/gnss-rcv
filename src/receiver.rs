use gnss_rs::sv::SV;
use gnss_rtk::prelude::{
    AprioriPosition, Candidate, Carrier, Config, Epoch, InterpolationResult, IonosphereBias,
    Method, Observation, Solver, TroposphereBias, Vector3,
};

use colored::Colorize;
use gnss_rtk::prelude::Duration;

use rayon::prelude::*;
use rustfft::num_complex::Complex64;
use std::collections::HashMap;
use std::time::Instant;

use crate::channel::Channel;
use crate::ephemeris::Ephemeris;
use crate::types::IQSample;

const PERIOD_RCV: f64 = 0.001;
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

pub struct Receiver {
    read_iq_fn: Box<dyn FnMut(usize, usize)->Result<Vec<Complex64>, Box<dyn std::error::Error>>>,
    period_sp: usize, // samples per period
    fs: f64,
    fi: f64,
    off_samples: usize,
    cached_iq_vec: Vec<Complex64>,
    cached_ts_sec_tail: f64,
    channels: HashMap<SV, Channel>,
    last_fix_sec: f64,
}

impl Receiver {
    pub fn new(read_iq_fn: Box<dyn FnMut(usize, usize)->Result<Vec<Complex64>, Box<dyn std::error::Error>>>, fs: f64, fi: f64, off_msec: usize) -> Self {
        let period_sp = (PERIOD_RCV * fs) as usize;
        Self {
            read_iq_fn,
            period_sp,
            fs,
            fi,
            off_samples: off_msec * period_sp,
            cached_iq_vec: Vec::<Complex64>::new(),
            cached_ts_sec_tail: 0.0,
            channels: HashMap::<SV, Channel>::new(),
            last_fix_sec: 0.0,
        }
    }

    pub fn init(&mut self, sig: &str, sat_vec: Vec<SV>) {
        for sv in sat_vec {
            self.channels
                .insert(sv, Channel::new(sig, sv, self.fs, self.fi));
        }
    }

    fn fetch_samples_msec(&mut self) -> (Result<IQSample, Box<dyn std::error::Error>>,u128) {
        let num_samples = if self.cached_iq_vec.len() == 0 {
            2 * self.period_sp
        } else {
            self.period_sp
        };

        let ts = Instant::now();
        let iq_vec = (self.read_iq_fn)(self.off_samples, num_samples).unwrap();
        let elapsed = ts.elapsed().as_millis();
        let mut sample = IQSample {
            iq_vec,
            ts_sec: self.off_samples as f64 / self.fs,
        };

        self.off_samples += num_samples;
        self.cached_iq_vec.append(&mut sample.iq_vec);
        self.cached_ts_sec_tail += num_samples as f64 / (1000.0 * self.period_sp as f64);

        if self.cached_iq_vec.len() > 2 * self.period_sp {
            let num_samples = self.period_sp;
            let _ = self.cached_iq_vec.drain(0..num_samples);
        }
        let len = self.cached_iq_vec.len();

        // we pass 2 code worth of iq data back
        // the timestamp given corresponds to the beginning of the last code
        // [...code...][...code...]
        //             ^
        (Ok(IQSample {
            iq_vec: self.cached_iq_vec[len - 2 * self.period_sp..].to_vec(),
            ts_sec: self.cached_ts_sec_tail - 0.001,
        }), elapsed)
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

    fn get_eccentric_anomaly(eph: &Ephemeris, t_k: f64) -> f64 {
        // earth gravitational constant
        const EARTH_MU_GPS: f64 = 3.9860058e14;

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
        log::warn!("n_iter={n_iter}");
        assert!(n_iter < 20);

        e
    }

    fn compute_sv_position_ecef(sv: SV, eph: &Ephemeris, t: Epoch) -> [f64; 3] {
        const EARTH_ROTATION_RATE: f64 = 7.2921151467e-5;
        let mut time_from_eph_t = (t - eph.toe_gpst).to_seconds();

        log::warn!("{sv}: delta-t={time_from_eph_t} toe={}", eph.toe_gpst);

        if time_from_eph_t > 302400.0 {
            time_from_eph_t -= 604800.0;
        }
        if time_from_eph_t < -302400.0 {
            time_from_eph_t += 604800.0;
        }

        let ecc_anomaly = Self::get_eccentric_anomaly(eph, time_from_eph_t);

        let v_k = ((1.0 - (eph.ecc * eph.ecc)).sqrt() * ecc_anomaly.sin())
            .atan2(ecc_anomaly.cos() - eph.ecc);

        let phi_k = v_k + eph.omg;
        let duk = eph.cus * (2.0 * phi_k).sin() + eph.cuc * (2.0 * phi_k).cos();
        let drk = eph.crs * (2.0 * phi_k).sin() + eph.crc * (2.0 * phi_k).cos();
        let dik = eph.cis * (2.0 * phi_k).sin() + eph.cic * (2.0 * phi_k).cos();

        let uk = phi_k + duk;
        let rk = eph.a * (1.0 - eph.ecc * ecc_anomaly.cos()) + drk;
        let ik = eph.i0 + eph.i_dot * time_from_eph_t + dik;

        let orb_plane_x = rk * uk.cos();
        let orb_plane_y = rk * uk.sin();

        let omega = eph.omg0 + (eph.omg_dot - EARTH_ROTATION_RATE) * time_from_eph_t
            - EARTH_ROTATION_RATE * eph.toe as f64;

        let ecef_x = orb_plane_x * omega.cos() - orb_plane_y * ik.cos() * omega.sin();
        let ecef_y = orb_plane_x * omega.sin() + orb_plane_y * ik.cos() * omega.cos();
        let ecef_z = orb_plane_y * ik.sin();

        log::warn!(
            "{sv}: position: x={ecef_x:.1} y={ecef_y:.1} z={ecef_z:.1} h={:.0}km:",
            (ecef_x * ecef_x + ecef_y * ecef_y + ecef_z * ecef_z).sqrt() / 1000.0
        );

        [ecef_x, ecef_y, ecef_z]
    }

    fn compute_fix(&mut self, ts: f64) {
        if ts - self.last_fix_sec < 2.0 {
            return;
        }
        let ts_sec = self.cached_ts_sec_tail - 0.001;
        log::warn!("t={ts_sec:.3} -- {}", format!("attempting fix").red());

        // somewhere in the middle of Lake Leman
        let initial = AprioriPosition::from_geo(Vector3::new(46.5, 6.6, 0.0));
        let cfg = Config::static_preset(Method::SPP);

        let mut pool: Vec<Candidate> = vec![]; // XXX
        let mut num_eph_complete = 0;

        for (_sv, channel) in &self.channels {
            if channel.trk.cn0 > 35.0 && channel.nav.eph.ts_sec != 0.0 {
                num_eph_complete += 1;
            }
        }

        if num_eph_complete < 4 {
            log::warn!("only {num_eph_complete} sats with appropriate ephemeris");
            self.last_fix_sec = ts;
            return;
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
         *
         *  sat0      []                   ~0
         *  sat1      [------]
         *  sat2      [-------------]
         */

        let mut ts_ref = f64::MAX;

        for (sv, channel) in &self.channels {
            if channel.trk.cn0 < 35.0
                || channel.nav.eph.week == 0
                || channel.nav.eph.toe == 0
                || channel.nav.eph.a <= 20000000.0
            {
                log::warn!(
                    "{sv} unfit: week={} toe={} a={:.1}",
                    channel.nav.eph.week,
                    channel.nav.eph.toe,
                    channel.nav.eph.a
                );
                continue;
            }

            let sv_ts =
                channel.nav.eph.tow_gpst.to_gpst_seconds() + ts_sec - channel.nav.eph.ts_sec;

            if sv_ts < ts_ref {
                ts_ref = sv_ts;
            }
        }

        for (sv, channel) in &self.channels {
            if channel.trk.cn0 < 35.0
                || channel.nav.eph.week == 0
                || channel.nav.eph.toe == 0
                || channel.nav.eph.a < 20000000.0
            {
                continue;
            }
            const BASE_DELAY: f64 = 68.802e-3;
            let sv_ts =
                channel.nav.eph.tow_gpst.to_gpst_seconds() + ts_sec - channel.nav.eph.ts_sec;
            let pseudo_range_sec = sv_ts - ts_ref;

            log::warn!(
                "{sv} - cn0={:.1} sv_ts={sv_ts} pseudo_range_sec={pseudo_range_sec}",
                channel.trk.cn0
            );
            assert!(pseudo_range_sec >= 0.0);

            let candidate = Candidate::new(
                SV::new(gnss_rs::constellation::Constellation::GPS, sv.prn),
                Epoch::from_gpst_seconds(sv_ts),
                Duration::from_seconds(0.0), // TBD
                Some(Duration::from_seconds(channel.nav.eph.tgd)),
                vec![Observation {
                    carrier: Carrier::L1,
                    value: SPEED_OF_LIGHT * (BASE_DELAY + pseudo_range_sec),
                    snr: Some(channel.trk.cn0),
                }],
                vec![], // not required for SPP
                vec![], // not used
            );

            pool.push(candidate);
        }

        let (tropo_bias, iono_bias) = Self::get_tropo_iono_bias();
        let ts_solve = Epoch::from_gpst_seconds(ts_ref);
        let sv_interp = |t: Epoch, sv: SV, _size: usize| {
            let ch = self.channels.get(&sv).unwrap();
            let pos = Self::compute_sv_position_ecef(sv, &ch.nav.eph, t);

            Some(InterpolationResult::from_apc_position(pos.into()))
        };

        let mut solver = Solver::new(&cfg, initial, sv_interp).expect("Solver issue");
        let solutions = solver.resolve(ts_solve, &pool, &iono_bias, &tropo_bias);
        if solutions.is_ok() {
            log::warn!("got a fix: {:?}", solutions)
        } else {
            log::warn!("Failed to get a fix: {}", solutions.err().unwrap());
        }

        self.last_fix_sec = ts;
    }

    pub fn process_step(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let ts = Instant::now();
        let (samples, e) = self.fetch_samples_msec();
        let fetch_ts = ts.elapsed().as_millis();
        let samples = samples.unwrap();

        self.channels
            .par_iter_mut()
            .for_each(|(_id, channel)| channel.process_samples(&samples.iq_vec, samples.ts_sec));

        log::warn!("receiver: ts={:.3} -- took {} / {e} -- {} msec",
                    samples.ts_sec, fetch_ts, ts.elapsed().as_millis());
        //self.compute_fix(samples.ts_sec);

        Ok(())
    }
}
