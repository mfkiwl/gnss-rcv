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
use crate::navigation::Ephemeris;
use crate::recording::IQRecording;
use crate::types::IQSample;

const PERIOD_RCV: f64 = 0.001;
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

pub struct Receiver {
    pub recording: IQRecording,
    period_sp: usize, // samples per period
    fs: f64,
    fi: f64,
    off_samples: usize,
    cached_iq_vec: Vec<Complex64>,
    cached_ts_sec_tail: f64,
    channels: HashMap<SV, Channel>,
    last_fix_sec: Instant,
}

impl Drop for Receiver {
    fn drop(&mut self) {}
}

impl Receiver {
    pub fn new(recording: IQRecording, fs: f64, fi: f64, off_msec: usize) -> Self {
        let period_sp = (PERIOD_RCV * fs) as usize;
        Self {
            recording,
            period_sp,
            fs,
            fi,
            off_samples: off_msec * period_sp,
            cached_iq_vec: Vec::<Complex64>::new(),
            cached_ts_sec_tail: 0.0,
            channels: HashMap::<SV, Channel>::new(),
            last_fix_sec: Instant::now(),
        }
    }

    pub fn init(&mut self, sig: &str, sat_vec: Vec<SV>) {
        for sv in sat_vec {
            self.channels
                .insert(sv, Channel::new(sig, sv, self.fs, self.fi));
        }
    }

    fn fetch_samples_msec(&mut self) -> Result<IQSample, Box<dyn std::error::Error>> {
        let num_samples = if self.cached_iq_vec.len() == 0 {
            2 * self.period_sp
        } else {
            self.period_sp
        };
        let mut sample = self.recording.read_iq_file(self.off_samples, num_samples)?;

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
        Ok(IQSample {
            iq_vec: self.cached_iq_vec[len - 2 * self.period_sp..].to_vec(),
            ts_sec: self.cached_ts_sec_tail - 0.001,
        })
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

    fn get_eccentric_anomaly(eph: &Ephemeris, time_from_ephemeris_ref_t: f64) -> f64 {
        let earth_gravitational_constant = 3.986004418e14;

        let computed_mean_motion = earth_gravitational_constant / eph.a.powf(1.5);
        let corrected_mean_motion = computed_mean_motion + eph.deln;

        let mean_anomaly_now = eph.m0 + corrected_mean_motion * time_from_ephemeris_ref_t;

        let mut eccentric_anomaly = mean_anomaly_now;

        for _i in 0..7 {
            eccentric_anomaly = eccentric_anomaly
                + (mean_anomaly_now - eccentric_anomaly + eph.ecc * eccentric_anomaly.sin())
                    / (1.0 - eph.ecc * eccentric_anomaly.cos())
        }

        eccentric_anomaly
    }

    fn compute_sv_position_ecef(sv: SV, eph: &Ephemeris, gps_time: f64) -> [f64; 3] {
        const EARTH_ROTATION_RATE: f64 = 7.2921151467e-5;
        let mut time_from_eph_t = gps_time - eph.toe_gpst.to_gpst_seconds();

        log::warn!("{sv}: delta-t={time_from_eph_t} toe={}", eph.toe_gpst);

        if time_from_eph_t > 302400.0 {
            time_from_eph_t -= 604800.0;
        }
        if time_from_eph_t < -302400.0 {
            time_from_eph_t += 604800.0;
        }

        let ecc_anomaly = Self::get_eccentric_anomaly(eph, time_from_eph_t);
        // XXX not the math in the GPS doc
        let true_anomaly = ((1.0 - (eph.ecc * eph.ecc)).sqrt() * ecc_anomaly.sin())
            .atan2(ecc_anomaly.cos() - eph.ecc);

        let vk = true_anomaly;

        let arg_lat = vk + eph.omg;
        let duk = eph.cus * (2.0 * arg_lat).sin() + eph.cuc * (2.0 * arg_lat).cos();
        let drk = eph.crs * (2.0 * arg_lat).sin() + eph.crc * (2.0 * arg_lat).cos();
        let dik = eph.cis * (2.0 * arg_lat).sin() + eph.cic * (2.0 * arg_lat).cos();

        let uk = arg_lat + duk;
        let rk = eph.a * (1.0 - eph.ecc * ecc_anomaly.cos()) + drk;
        let ik = eph.i0 + eph.idot * time_from_eph_t + dik;

        let orb_plane_x = rk * uk.cos();
        let orb_plane_y = rk * uk.sin();

        let omega_at_ref_time = eph.omg0 + (eph.omgd - EARTH_ROTATION_RATE) * time_from_eph_t
            - EARTH_ROTATION_RATE * eph.toe;

        let ecef_x = orb_plane_x * omega_at_ref_time.cos()
            - orb_plane_y * ik.cos() * omega_at_ref_time.sin();

        let ecef_y = orb_plane_x * omega_at_ref_time.sin()
            + orb_plane_y * ik.cos() * omega_at_ref_time.cos();

        let ecef_z = orb_plane_y * ik.sin();

        log::warn!(
            "{}: position: x={:.1} y={:.1} z={:.1} h={:.0}:",
            sv,
            ecef_x,
            ecef_y,
            ecef_z,
            (ecef_x * ecef_x + ecef_y * ecef_y + ecef_z * ecef_z).sqrt()
        );

        [ecef_x, ecef_y, ecef_z]
    }

    fn compute_fix(&mut self) {
        if self.last_fix_sec.elapsed().as_secs_f32() < 2.0 {
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
            self.last_fix_sec = Instant::now();
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
            if channel.trk.cn0 < 35.0 || channel.nav.eph.week == 0 || channel.nav.eph.toe == 0.0 {
                continue;
            }

            let sv_ts = channel.nav.eph.ts_gpst.to_gpst_seconds() + ts_sec - channel.nav.eph.ts_sec;
            log::warn!("{sv} has ts_sec={sv_ts:.4}");

            if sv_ts < ts_ref {
                ts_ref = sv_ts;
                log::warn!("{sv} is the ref")
            }
        }

        for (sv, channel) in &self.channels {
            if channel.trk.cn0 < 35.0 || channel.nav.eph.week == 0 || channel.nav.eph.toe == 0.0 {
                continue;
            }
            const BASE_DELAY: f64 = 68.802e-3;
            let sv_ts = channel.nav.eph.ts_gpst.to_gpst_seconds() + ts_sec - channel.nav.eph.ts_sec;
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
            let pos = Self::compute_sv_position_ecef(sv, &ch.nav.eph, t.to_gpst_seconds());

            Some(InterpolationResult::from_apc_position(pos.into()))
        };

        let mut solver = Solver::new(&cfg, initial, sv_interp).expect("Solver issue");
        let solutions = solver.resolve(ts_solve, &pool, &iono_bias, &tropo_bias);
        if solutions.is_ok() {
            log::warn!("got a fix: {:?}", solutions)
        } else {
            log::warn!("Failed to get a fix: {}", solutions.err().unwrap());
        }

        self.last_fix_sec = Instant::now();
    }

    pub fn process_step(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let samples = self.fetch_samples_msec()?;

        self.channels
            .par_iter_mut()
            .for_each(|(_id, sat)| sat.process_samples(&samples.iq_vec, samples.ts_sec));

        self.compute_fix();

        Ok(())
    }
}
