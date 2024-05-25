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
use std::str::FromStr;
use std::sync::Mutex;
use std::time::Instant;

use crate::channel::Channel;
use crate::navigation::Ephemeris;
use crate::recording::IQRecording;
use crate::types::IQSample;

lazy_static! {
    static ref EPHEMERIS_MAP: Mutex<HashMap<u8, Ephemeris>> = Mutex::new(HashMap::new());
}

const PERIOD_RCV: f64 = 0.001;

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
        let mean_anomaly_at_reference_time = eph.m0;
        let mean_anomaly_now =
            mean_anomaly_at_reference_time + corrected_mean_motion * time_from_ephemeris_ref_t;

        let mut eccentric_anomaly_now_estimation = mean_anomaly_now;
        for _i in 0..7 {
            eccentric_anomaly_now_estimation =
                mean_anomaly_now + (eph.ecc * eccentric_anomaly_now_estimation.sin())
        }

        eccentric_anomaly_now_estimation
    }

    fn compute_sv_position_ecef(eph: &Ephemeris, gps_time: f64) -> [f64; 3] {
        const EARTH_ROTATION_RATE: f64 = 7.2921151467e-5;
        let cuc = eph.cuc;
        let cus = eph.cus;
        let crc = eph.crc;
        let crs = eph.crs;
        let cic = eph.cic;
        let cis = eph.cis;
        let ecc = eph.ecc;
        let i0 = eph.i0;
        let idot = eph.idot;
        let a = eph.a;
        let omg = eph.omg;
        let omgd = eph.omgd;
        let omg0 = eph.omg0;
        let toes = eph.toes;

        let time_from_eph_t = gps_time - eph.toes;

        let ecc_anomaly = Self::get_eccentric_anomaly(eph, time_from_eph_t);
        let true_anomaly =
            ((1.0 - (ecc * ecc)).sqrt() * ecc_anomaly.sin()).atan2(ecc_anomaly.cos() - ecc);

        let vk = true_anomaly;

        let arg_lat = vk + omg;
        let duk = (cus * (2.0 * arg_lat).sin()) + (cuc * (2.0 * arg_lat).cos());
        let drk = (crs * (2.0 * arg_lat).sin()) + (crc * (2.0 * arg_lat).cos());
        let dik = (cis * (2.0 * arg_lat).sin()) + (cic * (2.0 * arg_lat).cos());

        let uk = arg_lat + duk;
        let rk = a * (1.0 - (ecc * ecc_anomaly.cos())) + drk;
        let ik = i0 + (idot * time_from_eph_t) + dik;

        let orb_plane_x = rk * uk.cos();
        let orb_plane_y = rk * uk.sin();

        let omega_at_ref_time =
            omg0 + (omgd - EARTH_ROTATION_RATE) * time_from_eph_t - (EARTH_ROTATION_RATE * toes);

        let ecef_x = orb_plane_x * omega_at_ref_time.cos()
            - (orb_plane_y * ik.cos() * omega_at_ref_time).sin();

        let ecef_y = (orb_plane_x * omega_at_ref_time.sin())
            + (orb_plane_y * ik.cos() * omega_at_ref_time.cos());

        let ecef_z = orb_plane_y * ik.sin();

        [ecef_x, ecef_y, ecef_z]
    }

    fn sv_interpolator(t: Epoch, sv: SV, size: usize) -> Option<InterpolationResult> {
        log::warn!("{sv}: sv_interpolator for {t} sz={size}");

        let map = EPHEMERIS_MAP.lock().unwrap();
        let eph = map.get(&sv.prn).unwrap();
        let pos = Self::compute_sv_position_ecef(eph, t.to_gpst_seconds());

        Some(InterpolationResult::from_apc_position((
            pos[0], pos[1], pos[2],
        )))
    }

    fn compute_fix(&mut self) {
        if self.last_fix_sec.elapsed().as_secs_f32() < 2.0 {
            return;
        }
        log::warn!(
            "t={:.3} -- {}",
            self.cached_ts_sec_tail,
            format!("attempting fix").red()
        );

        // somewhere in the middle of Lake Leman
        let initial = AprioriPosition::from_geo(Vector3::new(46.5, 6.6, 0.0));
        let cfg = Config::static_preset(Method::SPP);

        let gps_time = Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap();
        let mut pool: Vec<Candidate> = vec![]; // XXX
        let mut max_tow = 3600 * 24 * 7;

        {
            let mut map = EPHEMERIS_MAP.lock().unwrap();
            map.clear();

            for (sv, channel) in &self.channels {
                if channel.nav.eph.tow != 0 && channel.nav.eph.tow < max_tow {
                    max_tow = channel.nav.eph.tow;
                    log::warn!("best: {}: tow={max_tow}", sv.prn);
                }

                map.insert(sv.prn, channel.nav.eph);
                let candidate = Candidate::new(
                    SV::new(gnss_rs::constellation::Constellation::GPS, sv.prn),
                    Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap(),
                    Duration::from_seconds(0.162520179759E-04),
                    Some(Duration::from_nanoseconds(10.0)),
                    vec![Observation {
                        carrier: Carrier::L1,
                        value: 1.0E6_f64,
                        snr: None,
                    }],
                    vec![Observation {
                        carrier: Carrier::L1,
                        value: channel.trk.doppler_hz,
                        snr: None,
                    }],
                    vec![Observation {
                        carrier: Carrier::L1,
                        value: channel.trk.doppler_hz,
                        snr: None,
                    }],
                );

                pool.push(candidate);
            }
        }

        let (tropo_bias, iono_bias) = Self::get_tropo_iono_bias();
        let mut solver = Solver::new(&cfg, initial, Self::sv_interpolator).expect("Solver issue");
        let solutions = solver.resolve(gps_time, &pool, &iono_bias, &tropo_bias);
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
