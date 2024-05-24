use gnss_rs::sv::SV;
use gnss_rtk::prelude::{
    AprioriPosition, Candidate, Config, Epoch, InterpolationResult, IonosphereBias, Method,
    PVTSolutionType, Solver, TroposphereBias, Vector3,
};

use gnss_rtk::prelude::Filter;
use rayon::prelude::*;
use rustfft::num_complex::Complex64;
use std::collections::HashMap;
use std::str::FromStr;
use std::time::Instant;

use crate::channel::Channel;
use crate::recording::IQRecording;
use crate::types::IQSample;

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

    fn sv_interpolator(t: Epoch, sv: SV, size: usize) -> Option<InterpolationResult> {
        log::warn!("{sv}: sv_interpolator for {t} sz={size}");

        None
    }

    fn get_tropo_iono_bias(&mut self) -> (TroposphereBias, IonosphereBias) {
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

    fn get_eccentric_anomaly(&self, channel: &Channel, time_from_ephemeris_ref_t: f64) -> f64 {
        let earth_gravitational_constant = 3.986004418e14;

        let computed_mean_motion = earth_gravitational_constant / channel.nav.eph.a.powf(1.5);
        let corrected_mean_motion = computed_mean_motion + channel.nav.eph.deln;
        let mean_anomaly_at_reference_time = channel.nav.eph.m0;
        let mean_anomaly_now =
            mean_anomaly_at_reference_time + corrected_mean_motion * time_from_ephemeris_ref_t;

        let mut eccentric_anomaly_now_estimation = mean_anomaly_now;
        for _i in 0..7 {
            eccentric_anomaly_now_estimation =
                mean_anomaly_now + (channel.nav.eph.ecc * eccentric_anomaly_now_estimation.sin())
        }

        eccentric_anomaly_now_estimation
    }

    fn compute_sv_position_ecef(&self, prn: &u8, channel: &Channel, gps_time: f64) -> [f64; 3] {
        const EARTH_ROTATION_RATE: f64 = 7.2921151467e-5;
        let cuc = channel.nav.eph.cuc;
        let cus = channel.nav.eph.cus;
        let crc = channel.nav.eph.crc;
        let crs = channel.nav.eph.crs;
        let cic = channel.nav.eph.cic;
        let cis = channel.nav.eph.cis;
        let ecc = channel.nav.eph.ecc;
        let i0 = channel.nav.eph.i0;
        let idot = channel.nav.eph.idot;
        let a = channel.nav.eph.a;
        let omg = channel.nav.eph.omg;
        let omgd = channel.nav.eph.omgd;
        let omg0 = channel.nav.eph.omg0;
        let toes = channel.nav.eph.toes;

        let time_from_eph_t = gps_time - channel.nav.eph.toes;

        let ecc_anomaly = self.get_eccentric_anomaly(channel, time_from_eph_t);
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

    fn compute_fix(&mut self, ts_sec: f64) {
        if self.last_fix_sec.elapsed().as_secs_f32() < 2.0 {
            return;
        }
        log::warn!("t={:.3} -- attempting fix", ts_sec);

        // somewhere in the middle of Lake Leman
        let initial = AprioriPosition::from_geo(Vector3::new(46.5, 6.6, 0.0));

        let mut cfg = Config::static_preset(Method::SPP);

        cfg.min_snr = None;
        cfg.min_sv_elev = None;
        cfg.solver.filter = Filter::LSQ;
        cfg.sol_type = PVTSolutionType::PositionVelocityTime;

        let gps_time = Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap();
        let pool: Vec<Candidate> = vec![]; // XXX
        for (sv, channel) in &self.channels {
            if channel.num_tracking_samples < 1000 {
                continue;
            }
            let pos_ecef =
                self.compute_sv_position_ecef(&sv.prn, &channel, gps_time.to_gst_seconds());
        }

        let (tropo_bias, iono_bias) = self.get_tropo_iono_bias();
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

        self.compute_fix(samples.ts_sec);

        Ok(())
    }
}
