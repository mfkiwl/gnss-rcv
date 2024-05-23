use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::{
    AprioriPosition, Candidate, Config, Epoch, InterpolationResult, IonosphereBias, Method,
    PVTSolutionType, Solver, TroposphereBias, Vector3,
};
//use gnss_rtk::{Config, Error, Filter, Method, PVTSolutionType, Position, Solver, Vector3};

use gnss_rtk::prelude::Filter;
use rayon::prelude::*;
use rustfft::num_complex::Complex64;
use std::collections::HashMap;
use std::str::FromStr;
use std::time::Instant;

use crate::channel::Channel;
use crate::code::Code;
use crate::constants::ACQUISITION_WINDOW_MSEC;
use crate::recording::IQRecording;
use crate::types::IQSample;
use crate::util::get_num_samples_per_msec;

pub struct Receiver {
    gold_code: Code,
    pub recording: IQRecording,
    fs: f64,
    fi: f64,
    off_samples: usize,
    cached_iq_vec: Vec<Complex64>,
    cached_ts_sec_tail: f64,
    channels: HashMap<u8, Channel>,
    last_fix_sec: Instant,
}

impl Drop for Receiver {
    fn drop(&mut self) {}
}

impl Receiver {
    pub fn new(gold_code: Code, recording: IQRecording, fs: f64, fi: f64, off_msec: usize) -> Self {
        Self {
            gold_code,
            recording,
            fs,
            fi,
            off_samples: off_msec * get_num_samples_per_msec(),
            cached_iq_vec: Vec::<Complex64>::new(),
            cached_ts_sec_tail: 0.0,
            channels: HashMap::<u8, Channel>::new(),
            last_fix_sec: Instant::now(),
        }
    }

    pub fn init(&mut self, sat_vec: Vec<u8>) {
        for prn in sat_vec {
            self.channels.insert(
                prn,
                Channel::new(
                    Constellation::GPS,
                    prn as u8,
                    &mut self.gold_code,
                    self.fs,
                    self.fi,
                ),
            );
        }
    }

    fn fetch_samples_msec(&mut self) -> Result<IQSample, Box<dyn std::error::Error>> {
        let num_samples = if self.cached_iq_vec.len() == 0 {
            2 * get_num_samples_per_msec()
        } else {
            get_num_samples_per_msec()
        };
        let mut sample = self.recording.read_iq_file(self.off_samples, num_samples)?;

        self.off_samples += num_samples;
        self.cached_iq_vec.append(&mut sample.iq_vec);
        self.cached_ts_sec_tail +=
            num_samples as f64 / (1000.0 * get_num_samples_per_msec() as f64);

        if self.cached_iq_vec.len() > ACQUISITION_WINDOW_MSEC * get_num_samples_per_msec() {
            let num_samples = get_num_samples_per_msec();
            let _ = self.cached_iq_vec.drain(0..num_samples);
        }
        let len = self.cached_iq_vec.len();

        // we pass 2 code worth of iq data back
        // the timestamp given corresponds to the beginning of the last code
        // [...code...][...code...]
        //             ^
        Ok(IQSample {
            iq_vec: self.cached_iq_vec[len - 2 * get_num_samples_per_msec()..].to_vec(),
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

        let epoch = Epoch::from_str("2020-06-25T12:00:00 GPST").unwrap();
        let pool: Vec<Candidate> = vec![]; // XXX
        let (tropo_bias, iono_bias) = self.get_tropo_iono_bias();
        let mut solver = Solver::new(&cfg, initial, Self::sv_interpolator).expect("Solver issue");
        let solutions = solver.resolve(epoch, &pool, &iono_bias, &tropo_bias);
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
