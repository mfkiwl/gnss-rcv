use colored::Colorize;
use gnss_rs::sv::SV;
use rayon::prelude::*;
use rustfft::num_complex::Complex64;
use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use crate::channel::Channel;
use crate::solver::PositionSolver;

const PERIOD_RCV: f64 = 0.001;

pub type ReadIQFn = dyn FnMut(usize, usize) -> Result<Vec<Complex64>, Box<dyn std::error::Error>>;

pub struct Receiver {
    read_iq_fn: Box<ReadIQFn>,
    period_sp: usize, // samples per period
    off_samples: usize,
    cached_iq_vec: Vec<Complex64>,
    cached_ts_sec_tail: f64,
    channels: HashMap<SV, Channel>,
    solver: PositionSolver,
    last_fix_sec: f64,
}

impl Receiver {
    pub fn new(
        read_iq_fn: Box<ReadIQFn>,
        fs: f64,
        fi: f64,
        off_msec: usize,
        sig: &str,
        sat_vec: &[SV],
    ) -> Self {
        let period_sp = (PERIOD_RCV * fs) as usize;
        let mut channels = HashMap::<SV, Channel>::new();

        for sv in sat_vec {
            channels.insert(*sv, Channel::new(sig, *sv, fs, fi));
        }

        Self {
            read_iq_fn,
            period_sp,
            off_samples: off_msec * period_sp,
            cached_iq_vec: Vec::<Complex64>::new(),
            cached_ts_sec_tail: 0.0,
            channels,
            solver: PositionSolver::new(),
            last_fix_sec: 0.0,
        }
    }

    fn fetch_samples_msec(&mut self) -> Result<(Vec<Complex64>, f64), Box<dyn std::error::Error>> {
        let num_samples = if self.cached_iq_vec.is_empty() {
            2 * self.period_sp
        } else {
            self.period_sp
        };

        let mut iq_vec = (self.read_iq_fn)(self.off_samples, num_samples)?;

        self.off_samples += num_samples;
        self.cached_iq_vec.append(&mut iq_vec);
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

        Ok((
            self.cached_iq_vec[len - 2 * self.period_sp..].to_vec(),
            self.cached_ts_sec_tail - 0.001,
        ))
    }

    fn compute_fix(&mut self, ts_sec: f64) {
        if ts_sec - self.last_fix_sec < 2.0 {
            return;
        }

        let ephs: Vec<_> = self
            .channels
            .values()
            .filter(|&ch| ch.is_ephemeris_complete())
            .map(|ch| ch.nav.eph)
            .collect();

        if ephs.len() < 4 {
            return;
        }

        log::warn!(
            "t={ts_sec:.3} -- {}",
            format!("attempting fix with {} SVs", ephs.len()).red()
        );

        self.solver.compute_position(ts_sec, &ephs);
        self.last_fix_sec = ts_sec;
    }

    fn process_step(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let (iq_vec, ts_sec) = self.fetch_samples_msec()?;

        self.channels
            .par_iter_mut()
            .for_each(|(_id, channel)| channel.process_samples(&iq_vec, ts_sec));

        self.compute_fix(ts_sec);

        Ok(())
    }

    pub fn run_loop(&mut self, num_msec: usize, exit_req: Arc<AtomicBool>) {
        let mut n = 0;
        loop {
            if self.process_step().is_err() {
                break;
            }
            if exit_req.load(Ordering::SeqCst) {
                break;
            }
            n += 1;
            if num_msec != 0 && n >= num_msec {
                break;
            }
        }
    }
}
