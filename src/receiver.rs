use colored::Colorize;
use rayon::prelude::*;
use rustfft::{num_complex::Complex64, FftPlanner};
use std::collections::HashMap;
use std::collections::HashSet;
use std::time::Instant;

use crate::acquisition::try_acquisition_one_sat;
use crate::constants::ACQUISITION_WINDOW_MSEC;
use crate::gold_code::GoldCode;
use crate::recording::IQRecording;
use crate::satellite::GnssSatellite;
use crate::types::GnssCorrelationParam;
use crate::types::IQSample;
use crate::util::get_num_samples_per_msec;

const ACQUISITION_PERIOD_SEC: f64 = 3.0; // run acquisition every 3sec

pub struct GnssReceiver {
    gold_code: GoldCode,
    pub iq_recording: IQRecording,
    sat_vec: Vec<usize>,
    off_samples: usize,
    cached_iq_vec: Vec<Complex64>,
    cached_num_msec: usize,
    cached_ts_sec_tail: f64,
    last_acq_ts_sec: f64,
    satellites: HashMap<usize, GnssSatellite>,
    satellites_found: HashSet<usize>,
}

impl Drop for GnssReceiver {
    fn drop(&mut self) {
        if self.satellites_found.is_empty() {
            return;
        }
        log::warn!("found {} satellites", self.satellites_found.len());
        for prn in &self.satellites_found {
            log::warn!("sat-{}", *prn)
        }
    }
}

impl GnssReceiver {
    pub fn new(
        gold_code: GoldCode,
        iq_recording: IQRecording,
        off_msec: usize,
        sat_vec: Vec<usize>,
    ) -> Self {
        Self {
            gold_code,
            iq_recording,
            sat_vec,
            off_samples: off_msec * get_num_samples_per_msec(),
            last_acq_ts_sec: 0.0,
            cached_iq_vec: Vec::<Complex64>::new(),
            cached_num_msec: 0,
            cached_ts_sec_tail: 0.0,
            satellites: HashMap::<usize, GnssSatellite>::new(),
            satellites_found: HashSet::<usize>::new(),
        }
    }

    fn try_periodic_acquisition(&mut self) {
        if self.cached_num_msec < ACQUISITION_WINDOW_MSEC {
            return;
        }

        if self.cached_ts_sec_tail < self.last_acq_ts_sec + ACQUISITION_PERIOD_SEC {
            return;
        }

        let ts = Instant::now();
        let cached_vec_len = self.cached_iq_vec.len();
        let num_samples = ACQUISITION_WINDOW_MSEC * get_num_samples_per_msec();

        log::warn!(
            "--- try_acq: cached_ts_sec_tail={:.3} sec cached_vec_len={}",
            format!("{}", self.cached_ts_sec_tail).green(),
            cached_vec_len
        );

        let samples_vec = self.cached_iq_vec[cached_vec_len - num_samples..cached_vec_len].to_vec();
        let samples_ts_sec = self.cached_ts_sec_tail - ACQUISITION_WINDOW_MSEC as f64 / 1000.0;
        let mut new_sats = HashMap::<usize, GnssCorrelationParam>::new();

        let sample = IQSample {
            iq_vec: samples_vec,
            ts_sec: samples_ts_sec,
            sample_rate: self.iq_recording.sample_rate,
        };

        // Perform acquisition on each possible satellite in parallel
        let new_sats_vec_res: Vec<_> = self
            .sat_vec
            .par_iter()
            .map(|&id| {
                let mut fft_planner: FftPlanner<f64> = FftPlanner::new();
                try_acquisition_one_sat(&self.gold_code, &mut fft_planner, id, &sample)
            })
            .collect();

        // So far we've only collected an array of options, let's insert them
        // in the map for easy look-up.
        for (i, opt) in new_sats_vec_res.iter().enumerate() {
            let id = self.sat_vec[i];
            if let Some(param) = opt {
                new_sats.insert(id, *param);
            }
        }

        for (id, param) in &new_sats {
            match self.satellites.get_mut(id) {
                Some(sat) => sat.update_param(param, samples_ts_sec),
                None => {
                    let prn_code = self.gold_code.get_prn_code_upsampled_complex(*id);
                    let prn_code_fft = self.gold_code.get_prn_code_fft(*id);
                    self.satellites.insert(
                        *id,
                        GnssSatellite::new(*id, prn_code, prn_code_fft, *param, samples_ts_sec),
                    );
                    self.satellites_found.insert(*id);
                }
            }
        }
        let mut sat_removed = vec![];
        for id in self.satellites.keys() {
            if !new_sats.contains_key(&id) {
                log::warn!("{}", format!("sat {}: disappeared", id).red());
                sat_removed.push(id.clone());
            }
        }
        for id in sat_removed {
            self.satellites.remove(&id);
        }
        log::debug!("acquisition: {} msec", ts.elapsed().as_millis());
        self.last_acq_ts_sec = self.cached_ts_sec_tail;
    }

    fn fetch_samples_msec(
        &mut self,
        num_msec: usize,
    ) -> Result<IQSample, Box<dyn std::error::Error>> {
        let num_samples = num_msec * get_num_samples_per_msec();
        let mut sample = self
            .iq_recording
            .read_iq_file(self.off_samples, num_samples)?;

        assert!(sample.sample_rate > 0);

        self.off_samples += num_samples;
        self.cached_iq_vec.append(&mut sample.iq_vec);
        self.cached_ts_sec_tail += num_msec as f64 / 1000.0;
        self.cached_num_msec += num_msec;

        if self.cached_num_msec > ACQUISITION_WINDOW_MSEC {
            let num_msec_to_remove = self.cached_num_msec - ACQUISITION_WINDOW_MSEC;
            let num_samples = num_msec_to_remove * get_num_samples_per_msec();
            let _ = self.cached_iq_vec.drain(0..num_samples);
            self.cached_num_msec -= num_msec_to_remove;
            assert_eq!(self.cached_num_msec, ACQUISITION_WINDOW_MSEC);
        }
        let len = self.cached_iq_vec.len();
        Ok(IQSample {
            iq_vec: self.cached_iq_vec[len - num_samples..len].to_vec(),
            ts_sec: self.cached_ts_sec_tail - num_msec as f64 / 1000.0,
            sample_rate: sample.sample_rate,
        })
    }

    pub fn process_step(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let samples = self.fetch_samples_msec(1)?;
        self.try_periodic_acquisition();

        self.satellites
            .par_iter_mut()
            .for_each(|(_id, sat)| sat.process_samples(&samples));

        Ok(())
    }
}
