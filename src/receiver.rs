use rayon::prelude::*;
use rustfft::num_complex::Complex64;
use std::collections::HashMap;

use crate::channel::Channel;
use crate::code::Code;
use crate::constants::ACQUISITION_WINDOW_MSEC;
use crate::recording::IQRecording;
use crate::types::IQSample;
use crate::util::get_num_samples_per_msec;

pub struct GnssReceiver {
    gold_code: Code,
    pub recording: IQRecording,
    fs: f64,
    fi: f64,
    off_samples: usize,
    cached_iq_vec: Vec<Complex64>,
    cached_ts_sec_tail: f64,
    satellites: HashMap<usize, Channel>,
}

impl Drop for GnssReceiver {
    fn drop(&mut self) {}
}

impl GnssReceiver {
    pub fn new(gold_code: Code, recording: IQRecording, fs: f64, fi: f64, off_msec: usize) -> Self {
        Self {
            gold_code,
            recording,
            fs,
            fi,
            off_samples: off_msec * get_num_samples_per_msec(),
            cached_iq_vec: Vec::<Complex64>::new(),
            cached_ts_sec_tail: 0.0,
            satellites: HashMap::<usize, Channel>::new(),
        }
    }

    pub fn init(&mut self, sat_vec: Vec<usize>) {
        for sv in sat_vec {
            self.satellites
                .insert(sv, Channel::new(sv, &mut self.gold_code, self.fs, self.fi));
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

    pub fn process_step(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let samples = self.fetch_samples_msec()?;

        self.satellites
            .par_iter_mut()
            .for_each(|(_id, sat)| sat.process_samples(&samples.iq_vec, samples.ts_sec));

        Ok(())
    }
}
