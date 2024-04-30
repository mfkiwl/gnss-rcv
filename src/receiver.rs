use colored::Colorize;
use rayon::prelude::*;
use rustfft::{num_complex::Complex64, FftPlanner};
use std::collections::HashMap;
use std::ops::Mul;
use std::time::Instant;

use crate::gold_code::GoldCode;
use crate::recording::IQRecording;
use crate::satellite::GnssSatellite;
use crate::types::GnssCorrelationParam;
use crate::types::IQSample;
use crate::util::calc_correlation;
use crate::util::get_2nd_max;
use crate::util::get_max_with_idx;
use crate::util::get_num_samples_per_msec;

const DOPPLER_SPREAD_HZ: u32 = 8 * 1000;
const DOPPLER_SPREAD_BINS: u32 = 10;
const ACQUISITION_WINDOW_MSEC: usize = 10; // acquire on 10msec of data
const ACQUISITION_PERIOD_SEC: f64 = 3.0; // run acquisition every 3sec
const SNR_THRESHOLD: f64 = 3.0;
const PI: f64 = std::f64::consts::PI;

pub struct GnssReceiver {
    gold_code: GoldCode,
    pub iq_recording: IQRecording,
    sat_vec: Vec<usize>,
    off_samples: usize,
    last_acq_ts_sec: f64,
    cached_iq_vec: Vec<Complex64>,
    cached_num_msec: usize,
    cached_ts_sec_tail: f64,
    satellites: HashMap<usize, GnssSatellite>,
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
        }
    }

    fn calc_cross_correlation(
        &self,
        fft_planner: &mut FftPlanner<f64>,
        sample: &IQSample,
        sat_id: usize,
        num_msec: usize,
        estimate_hz: i32,
        spread_hz: u32,
    ) -> GnssCorrelationParam {
        let num_samples_per_msec = get_num_samples_per_msec();
        assert_eq!(sample.iq_vec.len(), num_samples_per_msec * num_msec);

        let mut best_param = GnssCorrelationParam::default();
        let prn_code_fft = self.gold_code.get_fft_code(sat_id);

        let lo_hz = estimate_hz - spread_hz as i32;
        let hi_hz = estimate_hz + spread_hz as i32 + 1;

        for doppler_hz in (lo_hz..hi_hz).step_by((spread_hz / DOPPLER_SPREAD_BINS) as usize) {
            let imaginary = -2.0 * PI * doppler_hz as f64;
            let mut b_corr = vec![0f64; get_num_samples_per_msec()];

            for idx in 0..num_msec {
                let sample_index = idx * num_samples_per_msec;
                let doppler_shift: Vec<_> = (0..num_samples_per_msec)
                    .map(|x| {
                        Complex64::from_polar(
                            1.0,
                            imaginary * (x + sample_index) as f64 / sample.sample_rate as f64,
                        )
                    })
                    .collect();
                let range_lo = (idx + 0) * num_samples_per_msec;
                let range_hi = (idx + 1) * num_samples_per_msec;
                let mut iq_vec_sample = Vec::from(&sample.iq_vec[range_lo..range_hi]);
                assert_eq!(iq_vec_sample.len(), doppler_shift.len());
                assert_eq!(iq_vec_sample.len(), prn_code_fft.len());

                for i in 0..iq_vec_sample.len() {
                    iq_vec_sample[i] = iq_vec_sample[i].mul(doppler_shift[i]);
                }

                let corr = calc_correlation(fft_planner, &iq_vec_sample, &prn_code_fft);
                for i in 0..corr.len() {
                    b_corr[i] += corr[i].norm();
                }
            }
            log::trace!(
                "  get_cross_correlation: {}-{} Hz -- trying {} Hz",
                estimate_hz - spread_hz as i32,
                estimate_hz + spread_hz as i32,
                doppler_hz,
            );

            let b_corr_norm = b_corr.iter().map(|&x| x * x).sum::<f64>();

            if b_corr_norm > best_param.corr_norm {
                let b_corr_second = get_2nd_max(&b_corr);
                let (idx, b_corr_peak) = get_max_with_idx(&b_corr);
                assert!(b_corr_peak != 0.0);
                assert!(b_corr_second != 0.0);
                // XXX: this results in faster acquisition and processing. Why?
                //let b_peak_to_second = 10.0 * (b_corr_peak / b_corr_second).log10();
                let b_peak_to_second =
                    10.0 * ((b_corr_peak - b_corr_second) / b_corr_second).log10();
                best_param.snr = b_peak_to_second;
                best_param.doppler_hz = doppler_hz as i32;
                best_param.phase_offset = idx;
                best_param.corr_norm = b_corr_norm;
                log::trace!(
                    "   best_doppler: {} Hz snr: {:+.1} idx={}",
                    doppler_hz,
                    b_peak_to_second,
                    idx / 2
                );
            }
        }
        best_param
    }

    fn try_acquisition_one_sat(
        &self,
        fft_planner: &mut FftPlanner<f64>,
        sat_id: usize,
        sample: &IQSample,
    ) -> Option<GnssCorrelationParam> {
        assert_eq!(
            sample.iq_vec.len(),
            get_num_samples_per_msec() * ACQUISITION_WINDOW_MSEC
        );
        let mut spread_hz = DOPPLER_SPREAD_HZ;
        let mut best_param = GnssCorrelationParam::default();

        while spread_hz > DOPPLER_SPREAD_BINS {
            let param = self.calc_cross_correlation(
                fft_planner,
                sample,
                sat_id,
                ACQUISITION_WINDOW_MSEC,
                best_param.doppler_hz,
                spread_hz,
            );
            if param.snr <= best_param.snr {
                break;
            }
            spread_hz = spread_hz / DOPPLER_SPREAD_BINS;
            best_param = param;

            log::debug!(
                "BEST: sat_id={} -- estimate_hz={} spread_hz={:3} snr={:.3} phase_idx={}",
                sat_id,
                best_param.doppler_hz,
                spread_hz,
                best_param.snr,
                best_param.phase_offset
            );
        }
        if best_param.snr >= SNR_THRESHOLD {
            log::info!(
                " sat_id: {} -- doppler_hz: {:5} phase_idx: {:4} snr: {}",
                format!("{:2}", sat_id).yellow(),
                best_param.doppler_hz,
                best_param.phase_offset / 2,
                format!("{:.2}", best_param.snr).green(),
            );
            Some(best_param)
        } else {
            None
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
            "--- try_acq: cached_ts_sec_tail={:3} sec cached_vec_len={}",
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
                self.try_acquisition_one_sat(&mut fft_planner, id, &sample)
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
                    self.satellites
                        .insert(*id, GnssSatellite::new(*id, *param, samples_ts_sec));
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

        for (_id, sat) in &mut self.satellites {
            sat.process_samples(&samples);
        }

        Ok(())
    }
}
