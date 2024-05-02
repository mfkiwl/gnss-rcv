use crate::types::GnssCorrelationParam;
use crate::types::IQSample;
use crate::util::correlate_vec;
use crate::util::doppler_shifted_carrier;
use crate::util::get_num_samples_per_msec;
use colored::Colorize;
use rustfft::num_complex::Complex64;
use std::ops::Mul;

pub struct GnssSatellite {
    prn: usize,
    param: GnssCorrelationParam,
    phase_offset_f64: f64,
    creation_ts_sec: f64,
    prn_code: Vec<Complex64>,
}

impl GnssSatellite {
    pub fn new(
        prn: usize,
        prn_code: Vec<Complex64>,
        param: GnssCorrelationParam,
        ts_sec: f64,
    ) -> Self {
        log::warn!(
            "{}",
            format!(
                "sat {}: new: doppler={} phase_shift={} snr={:.2} carrier_phase_shift={:.1}",
                prn, param.doppler_hz, param.phase_offset, param.snr, param.carrier_phase_shift
            )
            .green()
        );
        Self {
            prn,
            prn_code,
            param,
            phase_offset_f64: param.phase_offset as f64,
            creation_ts_sec: ts_sec,
        }
    }

    pub fn update_param(&mut self, param: &GnssCorrelationParam, ts_sec: f64) {
        self.param = *param;

        log::warn!(
            "sat {}: exists: age={:.3} sec -- doppler_hz={} phase_shift={} snr={:.2}",
            self.prn,
            ts_sec - self.creation_ts_sec,
            param.doppler_hz,
            param.phase_offset,
            param.snr
        );
    }

    pub fn verify_correlation_peak(&self, doppler_shifted_samples: Vec<Complex64>) {
        let mut res = vec![];
        let mut p = self.prn_code.clone();
        for _i in 0..get_num_samples_per_msec() {
            let c = correlate_vec(&doppler_shifted_samples, &p);
            res.push(c.norm_sqr());
            p.rotate_right(1);
        }
        let mut max_idx = 0;
        let mut max_res = 0.0;
        for i in 0..res.len() {
            if res[i] > max_res {
                max_res = res[i];
                max_idx = i;
            }
        }
        if max_idx.abs_diff(self.param.phase_offset) > 3 {
            log::warn!(
                "sat-{}: {}: peak at: {} but phase_offset={}",
                self.prn,
                format!("XXX code tracking error").red(),
                max_idx,
                self.param.phase_offset,
            );
        }
    }

    pub fn process_samples(&mut self, sample: &IQSample) {
        let vec_len = sample.iq_vec.len();
        log::info!(
            "sat-{}: processing: ts_sec={:.4} sec num_samples={}: doppler={} phase={}",
            self.prn,
            sample.ts_sec,
            sample.iq_vec.len(),
            self.param.doppler_hz,
            self.param.phase_offset,
        );

        let mut signal = doppler_shifted_carrier(
            self.param.doppler_hz,
            sample.ts_sec,
            -self.param.carrier_phase_shift,
            sample.sample_rate,
            vec_len,
        );

        for i in 0..vec_len {
            signal[i] = signal[i].mul(sample.iq_vec[i]);
        }

        let doppler_shifted_samples = signal;

        let mut prn_code_early = self.prn_code.clone();
        let mut prn_code_late = self.prn_code.clone();

        if self.param.phase_offset >= 1 {
            prn_code_early.rotate_right(self.param.phase_offset - 1);
        } else {
            prn_code_early.rotate_left(1);
        }
        prn_code_late.rotate_right((self.param.phase_offset + 1) % vec_len);

        let corr_early = correlate_vec(&doppler_shifted_samples, &prn_code_early);
        let corr_late = correlate_vec(&doppler_shifted_samples, &prn_code_late);

        let discriminator = (corr_early.norm_sqr() - corr_late.norm_sqr()) / 2.0;

        self.phase_offset_f64 -= discriminator * 0.002;
        if self.phase_offset_f64 < 0.0 {
            self.phase_offset_f64 += get_num_samples_per_msec() as f64;
        }
        self.phase_offset_f64 = self.phase_offset_f64 % get_num_samples_per_msec() as f64;
        self.param.phase_offset =
            self.phase_offset_f64.round() as usize % get_num_samples_per_msec();

        // verify tracking error every second
        if (sample.ts_sec - sample.ts_sec.round() as f64).abs() < 0.001 {
            self.verify_correlation_peak(doppler_shifted_samples);
        }

        log::info!(
            "sat {} -- doppler_hz={} phase_offset={} polar: carrier_phase_shift={}",
            self.prn,
            self.param.doppler_hz,
            self.param.phase_offset,
            self.param.carrier_phase_shift
        );
    }
}
