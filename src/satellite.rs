use colored::Colorize;
use rustfft::num_complex::Complex64;

use crate::types::GnssCorrelationParam;
use crate::types::IQSample;
use crate::util::get_num_samples_per_msec;

const PI: f64 = std::f64::consts::PI;

pub struct GnssSatellite {
    prn: usize,
    pub param: GnssCorrelationParam,
    creation_ts_sec: f64,
}

impl GnssSatellite {
    pub fn new(prn: usize, param: GnssCorrelationParam, ts_sec: f64) -> Self {
        log::warn!(
            "{}",
            format!(
                "sat {}: new: doppler={} phase_shift={} snr={:.2}",
                prn, param.doppler_hz, param.phase_offset, param.snr
            )
            .green()
        );
        Self {
            prn,
            param,
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

    pub fn process_samples(&mut self, sample: &IQSample) {
        log::info!(
            "sat-{}: processing: ts_sec={:.4} sec num_samples={}: doppler={} phase={}",
            self.prn,
            sample.ts_sec,
            sample.iq_vec.len(),
            self.param.doppler_hz,
            self.param.phase_offset,
        );
        let img = -2.0 * PI * self.param.doppler_hz as f64;

        let num_samples_per_msec = get_num_samples_per_msec();
        let doppler_shift: Vec<_> = (0..num_samples_per_msec)
            .map(|x| Complex64::from_polar(1.0, img * x as f64 / sample.sample_rate as f64))
            .collect();
    }
}
