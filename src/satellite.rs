use crate::acquisition::integrate_correlation;
use crate::types::GnssCorrelationParam;
use crate::types::IQSample;
use colored::Colorize;
use rustfft::{num_complex::Complex64, FftPlanner};

pub struct GnssSatellite {
    prn: usize,
    param: GnssCorrelationParam,
    creation_ts_sec: f64,
    prn_code_fft: Vec<Complex64>,
    fft_planner: FftPlanner<f64>,
}

impl GnssSatellite {
    pub fn new(
        prn: usize,
        prn_code_fft: Vec<Complex64>,
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
            prn_code_fft,
            param,
            creation_ts_sec: ts_sec,
            fft_planner: FftPlanner::new(),
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

        let (_, corr_c) = integrate_correlation(
            &mut self.fft_planner,
            &self.prn_code_fft,
            sample,
            1,
            self.param.doppler_hz,
        );

        // compute the carrier phase shift.
        let polar = corr_c[self.param.phase_offset].to_polar();
        self.param.carrier_phase_shift = polar.1;
        log::info!(
            "sat {} -- doppler_hz={} phase_offset={} polar: r={} theta={}",
            self.prn,
            self.param.doppler_hz,
            self.param.phase_offset,
            polar.0,
            polar.1
        );
    }
}
