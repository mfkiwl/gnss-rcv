use crate::types::GnssCorrelationParam;
use crate::types::IQSample;
use colored::Colorize;

pub struct GnssSatellite {
    prn: usize,
    pub param: GnssCorrelationParam,
    creation_ts_msec: usize,
}

impl GnssSatellite {
    pub fn new(prn: usize, param: GnssCorrelationParam, off_msec: usize) -> Self {
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
            creation_ts_msec: off_msec,
        }
    }

    pub fn update_param(&mut self, param: &GnssCorrelationParam, off_msec: usize) {
        self.param = *param;

        log::warn!(
            "sat {}: exists: age={} msec -- doppler_hz={} phase_shift={} snr={:.2}",
            self.prn,
            off_msec - self.creation_ts_msec,
            param.doppler_hz,
            param.phase_offset,
            param.snr
        );
    }

    pub fn process_samples(&mut self, sample: &IQSample) {
        log::debug!(
            "sat-{}: processing {} samples",
            self.prn,
            sample.iq_vec.len()
        );
    }
}
