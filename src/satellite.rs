use colored::Colorize;
use rustfft::num_complex::Complex64;

pub struct GnssSatellite {
    prn: usize,
    pub snr: f64,
    pub doppler_hz: i32,
    pub phase_shift: usize,
    creation_ts_msec: usize,
}

impl GnssSatellite {
    pub fn new(prn: usize, snr: f64, doppler_hz: i32, phase_shift: usize, off_msec: usize) -> Self {
        log::warn!(
            "{}",
            format!(
                "sat {}: new: doppler={} phase_shift={} snr={:.2}",
                prn, doppler_hz, phase_shift, snr
            )
            .green()
        );
        Self {
            prn,
            snr,
            doppler_hz,
            phase_shift,
            creation_ts_msec: off_msec,
        }
    }

    pub fn update_after_new_acq(
        &mut self,
        snr: f64,
        doppler_hz: i32,
        phase_shift: usize,
        off_msec: usize,
    ) {
        self.snr = snr;
        self.doppler_hz = doppler_hz;
        self.phase_shift = phase_shift;

        log::warn!(
            "sat {}: exists: age={} msec -- doppler_hz={} phase_shift={} snr={:.2}",
            self.prn,
            off_msec - self.creation_ts_msec,
            doppler_hz,
            phase_shift,
            snr
        );
    }

    pub fn process_samples(&mut self, samples: &Vec<Complex64>) {
        log::trace!("sat-{}: processing {} samples", self.prn, samples.len());
    }
}
