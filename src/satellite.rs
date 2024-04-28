use rustfft::num_complex::Complex64;

pub struct GnssSatellite {
    prn: usize,
    pub snr: f64,
    pub doppler_hz: i32,
    pub phase_shift: usize,
}

impl GnssSatellite {
    pub fn new(prn: usize, snr: f64, doppler_hz: i32, phase_shift: usize) -> Self {
        Self {
            prn,
            snr,
            doppler_hz,
            phase_shift,
        }
    }

    pub fn process_samples(&mut self, samples: &Vec<Complex64>) {
        log::trace!("sat-{}: processing {} samples", self.prn, samples.len());
    }
}
