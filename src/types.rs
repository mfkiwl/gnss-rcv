use rustfft::num_complex::Complex64;

#[derive(Default, Clone, Copy)]
pub struct GnssCorrelationParam {
    pub doppler_hz: f64,
    pub code_phase_offset: usize,
    pub carrier_phase_shift: f64,
    pub cn0: f64,
    pub corr_norm: f64,
}

#[derive(Default, Clone)]
pub struct IQSample {
    pub iq_vec: Vec<Complex64>,
    pub ts_sec: f64,
    pub sample_rate: usize,
}
