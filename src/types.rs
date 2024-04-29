#[derive(Default, Clone, Copy)]
pub struct GnssCorrelationParam {
    pub doppler_hz: i32,
    pub phase_offset: usize,
    pub snr: f64,
    pub corr_norm: f64,
}
