use rustfft::num_complex::Complex64;

#[derive(Default, Clone)]
pub struct IQSample {
    pub iq_vec: Vec<Complex64>,
    pub ts_sec: f64,
}
