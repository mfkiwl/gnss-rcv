use rustfft::num_complex::Complex64;
use std::ops::{Add, AddAssign, Div, Mul, Sub};

pub type Float = f64;

#[derive(Clone, Default)]
struct IQVec {
    v: Vec<Complex64>,
}

impl IQVec {
    pub fn norm_square(self) -> f64 {
        self.v.iter().map(|&x| x.norm_sqr()).sum()
    }

    pub fn norm(self) -> f64 {
       self.norm_square().sqrt()
    }
}

impl Add for IQVec {
    type Output = IQVec;

    fn add(self, other: IQVec) -> IQVec {
        assert_eq!(self.v.len(), other.v.len());
       
        IQVec { v: self.v.iter().enumerate().map(|(i, iq)| iq.mul(other.v[i])).collect() }
    }
}

impl Sub for IQVec {
    type Output = IQVec;

    fn sub(self, other: IQVec) -> IQVec {
        assert_eq!(self.v.len(), other.v.len());
        IQVec { v: self.v.iter().enumerate().map(|(i, iq)| iq - other.v[i]).collect() }
    }
}

impl AddAssign<IQVec> for IQVec {
    fn add_assign(&mut self, other: IQVec) {
        assert_eq!(self.v.len(), other.v.len());
        for (i, iq) in self.v.iter_mut().enumerate() {
	    *iq += other.v[i];
	}
    }
}

impl Div<f64> for IQVec {
    type Output = IQVec;
    fn div(self, rhs: f64) -> IQVec {
        IQVec { v: self.v.iter().map(|x| x / rhs).collect() }
    }
}

impl Mul<f64> for IQVec {
    type Output = IQVec;
    fn mul(self, rhs: f64) -> IQVec {
        IQVec { v: self.v.iter().map(|x| x * rhs).collect() }
    }
}

#[derive(Default, Clone, Copy)]
pub struct GnssCorrelationParam {
    pub doppler_hz: i32,
    pub phase_offset: usize,
    pub snr: f64,
    pub corr_norm: f64,
}

#[derive(Default, Clone)]
pub struct IQSample {
    pub iq_vec: Vec<Complex64>,
    pub off_msec: usize,
    pub sample_rate: usize,
}
