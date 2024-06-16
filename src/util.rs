use rustfft::{num_complex::Complex64, FftPlanner};
use std::ops::Mul;

const PI: f64 = std::f64::consts::PI;

pub fn norm_square(v: &Vec<Complex64>) -> f64 {
    v.iter().map(|&x| x.norm_sqr()).sum::<f64>()
}

pub fn norm(v: &Vec<Complex64>) -> f64 {
    norm_square(v).sqrt()
}

pub fn get_max_with_idx(v: &Vec<f64>) -> (usize, f64) {
    let mut max = 0.0f64;
    let mut idx = 0;
    for i in 0..v.len() {
        if v[i] > max {
            max = v[i];
            idx = i;
        }
    }
    (idx, max)
}

pub fn get_average(v: &Vec<f64>) -> f64 {
    v.iter().sum::<f64>() / v.len() as f64
}

fn normalize_post_fft(data: &mut Vec<Complex64>) {
    let len = data.len() as f64;
    data.iter_mut().for_each(|x| *x /= len);
}

pub fn correlate_vec(a: &Vec<Complex64>, b: &Vec<Complex64>) -> Complex64 {
    let mut sum = Complex64 { re: 0.0, im: 0.0 };
    for i in 0..a.len() {
        sum += a[i].mul(b[i].conj());
    }
    sum
}

pub fn calc_correlation(
    fft_planner: &mut FftPlanner<f64>,
    iq_vec: &Vec<Complex64>,
    prn_code_fft: &Vec<Complex64>,
) -> Vec<Complex64> {
    let num_samples = iq_vec.len();
    assert_eq!(iq_vec.len(), prn_code_fft.len());
    let fft_fw = fft_planner.plan_fft_forward(num_samples);

    let mut iq_samples_fft = iq_vec.clone();

    fft_fw.process(&mut iq_samples_fft);

    let mut v_res: Vec<_> = (0..num_samples)
        .map(|i| iq_samples_fft[i] * prn_code_fft[i].conj())
        .collect();

    let fft_bw = fft_planner.plan_fft_inverse(num_samples);
    fft_bw.process(&mut v_res);
    normalize_post_fft(&mut v_res);
    v_res
}

fn doppler_shifted_carrier(doppler_hz: f64, phi: f64, fs: f64, len: usize) -> Vec<Complex64> {
    let imaginary = 2.0 * PI * doppler_hz;
    let phi_off = 2.0 * PI * phi;

    let carrier: Vec<Complex64> = (0..len)
        .map(|x| x as f64)
        .map(|y| Complex64::from_polar(1.0, -imaginary * (y / fs) - phi_off))
        .collect();

    carrier
}

pub fn doppler_shift(iq_vec: &mut Vec<Complex64>, doppler_hz: f64, phi: f64, fs: f64) {
    let carrier = doppler_shifted_carrier(doppler_hz, phi, fs, iq_vec.len());

    assert_eq!(iq_vec.len(), carrier.len());

    for i in 0..iq_vec.len() {
        iq_vec[i] = iq_vec[i] * carrier[i];
    }
}

pub fn getbitu(buf: &[u8], pos: usize, len: usize) -> u32 {
    assert!(len <= 32);
    let mut bits = 0;
    for i in pos..pos + len {
        bits = (bits << 1) | ((buf[i / 8] >> (7 - i % 8)) & 1) as u32;
    }
    bits
}

pub fn getbits(buf: &[u8], pos: usize, len: usize) -> i32 {
    let bits = getbitu(buf, pos, len);

    let sign = (1 << (len - 1)) & bits;
    let mask = (0xffffffff >> (len - 1)) << (len - 1);
    let res = if sign != 0 { bits | mask } else { bits & !mask };
    res as i32
}

pub fn getbitu2(buf: &[u8], p1: usize, l1: usize, p2: usize, l2: usize) -> u32 {
    assert!(l1 + l2 <= 32);
    let hi = getbitu(buf, p1, l1);
    let lo = getbitu(buf, p2, l2);
    (hi << l2) + lo
}

pub fn getbits2(buf: &[u8], p1: usize, l1: usize, p2: usize, l2: usize) -> i32 {
    assert!(l1 + l2 <= 32);
    if getbitu(buf, p1, 1) != 0 {
        ((getbits(buf, p1, l1) << l2) + getbitu(buf, p2, l2) as i32) as i32
    } else {
        getbitu2(buf, p1, l1, p2, l2) as i32
    }
}

pub fn hex_str(data: &[u8]) -> String {
    let num_bits = data.len();
    let mut s = String::new();
    for i in 0..(num_bits + 7) / 8 {
        let n = format!("{:02x}", data[i]);
        s.push_str(&n);
    }
    s
}

pub fn xor_bits(v: u32) -> u8 {
    const XOR_8B: [u8; 256] = [
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0,
        0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0,
        0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0,
        0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0,
        0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    ];

    let bytes = v.to_le_bytes().map(|v| v as usize);
    XOR_8B[bytes[0]] ^ XOR_8B[bytes[1]] ^ XOR_8B[bytes[2]] ^ XOR_8B[bytes[3]]
}

pub fn bits_opposed(bits0: &[u8], bits1: &[u8]) -> bool {
    let bits1_rev: Vec<_> = bits1.iter().map(|v| 1 - v).collect();
    bits_equal(bits0, bits1_rev.as_slice())
}

pub fn bits_equal(bits0: &[u8], bits1: &[u8]) -> bool {
    assert_eq!(bits0.len(), bits1.len());
    bits0 == bits1
}

pub fn setbitu(buf: &mut [u8], pos: usize, len: usize, data: u32) {
    let mut mask = 1u32 << (len - 1);
    if len > 32 {
        return;
    }
    for i in pos..pos + len {
        let bit = 1u8 << (7 - i % 8);
        if data & mask != 0 {
            buf[i / 8] |= bit;
        } else {
            buf[i / 8] &= !bit;
        }
        mask = mask >> 1;
    }
}
