use colored::Colorize;
use rustfft::{num_complex::Complex64, FftPlanner};
use std::collections::HashSet;
use std::ops::Mul;
use std::time::Instant;

use crate::gold_code::gen_code;
use crate::recording::IQRecording;
//use crate::util::norm;

const PRN_CODE_LEN: usize = 1023;
const DOPPLER_SPREAD_HZ: u32 = 8 * 1000;
const DOPPLER_SPREAD_BINS: u32 = 10;
const ACQUISITION_PERIOD_MSEC: usize = 10;
const SNR_THRESHOLD: f64 = 2.0;
const PI: f64 = std::f64::consts::PI;

pub struct GpsReceiver {
    pub iq_recording: IQRecording,
    pub verbose: bool,
}

impl GpsReceiver {
    pub fn new(iq_recording: IQRecording, verbose: bool) -> Self {
        Self {
            iq_recording,
            verbose,
        }
    }

    fn normalize(data: &Vec<Complex64>) -> Vec<Complex64> {
        let len = data.len() as f64;
        data.iter().map(|x| x / len).collect()
    }

    fn calc_correlation(v_antenna: &Vec<Complex64>, v_prn: &Vec<Complex64>) -> Vec<Complex64> {
        let num_samples = v_antenna.len();
        assert_eq!(v_antenna.len(), v_prn.len());
        let mut planner = FftPlanner::new();
        let fft_fw_ant = planner.plan_fft_forward(num_samples);
        let fft_fw_prn = planner.plan_fft_forward(num_samples);

        let mut v_antenna_buf = v_antenna.clone();
        let mut v_prn_buf = v_prn.clone();

        fft_fw_ant.process(&mut v_antenna_buf);
        fft_fw_prn.process(&mut v_prn_buf);

        let mut v_res: Vec<_> = (0..v_antenna_buf.len())
            .map(|i| v_antenna_buf[i].mul(v_prn_buf[i].conj()))
            .collect();

        let fft_bw = planner.plan_fft_inverse(num_samples);
        fft_bw.process(&mut v_res);
        Self::normalize(&v_res)
    }

    fn get_max(v: &Vec<f64>) -> (usize, f64) {
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

    fn get_2nd_max(v: &Vec<f64>) -> f64 {
        let (i_max, max) = Self::get_max(v);

        let mut second = 0.0;
        let delta = 50;
        for i in 0..v.len() {
            if v[i] > second && v[i] < max && (i > i_max + delta || i < i_max - delta) {
                second = v[i];
            }
        }
        second
    }

    fn calc_cross_correlation(
        &self,
        iq_vec: &Vec<Complex64>,
        prn_code: &Vec<Complex64>,
        num_msec: usize,
        off_msec: usize,
        estimate_hz: i32,
        spread_hz: u32,
    ) -> (i32, f64, usize) {
        let sample_rate = self.iq_recording.sample_rate;
        let num_samples_per_prn = PRN_CODE_LEN * 2;
        assert_eq!(iq_vec.len(), num_samples_per_prn * num_msec);
        let mut best_doppler_hz: i32 = 0;
        let mut best_phase_offset = 0;
        let mut best_snr = 0.0;
        let mut best_corr_norm = 0.0;

        let lo_hz = estimate_hz - spread_hz as i32;
        let hi_hz = estimate_hz + spread_hz as i32 + 1;

        for doppler_hz in (lo_hz..hi_hz).step_by((spread_hz / DOPPLER_SPREAD_BINS) as usize) {
            let imaginary = -2.0 * PI * doppler_hz as f64;
            let mut b_corr = vec![0f64; 2046];

            for idx in 0..num_msec {
                let sample_index = idx * num_samples_per_prn + off_msec;
                let doppler_shift: Vec<_> = (0..num_samples_per_prn)
                    .map(|x| {
                        Complex64::from_polar(
                            1.0,
                            imaginary * (x + sample_index) as f64 / sample_rate as f64,
                        )
                    })
                    .collect();
                let range_lo = (idx + 0) * num_samples_per_prn;
                let range_hi = (idx + 1) * num_samples_per_prn;
                let mut iq_vec_sample = Vec::from(&iq_vec[range_lo..range_hi]);
                assert_eq!(iq_vec_sample.len(), doppler_shift.len());
                assert_eq!(iq_vec_sample.len(), prn_code.len());

                for i in 0..iq_vec_sample.len() {
                    iq_vec_sample[i] = iq_vec_sample[i].mul(doppler_shift[i]);
                }

                let corr = Self::calc_correlation(&iq_vec_sample, &prn_code);
                for i in 0..corr.len() {
                    b_corr[i] += corr[i].norm();
                }
            }
            if self.verbose {
                println!(
                    "  get_cross_correlation: {}-{} Hz -- trying {} Hz",
                    estimate_hz - spread_hz as i32,
                    estimate_hz + spread_hz as i32,
                    doppler_hz,
                );
            }

            let b_corr_norm = b_corr.iter().map(|&x| x * x).sum::<f64>();

            if b_corr_norm > best_corr_norm || self.verbose {
                let b_corr_second = Self::get_2nd_max(&b_corr);
                let (idx, b_corr_peak) = Self::get_max(&b_corr);
                assert!(b_corr_peak != 0.0);
                assert!(b_corr_second != 0.0);
                let b_peak_to_second = 10.0 * (b_corr_peak / b_corr_second).log10();
                best_snr = b_peak_to_second;
                best_doppler_hz = doppler_hz as i32;
                best_phase_offset = idx;
                best_corr_norm = b_corr_norm;
                if self.verbose {
                    println!(
                        "   best_doppler: {} Hz snr: {:+.1} idx={}",
                        doppler_hz, b_peak_to_second, idx / 2
                    );
                }
            }
        }
        (best_doppler_hz, best_snr, best_phase_offset)
    }

    fn try_acquisition_one_sat(
        &self,
        iq_vec: &Vec<Complex64>,
        sat_id: usize,
        num_msec: usize,
        off_msec: usize,
    ) {
        assert_eq!(iq_vec.len(), PRN_CODE_LEN * 2 * num_msec);
        let one_prn_code = gen_code(sat_id);
        assert_eq!(one_prn_code.len(), PRN_CODE_LEN);
        let prn_code_upsampled: Vec<_> = one_prn_code
            .iter()
            .map(|&x| Complex64 {
                re: if x == 0 { -1.0 } else { 1.0 },
                im: 0.0,
            })
            .flat_map(|x| [x, x])
            .collect();
        assert_eq!(prn_code_upsampled.len(), PRN_CODE_LEN * 2); // 1msec worth

        let mut spread_hz = DOPPLER_SPREAD_HZ;
        let mut best_estimate_hz = 0i32;
        let mut best_snr = 0.0f64;
        let mut best_phase_idx = 0;

        while spread_hz > DOPPLER_SPREAD_BINS {
            let (estimate_hz, snr, phase_idx) = self.calc_cross_correlation(
                iq_vec,
                &prn_code_upsampled,
                num_msec,
                off_msec,
                best_estimate_hz,
                spread_hz,
            );
            if snr <= best_snr {
                break;
            }
            spread_hz = spread_hz / DOPPLER_SPREAD_BINS;
            best_estimate_hz = estimate_hz;
            best_snr = snr;
            best_phase_idx = phase_idx;

            if self.verbose {
                let s = format!(
                    "BEST: sat_id={} -- estimate_hz={} spread_hz={:3} snr={:.3} phase_idx={}",
                    sat_id, estimate_hz, spread_hz, snr, phase_idx
                );
                println!("{}", s.green());
            }
        }
        if best_snr >= SNR_THRESHOLD {
            println!(
                    " sat_id: {} -- doppler_hz: {:5} phase_idx: {:4} snr: {}",
                    format!("{:2}", sat_id).yellow(),
                    best_estimate_hz,
                    best_phase_idx / 2,
                    format!("{:.2}", best_snr).green(),
                    );
        }
    }

    pub fn try_acquisition(
        &mut self,
        off_msec: usize,
        sat_set: HashSet<usize>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let num_msec = ACQUISITION_PERIOD_MSEC;
        let samples = self.iq_recording.get_msec_sample(off_msec, num_msec);
        let ts = Instant::now();
        if !sat_set.is_empty() {
            for id in sat_set {
                self.try_acquisition_one_sat(&samples, id, num_msec, off_msec);
            }
        } else {
            for id in 0..32 {
                self.try_acquisition_one_sat(&samples, 1 + id, num_msec, off_msec);
            }
        }

        println!("duration: {} msec", ts.elapsed().as_millis());
        Ok(())
    }
}
