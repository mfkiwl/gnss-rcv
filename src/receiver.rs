use colored::Colorize;
use rustfft::{num_complex::Complex64, FftPlanner};
use std::collections::HashMap;
use std::ops::Mul;
use std::time::Instant;

use crate::gold_code::gen_code;
use crate::recording::IQRecording;
use crate::util::get_2nd_max;
use crate::util::get_max_with_idx;

const PRN_CODE_LEN: usize = 1023;
const DOPPLER_SPREAD_HZ: u32 = 8 * 1000;
const DOPPLER_SPREAD_BINS: u32 = 10;
const ACQUISITION_PERIOD_MSEC: usize = 10;
const SNR_THRESHOLD: f64 = 1.0;
const PI: f64 = std::f64::consts::PI;

pub struct GnssReceiver {
    pub iq_recording: IQRecording,
    pub verbose: bool,
    fft_planner: FftPlanner<f64>,
    prn_code_fft_map: HashMap<usize, Vec<Complex64>>,
    sat_vec: Vec<usize>,
    off_msec: usize,
    last_acq_off_msec: usize,
    cached_iq_vec: Vec<Complex64>,
    cached_num_msec: usize,
    cached_off_msec_tail: usize,
}

impl GnssReceiver {
    pub fn new(iq_recording: IQRecording, verbose: bool, sat_vec: Vec<usize>) -> Self {
        Self {
            iq_recording,
            verbose,
            fft_planner: FftPlanner::new(),
            prn_code_fft_map: HashMap::<usize, Vec<Complex64>>::new(),
            sat_vec,
            off_msec: 0,
            last_acq_off_msec: 0,
            cached_iq_vec: Vec::<Complex64>::new(),
            cached_num_msec: 0,
            cached_off_msec_tail: 0,
        }
    }

    fn normalize_post_fft(data: &Vec<Complex64>) -> Vec<Complex64> {
        let len = data.len() as f64;
        data.iter().map(|x| x / len).collect()
    }

    fn calc_correlation(
        &mut self,
        v_antenna: &Vec<Complex64>,
        prn_code_fft: &Vec<Complex64>,
    ) -> Vec<Complex64> {
        let num_samples = v_antenna.len();
        assert_eq!(v_antenna.len(), prn_code_fft.len());
        let fft_fw = self.fft_planner.plan_fft_forward(num_samples);

        let mut v_antenna_buf = v_antenna.clone();

        fft_fw.process(&mut v_antenna_buf);

        let mut v_res: Vec<_> = (0..v_antenna_buf.len())
            .map(|i| v_antenna_buf[i].mul(prn_code_fft[i].conj()))
            .collect();

        let fft_bw = self.fft_planner.plan_fft_inverse(num_samples);
        fft_bw.process(&mut v_res);
        Self::normalize_post_fft(&v_res) // not really required
    }

    fn calc_prn_fft(&mut self, v_prn: &Vec<Complex64>) -> Vec<Complex64> {
        let fft_fw = self.fft_planner.plan_fft_forward(v_prn.len());
        let mut v_prn_buf = v_prn.clone();
        fft_fw.process(&mut v_prn_buf);
        v_prn_buf
    }

    fn calc_cross_correlation(
        &mut self,
        iq_vec: &Vec<Complex64>,
        sat_id: usize,
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

        let prn_code_fft;
        if !self.prn_code_fft_map.contains_key(&sat_id) {
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
            prn_code_fft = self.calc_prn_fft(&prn_code_upsampled);
            self.prn_code_fft_map.insert(sat_id, prn_code_fft.clone());
        } else {
            prn_code_fft = self.prn_code_fft_map.get(&sat_id).unwrap().clone();
        }

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
                assert_eq!(iq_vec_sample.len(), prn_code_fft.len());

                for i in 0..iq_vec_sample.len() {
                    iq_vec_sample[i] = iq_vec_sample[i].mul(doppler_shift[i]);
                }

                let corr = self.calc_correlation(&iq_vec_sample, &prn_code_fft);
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
                let b_corr_second = get_2nd_max(&b_corr);
                let (idx, b_corr_peak) = get_max_with_idx(&b_corr);
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
                        doppler_hz,
                        b_peak_to_second,
                        idx / 2
                    );
                }
            }
        }
        (best_doppler_hz, best_snr, best_phase_offset)
    }

    fn try_acquisition_one_sat(&mut self, iq_vec: &Vec<Complex64>, sat_id: usize, off_msec: usize) {
        assert_eq!(iq_vec.len(), PRN_CODE_LEN * 2 * ACQUISITION_PERIOD_MSEC);
        let mut spread_hz = DOPPLER_SPREAD_HZ;
        let mut best_estimate_hz = 0i32;
        let mut best_snr = 0.0f64;
        let mut best_phase_idx = 0;

        while spread_hz > DOPPLER_SPREAD_BINS {
            let (estimate_hz, snr, phase_idx) = self.calc_cross_correlation(
                iq_vec,
                sat_id,
                ACQUISITION_PERIOD_MSEC,
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

    fn try_periodic_acquisition(&mut self) {
        if self.cached_num_msec < ACQUISITION_PERIOD_MSEC {
            return;
        }

        if self.cached_off_msec_tail < self.last_acq_off_msec + ACQUISITION_PERIOD_MSEC {
            return;
        }

        let ts = Instant::now();
        let cached_vec_len = self.cached_iq_vec.len();
        let num_samples = ACQUISITION_PERIOD_MSEC * PRN_CODE_LEN * 2;

        log::warn!(
            "try_acq: vec_deq_off_msec_tail={} vec_deq_len={}",
            self.cached_off_msec_tail,
            cached_vec_len
        );

        let samples = &self.cached_iq_vec[cached_vec_len - num_samples..cached_vec_len].to_vec();
        let samples_off_msec = self.cached_off_msec_tail - ACQUISITION_PERIOD_MSEC;

        for id in self.sat_vec.clone() {
            self.try_acquisition_one_sat(&samples, id, samples_off_msec);
        }
        log::warn!("acquisition: {} msec", ts.elapsed().as_millis());
        self.last_acq_off_msec = self.cached_off_msec_tail;
    }

    pub fn process_step(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let num_msec_per_step = 1;
        let mut vec_samples = self
            .iq_recording
            .get_msec_sample(self.off_msec, num_msec_per_step);
        log::info!("step: off_msec={}", self.off_msec);

        self.cached_iq_vec.append(&mut vec_samples);
        self.cached_off_msec_tail += num_msec_per_step;
        self.cached_num_msec += num_msec_per_step;
        self.off_msec += num_msec_per_step;
        if self.cached_num_msec > ACQUISITION_PERIOD_MSEC {
            let num_msec_to_remove =  self.cached_num_msec - ACQUISITION_PERIOD_MSEC;
            let num_samples = num_msec_to_remove * PRN_CODE_LEN * 2;
            let _ = self.cached_iq_vec.drain(0..num_samples);
            self.cached_num_msec -= num_msec_to_remove;
        }

        self.try_periodic_acquisition();

        Ok(())
    }
}
