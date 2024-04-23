use rustfft::{num_complex::Complex32, FftPlanner};
use std::time::Instant;
use std::ops::Mul;

use crate::gold_code::gen_code;
use crate::recording::IQRecording;
use crate::util::norm;

const DOPPLER_SPREAD_HZ: u32 = 7 * 1000;
const DOPPLER_SPREAD_BINS: u32 = 10;
const PI: f32 = std::f32::consts::PI;

pub struct GpsReceiver {
    pub iq_recording: IQRecording,
    pub verbose: bool,
}

impl GpsReceiver {
    pub fn new(iq_recording: IQRecording, verbose: bool) -> Self {
        Self { iq_recording, verbose }
    }

    fn calc_correlation(va: &Vec<Complex32>, vb: &Vec<f32>) -> f32 {
        let num_samples = va.len();
        assert_eq!(va.len(), vb.len());
        let mut planner = FftPlanner::new();
        let fft_fw = planner.plan_fft_forward(num_samples);

        let mut va_buf = va.clone();
        let mut vb_buf: Vec<_> = vb.iter().map(|&x| Complex32::from(x)).collect();

        fft_fw.process(&mut va_buf);
        fft_fw.process(&mut vb_buf);

        let mut v_res: Vec<Complex32> = vec![];
        for i in 0..va_buf.len() {
            v_res.push(vb_buf[i].mul(va_buf[i].conj()));
        }

        let fft_bw = planner.plan_fft_inverse(num_samples);
        fft_bw.process(&mut v_res);
        norm(&v_res)
    }

    fn get_cross_correlation(
        &self,
        iq_vec: &Vec<Complex32>,
        scaled_prn_code: &Vec<f32>,
        estimate_hz: i32,
        spread_hz: u32,
    ) -> (i32, i32, f32) {
        let mut scaled_prn_code_ext = scaled_prn_code.clone();
        scaled_prn_code
            .iter()
            .for_each(|&x| scaled_prn_code_ext.push(x));

        let num_samples = iq_vec.len();
        let mut max_corr = 0.0;
        let mut max_idx: i32 = -1;
        let mut max_shift_hz: i32 = 0;

        for shift_hz in (estimate_hz - spread_hz as i32..estimate_hz + spread_hz as i32)
            .step_by(spread_hz as usize / DOPPLER_SPREAD_BINS as usize)
        {
            if self.verbose {
                println!(
                    "from {} to {} -- {}",
                    estimate_hz - spread_hz as i32,
                    estimate_hz + spread_hz as i32,
                    shift_hz
                );
            }

            let shift_op =
                Complex32::from_polar(1.0, 2.0 * PI * shift_hz as f32 * num_samples as f32);
            let v: Vec<Complex32> = (0..num_samples)
                .map(|x| {
                    if x == 0 {
                        Complex32 { re: 0.0, im: 0.0 }
                    } else {
                        shift_op.expf(0.001 * x as f32 / num_samples as f32)
                    }
                })
                .collect();
            for c in &v {
                assert!(!c.re.is_nan());
                assert!(!c.im.is_nan());
            }
            for idx in (0..num_samples).step_by(2) {
                let iq_vec_shifted = Vec::from(&scaled_prn_code_ext[idx..(idx + num_samples)]);
                let corr = Self::calc_correlation(&v, &iq_vec_shifted);

                if corr > max_corr {
                    max_corr = corr;
                    max_idx = idx as i32;
                    max_shift_hz = shift_hz as i32;
                    if self.verbose {
                        println!(
                            " shift: {:6}Hz idx: {:4} corr: {:+.3e}",
                            shift_hz, idx, corr
                        );
                    }
                }
            }
        }
        (max_shift_hz, max_idx, max_corr)
    }

    fn try_acquisition_one_sat(&self, iq_vec: &Vec<Complex32>, sat_id: usize) {
        println!("acquisition w/ sat_id={}..", sat_id);
        let prn_code = gen_code(sat_id);
        let code: Vec<f32> = prn_code
            .iter()
            .map(|&x| if x == 0 { -1.0 } else { 1.0 })
            .collect();
        // up-sample to match to sampling frequency.
        let scaled_code: Vec<f32> = code.iter().flat_map(|&x| [x, x]).collect();
        assert_eq!(scaled_code.len(), 1023 * 2);
        let mut spread_hz = DOPPLER_SPREAD_HZ;
        let mut best_estimate_hz = 0i32;
        let mut best_corr = 0.0f32;
        let mut best_idx = 0;

        while spread_hz > DOPPLER_SPREAD_BINS {
            let (estimate_shift, idx, corr) = self.get_cross_correlation(
                iq_vec,
                &scaled_code,
                best_estimate_hz,
                spread_hz,
            );
            if corr <= best_corr {
                break;
            }
            spread_hz = spread_hz / DOPPLER_SPREAD_BINS;
            best_estimate_hz = estimate_shift;
            best_corr = corr;
            best_idx = idx;
            if self.verbose {
                println!(
                    "MAX: sat_id={} -- estimate_hz={} spread_hz={:3} idx={:4} corr={:+.3e}",
                    sat_id, best_estimate_hz, spread_hz, best_idx, best_corr
                );
            }
        }
        println!(
            " sat_id={} -- estimate_hz={} spread_hz={:3} idx={:4} corr={:+.3e}",
            sat_id, best_estimate_hz, spread_hz, best_idx, best_corr
        );
    }

    pub fn try_acquisition(
        &mut self,
        sat_id: usize,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let sample = self.iq_recording.get_1msec_sample();
        println!("1msec: num_samples: {}", sample.len());
        let ts = Instant::now();
        if sat_id > 0 {
            self.try_acquisition_one_sat(&sample, sat_id);
        } else {
            for id in 1..32 {
                self.try_acquisition_one_sat(&sample, id);
            }
        }

        println!("acq. took {} msec", ts.elapsed().as_millis());
        Ok(())
    }
}
