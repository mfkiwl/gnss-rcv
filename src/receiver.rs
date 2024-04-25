use colored::Colorize;
use rustfft::{num_complex::Complex64, FftPlanner};
use std::ops::Mul;
use std::time::Instant;

use crate::gold_code::gen_code;
use crate::recording::IQRecording;
use crate::util::norm;

const PRN_CODE_LEN: usize = 1023;
const DOPPLER_SPREAD_HZ: u32 = 7 * 1000;
const DOPPLER_SPREAD_BINS: u32 = 10;
const ACQUISITION_PERIOD_MSEC: usize = 10;
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
        let fft_fw = planner.plan_fft_forward(num_samples);

        let mut v_antenna_buf = v_antenna.clone();
        let mut v_prn_buf = v_prn.clone();

        fft_fw.process(&mut v_antenna_buf);
        fft_fw.process(&mut v_prn_buf);

        let mut v_res: Vec<_> = (0..v_antenna_buf.len())
            .map(|i| v_antenna_buf[i].mul(v_prn_buf[i].conj()))
            .collect();

        let fft_bw = planner.plan_fft_inverse(num_samples);
        fft_bw.process(&mut v_res);
        Self::normalize(&v_res)
    }

    fn calc_cross_correlation(
        &self,
        iq_vec: &Vec<Complex64>,
        prn_code_upsampled_dup: &Vec<Complex64>,
        num_msec: usize,
        estimate_hz: i32,
        spread_hz: u32,
    ) -> (i32, f64) {
        let num_samples_per_prn = PRN_CODE_LEN * 2;
        assert_eq!(prn_code_upsampled_dup.len(), num_samples_per_prn * 2);
        let num_samples = iq_vec.len();
        assert_eq!(num_samples, num_samples_per_prn * num_msec);
        let mut best_corr = 0.0;
        let mut best_doppler_hz: i32 = 0;

        let lo_hz = estimate_hz - spread_hz as i32;
        let hi_hz = estimate_hz + spread_hz as i32 + 1;

        for doppler_hz in (lo_hz..hi_hz).step_by((spread_hz / DOPPLER_SPREAD_BINS) as usize) {
            let imaginary = -2.0 * PI * doppler_hz as f64;
            let mut b_corr = 0.0;

            for idx in 0..num_msec {
                let prn_code = Vec::from(&prn_code_upsampled_dup[0..num_samples_per_prn]);
                let sample_doppler_shifted: Vec<_> = (0..num_samples_per_prn)
                    .map(|x| Complex64 {
                        re: 0.0,
                        im: imaginary * (x + idx * num_samples_per_prn) as f64 / 2046000.0,
                    })
                    .collect();
                let range_lo = (idx + 0) * num_samples_per_prn;
                let range_hi = (idx + 1) * num_samples_per_prn;
                let iq_vec_sample = Vec::from(&iq_vec[range_lo..range_hi]);
                assert_eq!(iq_vec_sample.len(), sample_doppler_shifted.len());
                assert_eq!(iq_vec_sample.len(), prn_code.len());

                let corr = Self::calc_correlation(&sample_doppler_shifted, &prn_code);
                b_corr += norm(&corr);
            }
            if self.verbose {
                println!(
                    "  get_cross_correlation: from {} Hz to {} Hz -- trying {} Hz: corr={:+.3e}",
                    estimate_hz - spread_hz as i32,
                    estimate_hz + spread_hz as i32,
                    doppler_hz,
                    b_corr,
                );
            }

            if b_corr > best_corr {
                best_corr = b_corr;
                best_doppler_hz = doppler_hz as i32;
                if self.verbose {
                    println!("   best_doppler: corr: {:+.3e}", b_corr);
                }
            }
        }
        (best_doppler_hz, best_corr)
    }

    fn try_acquisition_one_sat(&self, iq_vec: &Vec<Complex64>, sat_id: usize, num_msec: usize) {
        println!(
            "acquisition w/ sat_id={} using {} samples.",
            sat_id,
            iq_vec.len()
        );
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
        let mut prn_code_upsampled_dup = prn_code_upsampled.clone();
        prn_code_upsampled
            .iter()
            .for_each(|&x| prn_code_upsampled_dup.push(x));
        assert_eq!(prn_code_upsampled_dup.len(), PRN_CODE_LEN * 2 * 2); // 2msec worth

        let mut spread_hz = DOPPLER_SPREAD_HZ;
        let mut best_estimate_hz = 0i32;
        let mut best_corr = 0.0f64;

        while spread_hz > DOPPLER_SPREAD_BINS {
            let (estimate_hz, corr) = self.calc_cross_correlation(
                iq_vec,
                &prn_code_upsampled_dup,
                num_msec,
                best_estimate_hz,
                spread_hz,
            );
            if corr <= best_corr {
                break;
            }
            spread_hz = spread_hz / DOPPLER_SPREAD_BINS;
            best_estimate_hz = estimate_hz;
            best_corr = corr;
            if self.verbose {
                let s = format!(
                    "BEST: sat_id={} -- estimate_hz={} spread_hz={:3} corr={:+.3e}",
                    sat_id, estimate_hz, spread_hz, corr
                );
                println!("{}", s.green());
            }
        }
        let s = format!(
            " sat_id={} -- doppler_hz={} corr={:+.3e}",
            sat_id, best_estimate_hz, best_corr
        );
        println!("{}", s.yellow());
    }

    pub fn try_acquisition(&mut self, sat_id: usize) -> Result<(), Box<dyn std::error::Error>> {
        let num_msec = ACQUISITION_PERIOD_MSEC;
        let samples = self.iq_recording.get_msec_sample(num_msec);
        println!(
            "try_acquisition: {} msec: num_samples: {}",
            num_msec,
            samples.len()
        );
        let ts = Instant::now();
        if sat_id > 0 {
            self.try_acquisition_one_sat(&samples, sat_id, num_msec);
        } else {
            for id in 1..32 {
                self.try_acquisition_one_sat(&samples, id, num_msec);
            }
        }

        println!("acq. took {} msec", ts.elapsed().as_millis());
        Ok(())
    }
}
