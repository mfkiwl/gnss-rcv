use colored::Colorize;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::ops::Mul;
use std::path::PathBuf;
use std::time::Instant;

use rustfft::{num_complex::Complex32, FftPlanner};

use crate::gold_code::gen_code;
use crate::util::pretty_print;

const BUFFER_SIZE: usize = 128 * 1024;
const DOPPLER_SPREAD_HZ: i32 = 7 * 1000;
const DOPPLER_SPREAD_BINS: u32 = 10;

pub struct IQRecording {
    pub file_path: PathBuf,
    pub sample_rate: u64,
    pub iq_vec: Vec<Complex32>,
}

//https://wiki.gnuradio.org/index.php?title=File_Sink#Handling_File_Sink_data

impl IQRecording {
    pub fn new(file_path: PathBuf, sample_rate: u64) -> Self {
        Self {
            file_path,
            sample_rate,
            iq_vec: vec![],
        }
    }
    pub fn read_iq_file(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let file = File::open(self.file_path.clone())?;
        let mut n: u64 = 0;

        println!(
            "{}: size: {}",
            self.file_path.display(),
            pretty_print(file.metadata().unwrap().len()).bold()
        );

        let mut reader = BufReader::with_capacity(BUFFER_SIZE, file);

        let ts = Instant::now();

        loop {
            let buf = reader.fill_buf()?;
            let len = buf.len();

            if len == 0 {
                break;
            }

            for i in 0..len / 8 {
                let off = 8 * i;
                let bi: [u8; 4] = [buf[off + 0], buf[off + 1], buf[off + 2], buf[off + 3]];
                let bq: [u8; 4] = [buf[off + 4], buf[off + 5], buf[off + 6], buf[off + 7]];
                let i = f32::from_le_bytes(bi);
                let q = f32::from_le_bytes(bq);
                self.iq_vec.push(Complex32::new(i, q));
            }
            n += 1;
            if n == 1000 {
                break;
            }
            reader.consume(len);
        }

        println!("num_samples: {:?} num_read: {}", self.iq_vec.len(), n);
        let elapsed_msec = ts.elapsed().as_millis();
        let bw = n as f64 * BUFFER_SIZE as f64 * 1000.0 / 1024.0 / 1024.0 / elapsed_msec as f64;
        println!(
            "elapsed: {} msec -- read bandwidth: {:.1} MB/sec",
            elapsed_msec, bw
        );

        Ok(())
    }

    fn get_1msec_sample(&self) -> Vec<Complex32> {
        //let num_samples = self.sample_rate / 1000;
        let num_samples = 2044;
        let mut v: Vec<Complex32> = vec![];
        for i in 0..num_samples as usize {
            v.push(self.iq_vec[i]);
        }
        v
    }

    fn norm(v: &Vec<Complex32>) -> f32 {
        let res: f32 = v.iter().map(|x| x.norm_sqr()).sum();
        res.sqrt()
    }

    fn find_correlation(va: &Vec<Complex32>, vb: &Vec<f32>) -> f32 {
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
        Self::norm(&v_res)
    }

    fn find_doppler_shift(
        &self,
        iq_vec: &Vec<Complex32>,
        scaled_prn_code: &Vec<f32>,
        estimate_hz: i32,
        spread_hz: i32,
    ) -> (i32, i32, f32) {
        const PI: f32 = std::f32::consts::PI;
        //let scaled_prn_code_ext = scaled_prn_code.clone().append(&mut scaled_prn_code.clone());
        let mut scaled_prn_code_ext = scaled_prn_code.clone();
        scaled_prn_code
            .iter()
            .for_each(|&x| scaled_prn_code_ext.push(x));

        let num_samples = iq_vec.len();
        let mut max_corr = 0.0;
        let mut max_idx: i32 = -1;
        let mut max_shift_hz: i32 = 0;

        for shift_hz in (estimate_hz - spread_hz..estimate_hz + spread_hz)
            .step_by(spread_hz as usize / DOPPLER_SPREAD_BINS as usize)
        {
            //println!("from {} to {} -- {}", estimate_hz - spread_hz, estimate_hz + spread_hz, shift_hz);
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
                let corr = Self::find_correlation(&v, &iq_vec_shifted);

                if corr > max_corr {
                    max_corr = corr;
                    max_idx = idx as i32;
                    max_shift_hz = shift_hz as i32;
                    //println!(
                    //    " shift: {:6}Hz idx: {:4} corr: {:+.3e}",
                    //    shift_hz, idx, corr
                    //);
                }
            }
        }
        (max_shift_hz, max_idx, max_corr)
    }

    fn try_acquisition_one_sat(&self, iq_vec: &Vec<Complex32>, sat_id: usize) {
        println!("acquisition w/ sat_id={}", sat_id);
        let prn_code = gen_code(sat_id);
        let code: Vec<f32> = prn_code
            .iter()
            .map(|&x| if x == 0 { -1.0 } else { 1.0 })
            .collect();
        let scaled_code: Vec<f32> = code.iter().flat_map(|&x| [x, x]).collect();
        let mut spread_hz = DOPPLER_SPREAD_HZ;
        let mut estimate_hz = 0i32;
        let mut max_corr = 0.0f32;

        while spread_hz > DOPPLER_SPREAD_BINS as i32 {
            let (estimate_shift, idx, corr) = self.find_doppler_shift(iq_vec, &scaled_code, estimate_hz, spread_hz);
            if corr <= max_corr {
                break;
            }
            estimate_hz = estimate_shift;
            spread_hz = spread_hz / DOPPLER_SPREAD_BINS as i32;
            max_corr = corr;
            println!(
                    "MAX: sat_id={} -- estimate_hz={} spread_hz={:3} idx={:4} corr={:+.3e}",
                    sat_id, estimate_hz, spread_hz, idx, corr);
        }
    }

    pub fn try_acquisition(&mut self, sat_id: usize) -> Result<(), Box<dyn std::error::Error>> {
        let sample = self.get_1msec_sample();
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
