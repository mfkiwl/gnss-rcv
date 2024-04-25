use colored::Colorize;
use rustfft::num_complex::Complex64;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::time::Instant;

use crate::util::pretty_print;

const BUFFER_SIZE: usize = 128 * 1024;

pub struct IQRecording {
    pub file_path: PathBuf,
    pub sample_rate: usize,
    pub iq_vec: Vec<Complex64>,
}

//https://wiki.gnuradio.org/index.php?title=File_Sink#Handling_File_Sink_data

impl IQRecording {
    pub fn new(file_path: PathBuf, sample_rate: usize) -> Self {
        Self {
            file_path,
            sample_rate,
            iq_vec: vec![],
        }
    }
    pub fn read_iq_file(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let file = File::open(self.file_path.clone())?;
        let mut reader = BufReader::with_capacity(BUFFER_SIZE, &file);
        let mut n: u64 = 0;
        let ts = Instant::now();

        println!(
            "{}: size: {} - sample rate: {} KHz",
            self.file_path.display(),
            pretty_print(file.metadata().unwrap().len()).bold(),
            self.sample_rate / 1000
        );

        loop {
            let buf = reader.fill_buf()?;
            let len = buf.len();

            if len == 0 {
                break;
            }

            for off in (0..len).step_by(8) {
                let i =
                    f32::from_le_bytes([buf[off + 0], buf[off + 1], buf[off + 2], buf[off + 3]]);
                let q =
                    f32::from_le_bytes([buf[off + 4], buf[off + 5], buf[off + 6], buf[off + 7]]);
                assert!(-1.0 <= i && i <= 1.0);
                assert!(-1.0 <= q && q <= 1.0);
                self.iq_vec.push(Complex64::new(i as f64, q as f64));
            }
            n += 1;
            if n == 1000 {
                break;
            }
            reader.consume(len);
        }

        println!(
            "{}: {:?} {:.1} sec num_read_ops={}",
            "num_samples".yellow(),
            self.iq_vec.len(),
            self.iq_vec.len() as f64 / self.sample_rate as f64,
            n
        );
        let elapsed_msec = ts.elapsed().as_millis();
        let bw = n as f64 * BUFFER_SIZE as f64 * 1000.0 / 1024.0 / 1024.0 / elapsed_msec as f64;
        println!(
            "read_from_file: {} msec -- bandwidth: {:.1} MB/sec",
            elapsed_msec, bw
        );

        Ok(())
    }

    pub fn get_msec_sample(&self, num_msec: usize) -> Vec<Complex64> {
        let num_samples = num_msec * self.sample_rate / 1000;
        println!(
            "get_msec_sample: {} msec -> {} samples",
            num_msec, num_samples
        );
        let w: Vec<_> = self.iq_vec[0..num_samples].iter().cloned().collect();
        w
    }
}
