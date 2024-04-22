use colored::Colorize;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::time::Instant;
use rustfft::num_complex::Complex32;

use crate::util::pretty_print;

const BUFFER_SIZE: usize = 128 * 1024;

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

    pub fn get_1msec_sample(&self) -> Vec<Complex32> {
        //let num_samples = self.sample_rate / 1000;
        let num_samples = 2044;
        let mut v: Vec<Complex32> = vec![];
        for i in 0..num_samples as usize {
            v.push(self.iq_vec[i]);
        }
        v
    }

}
