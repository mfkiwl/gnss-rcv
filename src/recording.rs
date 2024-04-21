use std::fs::File;
use std::io::{BufReader, BufRead};
use std::path::PathBuf;
use colored::Colorize;
use std::time::Instant;

use crate::util::pretty_print;
use crate::types::IQPair;

const BUFFER_SIZE: usize = 128 * 1024;

//https://wiki.gnuradio.org/index.php?title=File_Sink#Handling_File_Sink_data

pub fn read_iq_file(path: &PathBuf) -> Result<(), Box<dyn std::error::Error>> {
    let file = File::open(path)?;
    let mut n : u64 = 0;
    let mut iq_vec = Vec::new();

    println!("{}: size: {}", path.display(), pretty_print(file.metadata().unwrap().len()).bold());

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
            let bi : [u8; 4] = [buf[off+0], buf[off+1], buf[off+2], buf[off+3]];
            let bq : [u8; 4] = [buf[off+4], buf[off+5], buf[off+6], buf[off+7]];
            let i = f32::from_le_bytes(bi);
            let q = f32::from_le_bytes(bq);
            iq_vec.push(IQPair::new(i, q));
        }
        n += 1;
        if n == 1000 {
            break;
        }
        reader.consume(len);
    }

    println!("num_samples: {:?} num_read: {}", iq_vec.len(), n);
    let elapsed_msec = ts.elapsed().as_millis();
    let bw = n as f64 * BUFFER_SIZE as f64 * 1000.0 / 1024.0 / 1024.0 / elapsed_msec as f64;
    println!("elapsed: {} msec -- read bandwidth: {:.1} MB/sec", elapsed_msec, bw);
    for (i, &pair) in iq_vec.iter().enumerate().take(10) {
        println!("{}: {:?}", i, pair);
    }

    Ok(())
}
