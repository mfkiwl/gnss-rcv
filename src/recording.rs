use std::fs::File;
use std::io::{BufReader, BufRead};
use std::path::PathBuf;
use colored::Colorize;

use crate::util::pretty_print;
use crate::types::IQPair;

const BUFFER_SIZE: usize = 128 * 1024;

pub fn read_iq_file(path: &PathBuf) -> Result<(), Box<dyn std::error::Error>> {
    let file = File::open(path)?;
    let mut n : u32 = 0;
    let mut iq_vec = Vec::new();

    println!("{}: size: {}", path.display(), pretty_print(file.metadata().unwrap().len()).bold());

    let mut reader = BufReader::with_capacity(BUFFER_SIZE, file);

    loop {
        let buf = reader.fill_buf()?;
        let len = buf.len();

        if len == 0 {
            break;
        }
        
        for i in 0..len / 2 {
            iq_vec.push(IQPair::new(buf[2 * i] as i8, buf[2 * i + 1] as i8));
        }
        n += 1;
        if n == 1 {
            break;
        }
        reader.consume(len);
    }
    for (i, &pair) in iq_vec.iter().enumerate() {
        println!("{}: {:?}", i, pair);
    }


    Ok(())
}
