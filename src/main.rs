use std::path::PathBuf;
use structopt::StructOpt;
use colored::Colorize;
use std::fs::File;
use std::fmt;
use std::io::{BufReader, BufRead};
use serde::{Deserialize, Serialize};

const BUFFER_SIZE: usize = 128 * 1024;

#[derive(Clone, Copy, Serialize, Deserialize, PartialEq, Default)]
struct IQPair {
    pub i: i8,
    pub q: i8,
}
impl IQPair {
    pub fn new(i: i8, q: i8) -> Self {
        Self { i, q }
    }
}

impl fmt::Debug for IQPair {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "IQPair: {{ i={:4} q={:4} }}",
            self.i, self.q
        )
    }
}

#[derive(StructOpt, Debug)]
#[structopt(name = "gnss-test", about = "gnss tester")]
struct Options {
    #[structopt(long, default_value = "nov_3_time_18_48_st_ives")]
    file: PathBuf,
    #[structopt(short = "x", long, default_value = "0")]
    x: u32,
    #[structopt(short = "y", long, default_value = "0")]
    y: u32,
}

fn pretty_print(n: u64) -> String {
    let multiplier: u64;
    let unit: &str;
    let v: f64;

    if n >= 1024 * 1024 * 1024 {
        multiplier = 1024 * 1024 * 1024;
        unit = "GB";
    } else if n >= 1024 * 1024 {
        multiplier = 1024 * 1024;
        unit = "MB";
    } else if n >= 1024 {
        multiplier = 1024;
        unit = "KB";
    } else {
        multiplier = 1;
        unit = "bytes";
    }
    v = n as f64 / multiplier as f64;
    return format!("{:.3} {}", v, unit);
}

fn read_file_in_byte_chunks(path: &PathBuf) -> Result<(), Box<dyn std::error::Error>> {
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
        
        for i in 0..buf.len() / 2 {
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

fn main() {
    let opt = Options::from_args();

    println!("gnss-test: file: {} - x={} y={}", opt.file.display(), opt.x, opt.y);

    let _ = read_file_in_byte_chunks(&opt.file);

    println!("gnss-test done.");
}
