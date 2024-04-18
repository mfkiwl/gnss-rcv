use std::path::PathBuf;
use structopt::StructOpt;
use colored::Colorize;

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

fn main() {
    let opt = Options::from_args();

    println!("gnss-test: file: {} - x={} y={}", opt.file.display(), opt.x, opt.y);
}
