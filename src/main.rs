use std::path::PathBuf;
use structopt::StructOpt;

#[derive(StructOpt, Debug)]
#[structopt(name = "gnss-test", about = "gnss tester")]
struct Options {
    #[structopt(long, default_value = "raw-iq.dat")]
    img_file: PathBuf,
    #[structopt(short = "x", long, default_value = "0")]
    res_x: u32,
    #[structopt(short = "y", long, default_value = "0")]
    res_y: u32,
}

fn main() {
    println!("gnss-test");
}
