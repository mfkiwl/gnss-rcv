use colored::Colorize;
use std::path::PathBuf;
use structopt::StructOpt;
use std::collections::HashSet;

use gnss_test::util::pretty_print;
use gnss_test::gold_code::gen_gold_codes;
use gnss_test::receiver::GpsReceiver;
use gnss_test::recording::IQRecording;

#[derive(StructOpt, Debug)]
#[structopt(name = "gnss-test", about = "gnss tester")]
struct Options {
    #[structopt(short = "g", help = "generate gold codes")]
    gen_gold_code: bool,
    #[structopt(short = "f", long)]
    file: PathBuf,
    #[structopt(long)]
    format_int16: bool,
    #[structopt(long, default_value = "2046000")]
    sample_rate: usize,
    #[structopt(long, default_value = "0")]
    off_msec: usize,
    #[structopt(long, default_value="")]
    sats: String,
    #[structopt(long, short = "v")]
    verbose: bool,
}

fn main() -> std::io::Result<()> {
    let opt = Options::from_args();

    println!(
        "gnss-test: {} -- {} -- sample_rate: {} off_msec={}",
        opt.file.to_str().unwrap().green(),
        pretty_print(opt.file.metadata().unwrap().len()).bold(),
        format!("{} KHz", opt.sample_rate / 1000).bold(),
        opt.off_msec,
    );

    if opt.gen_gold_code {
        gen_gold_codes();
        return Ok(());
    }

    let mut sat_set : HashSet<usize> = HashSet::new();
    if !opt.sats.is_empty() {
        for s in opt.sats.split(',') {
            sat_set.insert(usize::from_str_radix(s, 10).unwrap());
        }
    }

    let mut recording = IQRecording::new(opt.file, opt.sample_rate, opt.format_int16);
    recording.read_iq_file().unwrap();

    let mut receiver = GpsReceiver::new(recording, opt.verbose);
    receiver.try_acquisition(opt.off_msec, sat_set).unwrap();
    Ok(())
}
