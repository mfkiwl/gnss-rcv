use bytesize::ByteSize;
use colored::Colorize;
use std::path::PathBuf;
use structopt::StructOpt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use gnss_test::gold_code::gen_gold_codes;
use gnss_test::receiver::GnssReceiver;
use gnss_test::recording::IQFileType;
use gnss_test::recording::IQRecording;

const NUM_GPS_SATS: usize = 32;

#[derive(StructOpt)]
#[structopt(name = "gnss-test", about = "gnss tester")]
struct Options {
    #[structopt(short = "g", help = "generate gold codes")]
    gen_gold_code: bool,
    #[structopt(
        short = "f",
        long,
        default_value = "resources/nov_3_time_18_48_st_ives"
    )]
    file: PathBuf,
    #[structopt(short = "t", long, default_value = "2xf32")]
    iq_file_type: IQFileType,
    #[structopt(long, default_value = "2046000")]
    sample_rate: usize,
    #[structopt(long, default_value = "0")]
    off_msec: usize,
    #[structopt(long, default_value = "")]
    sats: String,
    #[structopt(long, short = "v")]
    verbose: bool,
}

fn main() -> std::io::Result<()> {
    let opt = Options::from_args();
    let exit_req = Arc::new(AtomicBool::new(false));
    let exit_req_clone = exit_req.clone();

    if opt.gen_gold_code {
        gen_gold_codes();
        return Ok(());
    }

    env_logger::init();
    ctrlc::set_handler(move || {
            exit_req_clone.store(true, Ordering::SeqCst);
            }).expect("Error setting Ctrl-C handler");

    println!(
        "gnss-test: {} -- {} {} sample_rate: {} off_msec={}",
        opt.file.to_str().unwrap().green(),
        ByteSize::b(opt.file.metadata().unwrap().len())
            .to_string_as(false)
            .bold(),
        opt.iq_file_type,
        format!("{} KHz", opt.sample_rate / 1000).bold(),
        opt.off_msec,
    );

    let mut sat_vec: Vec<usize> = vec![];
    if !opt.sats.is_empty() {
        for s in opt.sats.split(',') {
            sat_vec.push(usize::from_str_radix(s, 10).unwrap());
        }
    } else {
        for id in 0..NUM_GPS_SATS {
            sat_vec.push(id + 1);
        }
    }

    let mut recording = IQRecording::new(opt.file, opt.sample_rate, opt.iq_file_type);
    recording.read_iq_file().unwrap();

    let mut receiver = GnssReceiver::new(recording, opt.verbose, sat_vec);

    loop {
        let _ = receiver.process_step();
        if exit_req.load(Ordering::SeqCst) {
            break;
        }
    }

    log::warn!("GNSS terminating.");

    Ok(())
}
