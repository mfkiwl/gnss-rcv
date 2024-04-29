use colored::Colorize;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Instant;
use structopt::StructOpt;

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
    #[structopt(long, default_value = "0")]
    num_msec: usize,
    #[structopt(long, default_value = "")]
    sats: String,
}

fn main() -> std::io::Result<()> {
    let opt = Options::from_args();
    let exit_req = Arc::new(AtomicBool::new(false));
    let exit_req_clone = exit_req.clone();

    if opt.gen_gold_code {
        gen_gold_codes();
        return Ok(());
    }

    env_logger::Builder::from_default_env()
        .format_target(false)
        .format_module_path(false)
        .format_timestamp_millis()
        .init();

    ctrlc::set_handler(move || {
        exit_req_clone.store(true, Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");

    println!(
        "gnss-test: sampling: {} off_msec={} num_msec={}",
        format!("{} KHz", opt.sample_rate / 1000).bold(),
        opt.off_msec,
        opt.num_msec,
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

    let recording = IQRecording::new(opt.file, opt.sample_rate, opt.iq_file_type);
    let mut receiver = GnssReceiver::new(recording, opt.off_msec, sat_vec);
    let mut n = 0;
    let ts = Instant::now();

    loop {
        if receiver.process_step().is_err() {
            break;
        }
        if exit_req.load(Ordering::SeqCst) {
            break;
        }
        n += 1;
        if opt.num_msec != 0 && n > opt.num_msec {
            break;
        }
    }

    log::warn!("GNSS terminating: {:.2} sec", ts.elapsed().as_secs_f32());

    Ok(())
}
