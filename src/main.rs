use chrono::Local;
use colored::Colorize;
use coredump::register_panic_handler;
use gnss_rcv::plots::plot_remove_old_graph;
use log::LevelFilter;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Instant;
use structopt::StructOpt;

use gnss_rcv::code::Code;
use gnss_rcv::constants::NUM_GPS_SATS;
use gnss_rcv::receiver::GnssReceiver;
use gnss_rcv::recording::IQFileType;
use gnss_rcv::recording::IQRecording;

#[derive(StructOpt)]
#[structopt(name = "gnss-rs", about = "Gnss tracker")]
struct Options {
    #[structopt(long, help = "print gold codes")]
    print_gold_code: bool,
    #[structopt(
        short = "f",
        long,
        default_value = "resources/nov_3_time_18_48_st_ives"
    )]
    file: PathBuf,
    #[structopt(short = "l", long, default_value = "")]
    log_file: PathBuf,
    #[structopt(short = "t", long, default_value = "2xf32")]
    iq_file_type: IQFileType,
    #[structopt(long, default_value = "2046000.0")]
    fs: f64,
    #[structopt(long, default_value = "0.0")]
    fi: f64,
    #[structopt(long, default_value = "0")]
    off_msec: usize,
    #[structopt(long, default_value = "0")]
    num_msec: usize,
    #[structopt(long, default_value = "")]
    sats: String,
}

fn init_logging(log_file: &PathBuf) {
    if !log_file.as_os_str().is_empty() {
        println!("using log file: {}", log_file.display());
        let target = Box::new(File::create(log_file).expect("log file err"));
        env_logger::Builder::new()
            .target(env_logger::Target::Pipe(target))
            .filter(None, LevelFilter::Debug)
            .format(|buf, record| {
                writeln!(
                    buf,
                    "{} {} {:<5}",
                    Local::now().format("%Y-%m-%dT%H:%M:%S%.3fZ"),
                    record.level(),
                    record.args()
                )
            })
            .init();
    } else {
        env_logger::Builder::from_default_env()
            .format_target(false)
            .format_module_path(false)
            .format_timestamp_millis()
            .init();
    }
}

fn init_ctrl_c(exit_req: Arc<AtomicBool>) {
    register_panic_handler().unwrap();
    ctrlc::set_handler(move || {
        exit_req.store(true, Ordering::SeqCst);
    })
    .expect("Error setting Ctrl-C handler");
}

fn main() -> std::io::Result<()> {
    let opt = Options::from_args();
    let exit_req = Arc::new(AtomicBool::new(false));

    if opt.print_gold_code {
        Code::print_gold_codes();
        return Ok(());
    }

    init_logging(&opt.log_file);

    init_ctrl_c(exit_req.clone());

    log::warn!(
        "gnss-rs: sampling: {} fi: {} off_msec={} num_msec={}",
        format!("{:.1} KHz", opt.fs / 1000.0).bold(),
        format!("{:.1} KHz", opt.fi / 1000.0).bold(),
        opt.off_msec,
        opt.num_msec,
    );

    let mut sat_vec: Vec<u8> = vec![];
    if !opt.sats.is_empty() {
        for s in opt.sats.split(',') {
            sat_vec.push(u8::from_str_radix(s, 10).unwrap());
        }
    } else {
        for id in 0..NUM_GPS_SATS as u8 {
            sat_vec.push(id + 1);
        }
    }

    plot_remove_old_graph();
    let gold_code = Code::new();
    let recording = IQRecording::new(opt.file, opt.fs, opt.iq_file_type);
    let mut receiver = GnssReceiver::new(gold_code, recording, opt.fs, opt.fi, opt.off_msec);

    receiver.init(sat_vec);
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
        if opt.num_msec != 0 && n >= opt.num_msec {
            break;
        }
    }

    println!("GNSS terminating: {:.2} sec", ts.elapsed().as_secs_f32());

    Ok(())
}
