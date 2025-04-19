use chrono::Local;
use colored::Colorize;
use coredump::register_panic_handler;
use log::LevelFilter;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;
use structopt::StructOpt;

use gnss_rcv::code::Code;
use gnss_rcv::plots::plot_remove_old_graph;
use gnss_rcv::receiver::Receiver;
use gnss_rcv::recording::IQFileType;
use gnss_rcv::state::GnssState;

#[derive(StructOpt)]
#[structopt(name = "gnss-rcv", about = "gnss-rcv: GNSS receiver")]
struct Options {
    #[structopt(
        short = "f",
        long,
        default_value = "resources/nov_3_time_18_48_st_ives"
    )]
    file: PathBuf,
    #[structopt(short = "s", long, help = "host for rtl-sdr-tcp", default_value = "")]
    hostname: String,
    #[structopt(long, help = "signal: L1CA, etc.", default_value = "L1CA")]
    sig: String,
    #[structopt(short = "d", long, help = "use rtl-sdr device")]
    use_device: bool,
    #[structopt(short = "l", long, help = "path to log file", default_value = "")]
    log_file: PathBuf,
    #[structopt(short = "t", long, help = "type of IQ file", default_value = "2xf32")]
    iq_file_type: IQFileType,
    #[structopt(long, help = "sampling frequency", default_value = "2046000.0")]
    fs: f64,
    #[structopt(long, help = "intermediate frequency", default_value = "0.0")]
    fi: f64,
    #[structopt(long, help = "offset in file", default_value = "0")]
    off_msec: usize,
    #[structopt(long, help = "duration of sample", default_value = "0")]
    num_msec: usize,
    #[structopt(long, help = "satellites to use", default_value = "")]
    sats: String,
    #[structopt(short = "-u", long, help = "use ui")]
    use_ui: bool,
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
                    "{} {} {}",
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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let opt = Options::from_args();
    let exit_req = Arc::new(AtomicBool::new(false));

    init_logging(&opt.log_file);
    init_ctrl_c(exit_req.clone());
    plot_remove_old_graph();

    log::warn!(
        "gnss-rcv: sampling: {} fi: {} off_msec={} num_msec={}",
        format!("{:.1} KHz", opt.fs / 1000.0).bold(),
        format!("{:.1} KHz", opt.fi / 1000.0).bold(),
        opt.off_msec,
        opt.num_msec,
    );
    log::warn!(
        "gnss-rcv: using signal {} frequency: {:.1} MHz",
        &opt.sig, Code::get_code_freq(&opt.sig) / 1000_000.0);


    if opt.use_ui {
        gnss_rcv::egui_main();
        return Ok(());
    }

    let mut receiver = Receiver::new(
        opt.use_device,
        &opt.hostname,
        &opt.file,
        &opt.iq_file_type,
        opt.fs,
        opt.fi,
        opt.off_msec,
        &opt.sig,
        &opt.sats,
        exit_req.clone(),
        Arc::new(Mutex::new(GnssState::new())),
    );

    let ts = Instant::now();

    receiver.run_loop(opt.num_msec);

    println!("GNSS terminating: {:.2} sec", ts.elapsed().as_secs_f32());
    exit_req.store(true, Ordering::SeqCst);

    Ok(())
}
