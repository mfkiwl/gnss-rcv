use chrono::Local;
use colored::Colorize;
use coredump::register_panic_handler;
use gnss_rcv::device::RtlSdrDevice;
use gnss_rcv::network::RtlSdrTcp;
use gnss_rcv::plots::plot_remove_old_graph;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use log::LevelFilter;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;
use structopt::StructOpt;

use gnss_rcv::code::Code;
use gnss_rcv::receiver::ReadIQFn;
use gnss_rcv::receiver::Receiver;
use gnss_rcv::recording::IQFileType;
use gnss_rcv::recording::IQRecording;

#[derive(StructOpt)]
#[structopt(name = "gnss-rcv", about = "GNSS receiver")]
struct Options {
    #[structopt(long, help = "print gold codes")]
    print_gold_code: bool,
    #[structopt(
        short = "f",
        long,
        default_value = "resources/nov_3_time_18_48_st_ives"
    )]
    file: PathBuf,
    #[structopt(short = "s", long, help = "host for rtl-sdr-tcp", default_value = "")]
    hostname: String,
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

fn get_sat_list(opt: &Options) -> Vec<SV> {
    let mut sat_vec: Vec<SV> = vec![];
    if !opt.sats.is_empty() {
        for s in opt.sats.split(',') {
            let prn = s.parse::<u8>().unwrap();
            sat_vec.push(SV::new(Constellation::GPS, prn));
        }
    } else {
        for prn in 1..=32_u8 {
            sat_vec.push(SV::new(Constellation::GPS, prn));
        }
        let use_sbas = false;
        if use_sbas {
            for prn in 120..=158_u8 {
                sat_vec.push(SV::new(Constellation::GPS, prn));
            }
        }
    }
    sat_vec
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let opt = Options::from_args();
    let exit_req = Arc::new(AtomicBool::new(false));

    if opt.print_gold_code {
        Code::print_l1ca_codes();
        return Ok(());
    }

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

    let sat_vec: Vec<SV> = get_sat_list(&opt);
    let read_fn: Box<ReadIQFn>;
    let sig = "L1CA";

    if opt.use_device {
        let res = RtlSdrDevice::new(sig, opt.fs);
        if res.is_err() {
            log::warn!("Failed to open rtl-sdr device.");
            return Ok(());
        }
        let mut dev = res.unwrap();

        read_fn = Box::new(move |_off_samples, num_samples| dev.read_iq_data(num_samples));
    } else if !opt.hostname.is_empty() {
        let mut net = RtlSdrTcp::new(&opt.hostname, exit_req.clone(), sig, opt.fs)?;

        log::warn!("Using rtl_tcp backend: {}", opt.hostname);
        read_fn = Box::new(move |_off_samples, num_samples| net.read_iq_data(num_samples));
    } else {
        let mut recording = IQRecording::new(opt.file, opt.fs, opt.iq_file_type);

        read_fn = Box::new(move |off_samples, num_samples| {
            recording.read_iq_file(off_samples, num_samples)
        });
    }

    if opt.use_ui {
        gnss_rcv::egui_main();
        return Ok(());
    }

    let mut receiver = Receiver::new(read_fn, opt.fs, opt.fi, opt.off_msec);

    receiver.init(sig, sat_vec);
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
    exit_req.store(true, Ordering::SeqCst);

    Ok(())
}
