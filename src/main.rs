use std::path::PathBuf;
use structopt::StructOpt;

use gnss_test::gold_code::gen_gold_codes;
use gnss_test::receiver::GpsReceiver;
use gnss_test::recording::IQRecording;

#[derive(StructOpt, Debug)]
#[structopt(name = "gnss-test", about = "gnss tester")]
struct Options {
    #[structopt(short = "g")]
    gen_gold_code: bool,
    #[structopt(long, default_value = "nov_3_time_18_48_st_ives")]
    file: PathBuf,
    #[structopt(long, default_value = "2046000")]
    sample_rate: u64,
    #[structopt(long, default_value = "0")]
    sat_id: usize,
    #[structopt(long, short = "v")]
    verbose: bool,
}

fn main() -> std::io::Result<()> {
    let opt = Options::from_args();

    println!(
        "gnss-test: file: {} - sample_rate={}",
        opt.file.display(),
        opt.sample_rate
    );

    if opt.gen_gold_code {
        println!("generating gold codes");
        gen_gold_codes();
        return Ok(());
    }

    let mut recording = IQRecording::new(opt.file, 1023 * 1000 * 2);
    recording.read_iq_file().unwrap();

    let mut receiver = GpsReceiver::new(recording);
    receiver.try_acquisition(opt.sat_id, opt.verbose).unwrap();

    println!("gnss-test done.");
    Ok(())
}
