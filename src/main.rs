use std::path::PathBuf;
use structopt::StructOpt;
use gnss_test::recording::read_iq_file;
use gnss_test::gold_code::gen_gold_codes;

#[derive(StructOpt, Debug)]
#[structopt(name = "gnss-test", about = "gnss tester")]
struct Options {
    #[structopt(short="g")]
    gen_gold_code: bool,
    #[structopt(long, default_value = "nov_3_time_18_48_st_ives")]
    file: PathBuf,
    #[structopt(short = "s", long, default_value = "0")]
    sample_rate: u64,
}


fn main() -> std::io::Result<()> {
    let opt = Options::from_args();

    println!("gnss-test: file: {} - sample_rate={}", opt.file.display(), opt.sample_rate);

    if opt.gen_gold_code {
        println!("generating gold codes");
        gen_gold_codes();
        return Ok(());
    }

    let _ = read_iq_file(&opt.file);

    println!("gnss-test done.");
    Ok(())
}
