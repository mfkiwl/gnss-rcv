use rustfft::num_complex::Complex32;

pub fn pretty_print(n: u64) -> String {
    let multiplier: u64;
    let unit: &str;
    let v: f64;

    if n >= 1024 * 1024 * 1024 {
        multiplier = 1024 * 1024 * 1024;
        unit = "GB";
    } else if n >= 1024 * 1024 {
        multiplier = 1024 * 1024;
        unit = "MB";
    } else if n >= 1024 {
        multiplier = 1024;
        unit = "KB";
    } else {
        multiplier = 1;
        unit = "bytes";
    }
    v = n as f64 / multiplier as f64;
    return format!("{:.3} {}", v, unit);
}

pub fn norm(v: &Vec<Complex32>) -> f32 {
    let res: f32 = v.iter().map(|x| x.norm_sqr()).sum();
    res.sqrt()
}
