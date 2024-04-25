use rustfft::num_complex::Complex64;

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

pub fn norm_square(v: &Vec<Complex64>) -> f64 {
    v.iter().map(|&x| x.norm_sqr()).sum::<f64>()
}

pub fn norm(v: &Vec<Complex64>) -> f64 {
    norm_square(v).sqrt()
}
