use rustfft::num_complex::Complex64;

pub fn norm_square(v: &Vec<Complex64>) -> f64 {
    v.iter().map(|&x| x.norm_sqr()).sum::<f64>()
}

pub fn norm(v: &Vec<Complex64>) -> f64 {
    norm_square(v).sqrt()
}

pub fn get_max_with_idx(v: &Vec<f64>) -> (usize, f64) {
    let mut max = 0.0f64;
    let mut idx = 0;
    for i in 0..v.len() {
        if v[i] > max {
            max = v[i];
            idx = i;
        }
    }
    (idx, max)
}

pub fn get_2nd_max(v: &Vec<f64>) -> f64 {
    let (i_max, max) = get_max_with_idx(v);

    let mut second = 0.0;
    let delta = 50;
    for i in 0..v.len() {
        if v[i] > second && v[i] < max && (i > i_max + delta || i < i_max - delta) {
            second = v[i];
        }
    }
    second
}
