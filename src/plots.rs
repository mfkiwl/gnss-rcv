use plotters::prelude::*;
use rustfft::num_complex::Complex64;

const PLOT_FONT_SIZE: u32 = 20;
const PLOT_SIZE_X: u32 = 250;
const PLOT_SIZE_Y: u32 = 250;
const PLOT_FOLDER: &str = "plots";

pub fn plot_time_graph(
    prn: usize,
    name: &str,
    time_series: &[f64],
    y_delta: f64,
    color: &RGBColor,
) {
    let file_name = format!("{}/sat-{}-{}.png", PLOT_FOLDER, prn, name);
    let root_area = BitMapBackend::new(&file_name, (PLOT_SIZE_X, PLOT_SIZE_Y)).into_drawing_area();
    root_area.fill(&WHITE).unwrap();

    let x_max = time_series.len() as f64 * 0.001;

    let mut y_max = time_series
        .iter()
        .fold(0.0, |acc, v| if *v > acc { *v } else { acc });
    y_max += y_delta;
    let mut y_min = time_series
        .iter()
        .fold(2046.0, |acc, v| if *v < acc { *v } else { acc });
    y_min -= y_delta;

    log::warn!("plot-{}: {:.2} -> {:.2}", name, y_min, y_max);

    let mut ctx = ChartBuilder::on(&root_area)
        .set_label_area_size(LabelAreaPosition::Left, 40)
        .set_label_area_size(LabelAreaPosition::Bottom, 40)
        .caption(
            format!("sat {}: {}", prn, name),
            ("sans-serif", PLOT_FONT_SIZE),
        )
        .build_cartesian_2d(0.0..x_max, y_min..y_max)
        .unwrap();

    ctx.configure_mesh().draw().unwrap();

    ctx.draw_series(
        time_series
            .iter()
            .enumerate()
            .map(|(idx, v)| Circle::new((idx as f64 * 0.001, *v), 1, color)),
    )
    .unwrap();
}

pub fn plot_iq_scatter(prn: usize, series: &Vec<Complex64>) {
    let file_name = format!("{}/sat-{}-iq-scatter.png", PLOT_FOLDER, prn);
    let root_area = BitMapBackend::new(&file_name, (PLOT_SIZE_X, PLOT_SIZE_Y)).into_drawing_area();
    root_area.fill(&WHITE).unwrap();

    let mut x_max = series
        .iter()
        .fold(0.0, |acc, c| if c.re > acc { c.re } else { acc });
    x_max += 5.0;
    let mut x_min = series
        .iter()
        .fold(0.0, |acc, c| if c.re < acc { c.re } else { acc });
    x_min -= 5.0;

    let mut y_max = series
        .iter()
        .fold(0.0, |acc, c| if c.im > acc { c.im } else { acc });
    y_max += 5.0;
    let mut y_min = series
        .iter()
        .fold(0.0, |acc, c| if c.im < acc { c.im } else { acc });
    y_min -= 5.0;
    let mut ctx = ChartBuilder::on(&root_area)
        .set_label_area_size(LabelAreaPosition::Left, 40)
        .set_label_area_size(LabelAreaPosition::Bottom, 40)
        .caption(
            format!("sat {}: iq-scatter", prn),
            ("sans-serif", PLOT_FONT_SIZE),
        )
        .build_cartesian_2d(x_min..x_max, y_min..y_max)
        .unwrap();

    ctx.configure_mesh().draw().unwrap();

    ctx.draw_series(
        series
            .iter()
            .map(|point| Circle::new((point.re, point.im), 1, &RED)),
    )
    .unwrap();
    log::info!("printed {} dots", series.len());
}
