use glob::glob;
use plotters::prelude::*;
use rustfft::num_complex::Complex64;

const PLOT_FONT_SIZE: u32 = 15;
const PLOT_SIZE_X: u32 = 200;
const PLOT_SIZE_Y: u32 = 200;
const PLOT_FOLDER: &str = "plots";

pub fn plot_remove_old_graph() {
    let pattern = format!("{}/*.png", PLOT_FOLDER);

    for path in glob(&pattern).unwrap() {
        match path {
            Ok(path) => {
                log::info!("Removing chart: {:?}", path.display());
                std::fs::remove_file(path).unwrap();
            }
            Err(e) => println!("{:?}", e),
        }
    }
}

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

    if time_series.len() < 10 {
        return;
    }

    let x_max = time_series.len() as f64 * 0.001;

    let mut y_max = time_series
        .iter()
        .fold(f64::MIN, |acc, v| if *v > acc { *v } else { acc });
    y_max += y_delta;
    let mut y_min = time_series
        .iter()
        .fold(f64::MAX, |acc, v| if *v < acc { *v } else { acc });
    y_min -= y_delta;

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

pub fn plot_iq_scatter(prn: usize, locked: bool, series: &[Complex64]) {
    let file_name = format!("{}/sat-{}-iq-scatter.png", PLOT_FOLDER, prn);
    let root_area = BitMapBackend::new(&file_name, (PLOT_SIZE_X, PLOT_SIZE_Y)).into_drawing_area();
    root_area.fill(&WHITE).unwrap();
    let delta = 5.0;

    if series.len() < 10 {
        return;
    }

    let mut x_max = series
        .iter()
        .fold(0.0, |acc, c| if c.re > acc { c.re } else { acc });
    x_max += delta;
    let mut x_min = series
        .iter()
        .fold(0.0, |acc, c| if c.re < acc { c.re } else { acc });
    x_min -= delta;

    let mut y_max = series
        .iter()
        .fold(0.0, |acc, c| if c.im > acc { c.im } else { acc });
    y_max += delta;
    let mut y_min = series
        .iter()
        .fold(0.0, |acc, c| if c.im < acc { c.im } else { acc });
    y_min -= delta;
    let mut ctx = ChartBuilder::on(&root_area)
        .set_label_area_size(LabelAreaPosition::Left, 40)
        .set_label_area_size(LabelAreaPosition::Bottom, 40)
        .caption(
            format!("sat {}: iq-scatter lck={}", prn, locked),
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
}
