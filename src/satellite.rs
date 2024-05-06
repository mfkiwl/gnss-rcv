use plotters::prelude::*;

use crate::types::GnssCorrelationParam;
use crate::types::IQSample;
use crate::util::calc_correlation;
use crate::util::correlate_vec;
use crate::util::doppler_shift;
use crate::util::get_max_with_idx;
use crate::util::get_normalized_correlation_strength;
use crate::util::get_num_samples_per_msec;
use colored::Colorize;
use rustfft::num_complex::Complex64;
use rustfft::FftPlanner;

const PI: f64 = std::f64::consts::PI;
const PLOT_FONT_SIZE: u32 = 20;
const PLOT_SIZE_X: u32 = 300;
const PLOT_SIZE_Y: u32 = 300;
const PLOT_FOLDER: &str = "plots";

pub struct GnssSatellite {
    prn: usize,
    param: GnssCorrelationParam,
    code_phase_offset_f64: f64,
    creation_ts_sec: f64,
    prn_code: Vec<Complex64>,
    _prn_code_fft: Vec<Complex64>,
    fft_planner: FftPlanner<f64>,

    correlation_peaks_rolling_buffer: Vec<Complex64>,
    code_phase_offset_rolling_buffer: Vec<f64>,
    carrier_wave_phase_errors_rolling_buffer: Vec<f64>,
    correlation_peak_angles_rolling_buffer: Vec<f64>,
}

impl Drop for GnssSatellite {
    fn drop(&mut self) {
        self.plot_iq_scatter();
        self.plot_code_phase_offset();
        self.plot_carrier_wave_phase_errors();
        self.plot_correlation_peak_angles();
    }
}

impl GnssSatellite {
    pub fn new(
        prn: usize,
        prn_code: Vec<Complex64>,
        prn_code_fft: Vec<Complex64>,
        param: GnssCorrelationParam,
        ts_sec: f64,
    ) -> Self {
        log::warn!(
            "{}",
            format!(
                "sat {}: new: doppler={} phase_shift={} snr={:.2} carrier_phase_shift={:.1}",
                prn,
                param.doppler_hz,
                param.code_phase_offset,
                param.snr,
                param.carrier_phase_shift
            )
            .green()
        );
        Self {
            prn,
            prn_code,
            _prn_code_fft: prn_code_fft,
            param,
            code_phase_offset_f64: param.code_phase_offset as f64,
            creation_ts_sec: ts_sec,
            fft_planner: FftPlanner::new(),
            correlation_peaks_rolling_buffer: vec![],
            code_phase_offset_rolling_buffer: vec![],
            carrier_wave_phase_errors_rolling_buffer: vec![],
            correlation_peak_angles_rolling_buffer: vec![],
        }
    }

    pub fn update_param(&mut self, param: &GnssCorrelationParam, ts_sec: f64) {
        self.param = *param;

        log::warn!(
            "sat {}: exists: age={:.3} sec -- doppler_hz={} phase_shift={} snr={:.2}",
            self.prn,
            ts_sec - self.creation_ts_sec,
            param.doppler_hz,
            param.code_phase_offset,
            param.snr
        );
    }

    fn plot_time_graph(&self, name: &str, time_series: &[f64], y_delta: f64) {
        let file_name = format!("{}/sat-{}-{}.png", PLOT_FOLDER, self.prn, name);
        let root_area =
            BitMapBackend::new(&file_name, (PLOT_SIZE_X, PLOT_SIZE_Y)).into_drawing_area();
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

        log::warn!("plot-{}: {} -> {}", name, y_min, y_max);

        let mut ctx = ChartBuilder::on(&root_area)
            .set_label_area_size(LabelAreaPosition::Left, 40)
            .set_label_area_size(LabelAreaPosition::Bottom, 40)
            .caption(
                format!("sat {}: {}", self.prn, name),
                ("sans-serif", PLOT_FONT_SIZE),
            )
            .build_cartesian_2d(0.0..x_max, y_min..y_max)
            .unwrap();

        ctx.configure_mesh().draw().unwrap();

        ctx.draw_series(
            time_series
                .iter()
                .enumerate()
                .map(|(idx, v)| Circle::new((idx as f64 * 0.001, *v), 1, &RED)),
        )
        .unwrap();
    }

    fn plot_code_phase_offset(&self) {
        let len = self.code_phase_offset_rolling_buffer.len();
        let n = 1000;
        self.plot_time_graph(
            "code-phase-offset",
            &self.code_phase_offset_rolling_buffer[len - n..len],
            5.0,
        );
    }

    fn plot_carrier_wave_phase_errors(&self) {
        let len = self.carrier_wave_phase_errors_rolling_buffer.len();
        let n = 1000;
        self.plot_time_graph(
            "carrier-phase-error",
            &self.carrier_wave_phase_errors_rolling_buffer[len - n..len],
            30.0,
        );
    }

    fn plot_correlation_peak_angles(&self) {
        let len = self.correlation_peak_angles_rolling_buffer.len();
        let n = 50;
        self.plot_time_graph(
            "correlation-peak-angles",
            &self.correlation_peak_angles_rolling_buffer[len - n..len],
            1.0,
        );
    }

    fn plot_iq_scatter(&self) {
        let file_name = format!("{}/sat-{}-iq-scatter.png", PLOT_FOLDER, self.prn);
        let root_area =
            BitMapBackend::new(&file_name, (PLOT_SIZE_X, PLOT_SIZE_Y)).into_drawing_area();
        root_area.fill(&WHITE).unwrap();

        let mut x_max = self
            .correlation_peaks_rolling_buffer
            .iter()
            .fold(0.0, |acc, c| if c.re > acc { c.re } else { acc });
        x_max += 5.0;
        let mut x_min = self
            .correlation_peaks_rolling_buffer
            .iter()
            .fold(0.0, |acc, c| if c.re < acc { c.re } else { acc });
        x_min -= 5.0;

        let mut y_max = self
            .correlation_peaks_rolling_buffer
            .iter()
            .fold(0.0, |acc, c| if c.im > acc { c.im } else { acc });
        y_max += 5.0;
        let mut y_min = self
            .correlation_peaks_rolling_buffer
            .iter()
            .fold(0.0, |acc, c| if c.im < acc { c.im } else { acc });
        y_min -= 5.0;
        let mut ctx = ChartBuilder::on(&root_area)
            .set_label_area_size(LabelAreaPosition::Left, 40)
            .set_label_area_size(LabelAreaPosition::Bottom, 40)
            .caption(
                format!("sat {}: iq-scatter", self.prn),
                ("sans-serif", PLOT_FONT_SIZE),
            )
            .build_cartesian_2d(x_min..x_max, y_min..y_max)
            .unwrap();

        ctx.configure_mesh().draw().unwrap();

        ctx.draw_series(
            self.correlation_peaks_rolling_buffer
                .iter()
                .map(|point| Circle::new((point.re, point.im), 1, &RED)),
        )
        .unwrap();
        log::info!(
            "printed {} dots",
            self.correlation_peaks_rolling_buffer.len()
        );
    }

    pub fn verify_correlation_peak(&self, doppler_shifted_samples: &Vec<Complex64>) {
        let mut res = vec![];
        let mut p = self.prn_code.clone();
        for _i in 0..get_num_samples_per_msec() {
            let c = correlate_vec(&doppler_shifted_samples, &p);
            res.push(c.norm_sqr());
            p.rotate_right(1);
        }
        let mut max_idx = 0;
        let mut max_res = 0.0;
        for i in 0..res.len() {
            if res[i] > max_res {
                max_res = res[i];
                max_idx = i;
            }
        }
        if max_idx.abs_diff(self.param.code_phase_offset) > 3 {
            log::warn!(
                "sat-{}: {}: peak at: {} but phase_offset={}",
                self.prn,
                format!("XXX code tracking error").red(),
                max_idx,
                self.param.code_phase_offset,
            );
        }
    }

    fn track_code_phase_offset(&mut self, doppler_shifted_samples: &Vec<Complex64>, ts_sec: f64) {
        let vec_len = doppler_shifted_samples.len();
        let mut prn_code_early = self.prn_code.clone();
        let mut prn_code_late = self.prn_code.clone();

        if self.param.code_phase_offset >= 1 {
            prn_code_early.rotate_right(self.param.code_phase_offset - 1);
        } else {
            prn_code_early.rotate_left(1);
        }
        prn_code_late.rotate_right((self.param.code_phase_offset + 1) % vec_len);

        let corr_early = correlate_vec(&doppler_shifted_samples, &prn_code_early);
        let corr_late = correlate_vec(&doppler_shifted_samples, &prn_code_late);

        let discriminator = (corr_early.norm_sqr() - corr_late.norm_sqr()) / 2.0;

        self.code_phase_offset_f64 -= discriminator * 0.002;
        if self.code_phase_offset_f64 < 0.0 {
            self.code_phase_offset_f64 += get_num_samples_per_msec() as f64;
        }
        self.code_phase_offset_f64 = self.code_phase_offset_f64 % get_num_samples_per_msec() as f64;
        self.param.code_phase_offset =
            self.code_phase_offset_f64.round() as usize % get_num_samples_per_msec();
        self.code_phase_offset_rolling_buffer
            .push(self.code_phase_offset_f64);

        // verify tracking error every second
        if (ts_sec - ts_sec.round() as f64).abs() < 0.001 {
            self.verify_correlation_peak(&doppler_shifted_samples);
        }
    }

    fn track_symbol(
        &mut self,
        doppler_shifted_sample: &Vec<Complex64>,
        ts_sec: f64,
    ) -> (i8, f64, f64, Complex64) {
        let mut prn_code_prompt = self.prn_code.clone();
        assert!(self.param.code_phase_offset < prn_code_prompt.len());
        prn_code_prompt.rotate_right(self.param.code_phase_offset);

        let fft_fw = self.fft_planner.plan_fft_forward(prn_code_prompt.len());
        fft_fw.process(&mut prn_code_prompt);
        let prn_code_prompt_fft = prn_code_prompt;

        let corr = calc_correlation(
            &mut self.fft_planner,
            doppler_shifted_sample,
            &prn_code_prompt_fft,
        );
        let non_coherent_corr: Vec<_> = corr.iter().map(|x| x.norm()).collect();
        let correlation_strength = get_normalized_correlation_strength(&non_coherent_corr);
        let (idx, _max) = get_max_with_idx(&non_coherent_corr);

        let coherent_correlation_peak = corr[idx];
        let navigation_bit = coherent_correlation_peak.re.signum() as i8;

        let delay_phase_sec = self.param.code_phase_offset as f64 / 2046.0 * 0.001;
        (
            navigation_bit,
            correlation_strength,
            ts_sec + delay_phase_sec,
            coherent_correlation_peak,
        )
    }

    fn calc_loop_filter_params(&self, loop_bw: f64) -> (f64, f64) {
        let ts_per_sample = 1.0 / 2046000.0; // sample_rate
        let damping_factor = 1.0 / 2.0f64.sqrt();
        let loop_gain_phase = 4.0 * damping_factor * loop_bw * ts_per_sample;
        let loop_gain_freq = 4.0 * loop_bw * loop_bw * ts_per_sample;
        (loop_gain_phase, loop_gain_freq)
    }

    //   return loop_gain_phase, loop_gain_freq
    fn is_carrier_tracker_locked(&self) -> bool {
        false
    }

    fn track_carrier_phase_shift(&mut self, coherent_corr_peak: Complex64) {
        let error = coherent_corr_peak.re * coherent_corr_peak.im;
        let loop_bw;

        if self.is_carrier_tracker_locked() {
            loop_bw = 3.0;
        } else {
            loop_bw = 6.0;
        }
        let (alpha, beta) = self.calc_loop_filter_params(loop_bw);
        self.param.carrier_phase_shift += error * alpha;
        self.param.carrier_phase_shift = self.param.carrier_phase_shift % (2.0 * PI);

        self.param.doppler_hz += (error * beta) as i32;
        self.carrier_wave_phase_errors_rolling_buffer.push(error);
        let (_r, theta) = coherent_corr_peak.to_polar();

        self.correlation_peak_angles_rolling_buffer.push(theta);
    }

    pub fn process_samples(&mut self, sample: &IQSample) {
        log::info!(
            "sat-{}: processing: ts_sec={:.4} sec num_samples={}: doppler={} code_phase_off={} carrier_phase_shift={:.2}",
            self.prn,
            sample.ts_sec,
            sample.iq_vec.len(),
            self.param.doppler_hz,
            self.param.code_phase_offset,
            self.param.carrier_phase_shift,
        );

        let mut signal = sample.iq_vec.clone();
        doppler_shift(
            self.param.doppler_hz,
            sample.ts_sec,
            &mut signal,
            self.param.carrier_phase_shift,
            sample.sample_rate,
        );

        let doppler_shifted_samples = signal;

        self.track_code_phase_offset(&doppler_shifted_samples, sample.ts_sec);

        let (_nav_bit, _corr_strength, _ts_sec, coherent_corr_peak) =
            self.track_symbol(&doppler_shifted_samples, sample.ts_sec);
        self.correlation_peaks_rolling_buffer
            .push(coherent_corr_peak);
        self.track_carrier_phase_shift(coherent_corr_peak);
    }
}
