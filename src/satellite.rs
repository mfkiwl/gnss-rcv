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

pub struct GnssSatellite {
    prn: usize,
    param: GnssCorrelationParam,
    phase_offset_f64: f64,
    creation_ts_sec: f64,
    prn_code: Vec<Complex64>,
    _prn_code_fft: Vec<Complex64>,
    fft_planner: FftPlanner<f64>,
    correlation_peaks_rolling_buffer: Vec<Complex64>,
    phase_offset_rolling_buffer: Vec<f64>,
}

impl Drop for GnssSatellite {
    fn drop(&mut self) {
        self.plot_iq_scatter();
        self.plot_doppler_shift();
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
                prn, param.doppler_hz, param.phase_offset, param.snr, param.carrier_phase_shift
            )
            .green()
        );
        Self {
            prn,
            prn_code,
            _prn_code_fft: prn_code_fft,
            param,
            phase_offset_f64: param.phase_offset as f64,
            creation_ts_sec: ts_sec,
            fft_planner: FftPlanner::new(),
            correlation_peaks_rolling_buffer: vec![],
            phase_offset_rolling_buffer: vec![],
        }
    }

    pub fn update_param(&mut self, param: &GnssCorrelationParam, ts_sec: f64) {
        self.param = *param;

        log::warn!(
            "sat {}: exists: age={:.3} sec -- doppler_hz={} phase_shift={} snr={:.2}",
            self.prn,
            ts_sec - self.creation_ts_sec,
            param.doppler_hz,
            param.phase_offset,
            param.snr
        );
    }

    fn plot_doppler_shift(&self) {
        let file_name = format!("plots/sat-{}-doppler-shift.png", self.prn);
        let root_area = BitMapBackend::new(&file_name, (400, 400)).into_drawing_area();
        root_area.fill(&WHITE).unwrap();

        let y_delta = 50.0;
        let x_max = self.phase_offset_rolling_buffer.len() as f64 * 0.001;

        let mut y_max =
            self.phase_offset_rolling_buffer
                .iter()
                .fold(0.0, |acc, v| if *v > acc { *v } else { acc });
        y_max += y_delta;
        let mut y_min = self
            .phase_offset_rolling_buffer
            .iter()
            .fold(2046.0, |acc, v| if *v < acc { *v } else { acc });
        if y_min > y_delta {
            y_min -= y_delta;
        } else {
            y_min = 0.0;
        }

        let mut ctx = ChartBuilder::on(&root_area)
            .set_label_area_size(LabelAreaPosition::Left, 40)
            .set_label_area_size(LabelAreaPosition::Bottom, 40)
            .caption(format!("sat {}", self.prn), ("sans-serif", 40))
            .build_cartesian_2d(0.0..x_max, y_min..y_max)
            .unwrap();

        ctx.configure_mesh().draw().unwrap();

        ctx.draw_series(
            self.phase_offset_rolling_buffer
                .iter()
                .enumerate()
                .map(|(idx, v)| Circle::new((idx as f64 * 0.001, *v), 1, &RED)),
        )
        .unwrap();
        log::info!(
            "doppler_shift: printed {} items",
            self.phase_offset_rolling_buffer.len()
        );
    }

    fn plot_iq_scatter(&self) {
        let file_name = format!("plots/sat-{}-iq-scatter.png", self.prn);
        let root_area = BitMapBackend::new(&file_name, (400, 400)).into_drawing_area();
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
            .caption(format!("sat {}", self.prn), ("sans-serif", 40))
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
        if max_idx.abs_diff(self.param.phase_offset) > 3 {
            log::warn!(
                "sat-{}: {}: peak at: {} but phase_offset={}",
                self.prn,
                format!("XXX code tracking error").red(),
                max_idx,
                self.param.phase_offset,
            );
        }
    }

    fn track_code_phase_offset(&mut self, doppler_shifted_samples: &Vec<Complex64>, ts_sec: f64) {
        let vec_len = doppler_shifted_samples.len();
        let mut prn_code_early = self.prn_code.clone();
        let mut prn_code_late = self.prn_code.clone();

        if self.param.phase_offset >= 1 {
            prn_code_early.rotate_right(self.param.phase_offset - 1);
        } else {
            prn_code_early.rotate_left(1);
        }
        prn_code_late.rotate_right((self.param.phase_offset + 1) % vec_len);

        let corr_early = correlate_vec(&doppler_shifted_samples, &prn_code_early);
        let corr_late = correlate_vec(&doppler_shifted_samples, &prn_code_late);

        let discriminator = (corr_early.norm_sqr() - corr_late.norm_sqr()) / 2.0;

        self.phase_offset_f64 -= discriminator * 0.002;
        if self.phase_offset_f64 < 0.0 {
            self.phase_offset_f64 += get_num_samples_per_msec() as f64;
        }
        self.phase_offset_f64 = self.phase_offset_f64 % get_num_samples_per_msec() as f64;
        self.param.phase_offset =
            self.phase_offset_f64.round() as usize % get_num_samples_per_msec();
        self.phase_offset_rolling_buffer.push(self.phase_offset_f64);

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
        //let vec_len = doppler_shifted_sample.len();
        let mut prn_code_prompt = self.prn_code.clone();
        assert!(self.param.phase_offset < prn_code_prompt.len());
        self.prn_code.clone().rotate_right(self.param.phase_offset);

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
        let navigation_bit_pseudosymbol_value = coherent_correlation_peak.re.signum() as i8;

        let delay_phase_sec = self.param.phase_offset as f64 / 2046.0 * 0.001;
        (
            navigation_bit_pseudosymbol_value,
            correlation_strength,
            ts_sec + delay_phase_sec,
            coherent_correlation_peak,
        )
    }

    fn calc_loop_filter_params(&self, loop_bw: f64) -> (f64, f64) {
        let ts_per_sample = 1.0 / 2046000.0;
        let damping_factor = 1.0 / 2.0f64.sqrt();
        let loop_gain_phase = 4.0 * damping_factor * loop_bw * ts_per_sample;
        let loop_gain_freq = 4.0 * loop_bw * loop_bw * ts_per_sample;
        (loop_gain_phase, loop_gain_freq)
    }

    //   return loop_gain_phase, loop_gain_freq
    fn is_carrier_tracker_locked(&self) -> bool {
        true
    }

    fn track_carrier(&mut self, coherent_corr_peak: Complex64) {
        let _error = coherent_corr_peak.re * coherent_corr_peak.im;

        if self.is_carrier_tracker_locked() {
        } else {
        }
    }

    pub fn process_samples(&mut self, sample: &IQSample) {
        log::info!(
            "sat-{}: processing: ts_sec={:.4} sec num_samples={}: doppler={} phase={}",
            self.prn,
            sample.ts_sec,
            sample.iq_vec.len(),
            self.param.doppler_hz,
            self.param.phase_offset,
        );

        let mut signal = sample.iq_vec.clone();
        doppler_shift(
            self.param.doppler_hz,
            sample.ts_sec,
            &mut signal,
            -self.param.carrier_phase_shift,
            sample.sample_rate,
        );

        let doppler_shifted_samples = signal;

        self.track_code_phase_offset(&doppler_shifted_samples, sample.ts_sec);

        let (_nav_bit, _corr_strength, _ts_sec, coherent_corr_peak) =
            self.track_symbol(&doppler_shifted_samples, sample.ts_sec);
        self.correlation_peaks_rolling_buffer
            .push(coherent_corr_peak);
        self.track_carrier(coherent_corr_peak);

        log::info!(
            "sat {} -- doppler_hz={} phase_offset={} polar: carrier_phase_shift={}",
            self.prn,
            self.param.doppler_hz,
            self.param.phase_offset,
            self.param.carrier_phase_shift
        );
    }
}
