use plotters::prelude::*;

use crate::plots::plot_iq_scatter;
use crate::plots::plot_time_graph;
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

pub struct GnssSatellite {
    prn: usize,
    param: GnssCorrelationParam,
    code_phase_offset_f64: f64,
    creation_ts_sec: f64,
    prn_code: Vec<Complex64>,
    _prn_code_fft: Vec<Complex64>,
    fft_planner: FftPlanner<f64>,

    correlation_peak_rolling_buffer: Vec<Complex64>,
    correlation_peak_angle_rolling_buffer: Vec<f64>,
    code_phase_offset_rolling_buffer: Vec<f64>,
    carrier_phase_error_rolling_buffer: Vec<f64>,
    carrier_phase_shift_rolling_buffer: Vec<f64>,
    doppler_hz_rolling_buffer: Vec<f64>,
}

impl Drop for GnssSatellite {
    fn drop(&mut self) {
        self.plot_iq_scatter();
        self.plot_code_phase_offset();
        self.plot_carrier_phase_error();
        self.plot_carrier_phase_shift();
        self.plot_correlation_peak_angles();
        self.plot_doppler_hz();
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
            code_phase_offset_rolling_buffer: vec![],
            carrier_phase_error_rolling_buffer: vec![],
            carrier_phase_shift_rolling_buffer: vec![],
            correlation_peak_rolling_buffer: vec![],
            correlation_peak_angle_rolling_buffer: vec![],
            doppler_hz_rolling_buffer: vec![],
        }
    }

    pub fn update_param(&mut self, param: &GnssCorrelationParam, ts_sec: f64) {
        log::warn!(
            "sat {}: exists: age={:.3} sec -- doppler_hz={} phase_shift={} snr={:.2}",
            self.prn,
            ts_sec - self.creation_ts_sec,
            param.doppler_hz,
            param.code_phase_offset,
            param.snr
        );
    }

    fn plot_code_phase_offset(&self) {
        plot_time_graph(
            self.prn,
            "code-phase-offset",
            self.code_phase_offset_rolling_buffer.as_slice(),
            50.0,
            &BLUE,
        );
    }

    fn plot_carrier_phase_shift(&self) {
        plot_time_graph(
            self.prn,
            "carrier-phase-shift",
            self.carrier_phase_shift_rolling_buffer.as_slice(),
            0.5,
            &BLACK,
        );
    }

    fn plot_doppler_hz(&self) {
        plot_time_graph(
            self.prn,
            "doppler-hz",
            &self.doppler_hz_rolling_buffer.as_slice(),
            10.0,
            &BLACK,
        );
    }

    fn plot_carrier_phase_error(&self) {
        let len = self.carrier_phase_error_rolling_buffer.len();
        let n = 1000;
        if len > n {
            plot_time_graph(
                self.prn,
                "carrier-phase-error",
                &self.carrier_phase_error_rolling_buffer[len - n..len],
                30.0,
                &BLACK,
            );
        }
    }

    fn plot_iq_scatter(&self) {
        let n = self.correlation_peak_rolling_buffer.len();
        if n > 1000 {
            plot_iq_scatter(self.prn, &self.correlation_peak_rolling_buffer[n - 1000..n]);
        }
    }

    fn plot_correlation_peak_angles(&self) {
        let len = self.correlation_peak_angle_rolling_buffer.len();
        let n = 50;
        plot_time_graph(
            self.prn,
            "correlation-peak-angles",
            &self.correlation_peak_angle_rolling_buffer[len - n..len],
            1.0,
            &BLACK,
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
        self.param.doppler_hz += error * beta;

        self.doppler_hz_rolling_buffer.push(self.param.doppler_hz);
        self.carrier_phase_shift_rolling_buffer
            .push(self.param.carrier_phase_shift);
        self.carrier_phase_error_rolling_buffer.push(error);
        let (_r, theta) = coherent_corr_peak.to_polar();

        self.correlation_peak_angle_rolling_buffer.push(theta);
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
        self.correlation_peak_rolling_buffer
            .push(coherent_corr_peak);
        self.track_carrier_phase_shift(coherent_corr_peak);
    }
}
