use plotters::prelude::*;

use crate::constants::PI;
use crate::plots::plot_iq_scatter;
use crate::plots::plot_time_graph;
use crate::types::GnssCorrelationParam;
use crate::types::IQSample;
use crate::util::calc_correlation;
use crate::util::correlate_vec;
use crate::util::doppler_shift;

use crate::util::get_max_with_idx;
use crate::util::get_num_samples_per_msec;
use crate::util::vector_mean_complex;
use crate::util::vector_variance;
use colored::Colorize;
use rustfft::num_complex::Complex64;
use rustfft::FftPlanner;

#[derive(PartialEq)]
enum TrackState {
    LOCKED,
    SEARCHING,
}

pub struct GnssSatellite {
    prn: usize,
    param: GnssCorrelationParam,
    code_phase_offset_f64: f64,
    creation_ts_sec: f64,
    prn_code: Vec<Complex64>,
    fft_planner: FftPlanner<f64>,
    state: TrackState,

    locked_ts: f64,
    total_locked_ts: f64,
    last_plot_ts: f64,

    correlation_peak_rolling_buffer: Vec<Complex64>,
    correlation_peak_angle_rolling_buffer: Vec<f64>,
    code_phase_offset_rolling_buffer: Vec<f64>,
    carrier_phase_error_rolling_buffer: Vec<f64>,
    carrier_phase_shift_rolling_buffer: Vec<f64>,
    doppler_hz_rolling_buffer: Vec<f64>,
}

impl Drop for GnssSatellite {
    fn drop(&mut self) {
        self.update_all_plots();
    }
}

impl GnssSatellite {
    pub fn new(
        prn: usize,
        prn_code: Vec<Complex64>,
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
            last_plot_ts: 0.0,

            locked_ts: 0.0,
            total_locked_ts: 0.0,

            state: TrackState::SEARCHING,
        }
    }

    fn update_locked_state(&mut self, locked: bool, ts_sec: f64) {
        let is_locked = self.state == TrackState::LOCKED;
        if is_locked && !locked {
            let duration_sec = ts_sec - self.locked_ts;
            self.total_locked_ts += duration_sec;
            if duration_sec > 1.0 {
                log::warn!(
                    "sat-{} locked for {:.1} sec. ts={:.1} sec -- total: {:.1} sec",
                    self.prn,
                    duration_sec,
                    ts_sec,
                    self.total_locked_ts,
                );
            }
            self.state = TrackState::SEARCHING;
            self.locked_ts = 0.0;
        } else if !is_locked && locked {
            self.state = TrackState::LOCKED;
            self.locked_ts = ts_sec;
        }
    }

    pub fn update_param(&mut self, param: &GnssCorrelationParam, ts_sec: f64) {
        log::warn!(
            "sat {}: exists: age={:.3} sec -- doppler_hz={} code_phase_shift={} snr={:.2}",
            self.prn,
            ts_sec - self.creation_ts_sec,
            param.doppler_hz,
            param.code_phase_offset,
            param.snr
        );
    }

    fn update_all_plots(&self) {
        self.plot_iq_scatter();
        self.plot_code_phase_offset();
        self.plot_carrier_phase_error();
        self.plot_carrier_phase_shift();
        self.plot_correlation_peak_angles();
        self.plot_doppler_hz();
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
        let n = usize::min(len, 10000);

        plot_time_graph(
            self.prn,
            "carrier-phase-error",
            &self.carrier_phase_error_rolling_buffer[len - n..len],
            30.0,
            &BLACK,
        );
    }

    fn plot_iq_scatter(&self) {
        let len = self.correlation_peak_rolling_buffer.len();
        let n = usize::min(len, 1000);
        plot_iq_scatter(
            self.prn,
            self.state == TrackState::LOCKED,
            &self.correlation_peak_rolling_buffer[len - n..len],
        );
    }

    fn plot_correlation_peak_angles(&self) {
        let len = self.correlation_peak_angle_rolling_buffer.len();
        let n = usize::min(500, len);
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

    fn track_symbol(&mut self, doppler_shifted_sample: &Vec<Complex64>) -> Complex64 {
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
        let (idx, _max) = get_max_with_idx(&non_coherent_corr);

        let coherent_correlation_peak = corr[idx];

        coherent_correlation_peak
    }

    fn is_carrier_tracker_locked(&mut self) -> bool {
        let len = self.carrier_phase_error_rolling_buffer.len();

        let n = 250;
        if len < n {
            return false;
        }
        let phase_error_slice = &self.carrier_phase_error_rolling_buffer[len - n..len];

        let variance = vector_variance(phase_error_slice);
        if variance > 900.0 {
            return false;
        }

        let correlation_peaks = &self.correlation_peak_rolling_buffer[len - n..len];
        if correlation_peaks.len() < 2 {
            return false;
        }

        let mut pos_corr = vec![];
        let mut neg_corr = vec![];
        for v in correlation_peaks {
            if v.re <= 0.0 {
                neg_corr.push(*v);
            } else {
                pos_corr.push(*v);
            }
        }

        let avg_pos_corr = vector_mean_complex(pos_corr.as_slice());

        let mut neg_i_corr = vec![];
        let mut pos_i_corr = vec![];
        for v in neg_corr {
            neg_i_corr.push(v.re);
        }
        for v in pos_corr {
            pos_i_corr.push(v.re);
        }
        let mut neg_i_corr_var = 0.0;
        let mut pos_i_corr_var = 0.0;
        if neg_i_corr.len() >= 2 {
            neg_i_corr_var = vector_variance(&neg_i_corr);
        }
        if pos_i_corr.len() >= 2 {
            pos_i_corr_var = vector_variance(&pos_i_corr);
        }
        let mean_i_peak_variance = 0.5 * (neg_i_corr_var + pos_i_corr_var);
        if mean_i_peak_variance > 2.0 {
            return false;
        }

        let (_r, theta) = avg_pos_corr.to_polar();
        let degree = theta * 360.0 / (2.0 * PI);

        if degree.abs() < 25.0 {
            return true;
        }

        false
    }

    fn calc_loop_filter_params(&self, loop_bw: f64) -> (f64, f64) {
        let ts_per_sample = 1.0 / 2046000.0; // sample_rate
        let damping_factor = 1.0 / 2.0f64.sqrt();
        let loop_gain_phase = 4.0 * damping_factor * loop_bw * ts_per_sample;
        let loop_gain_freq = 4.0 * loop_bw * loop_bw * ts_per_sample;
        (loop_gain_phase, loop_gain_freq)
    }

    fn track_carrier_phase(&mut self, coherent_corr_peak: Complex64, ts_sec: f64) {
        let error = coherent_corr_peak.re * coherent_corr_peak.im;
        let loop_bw;

        let locked = self.is_carrier_tracker_locked();
        self.update_locked_state(locked, ts_sec);
        if locked {
            loop_bw = 3.0;
        } else {
            loop_bw = 4.0;
        }

        let (alpha, beta) = self.calc_loop_filter_params(loop_bw);

        self.param.carrier_phase_shift += error * alpha * 1.0; // XXX
        if self.param.carrier_phase_shift < 0.0 {
            self.param.carrier_phase_shift += 2.0 * PI;
        }
        self.param.carrier_phase_shift = self.param.carrier_phase_shift % (2.0 * PI);
        self.param.doppler_hz += error * beta; // XXX

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

        let coherent_corr_peak = self.track_symbol(&doppler_shifted_samples);
        self.correlation_peak_rolling_buffer
            .push(coherent_corr_peak);
        self.track_carrier_phase(coherent_corr_peak, sample.ts_sec);
        if sample.ts_sec - self.last_plot_ts >= 2.0 {
            self.update_all_plots();
            self.last_plot_ts = sample.ts_sec;
        }
    }
}
