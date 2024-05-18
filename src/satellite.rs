use colored::Colorize;
use plotters::prelude::*;
use rustfft::num_complex::Complex64;
use rustfft::FftPlanner;

const PI: f64 = std::f64::consts::PI;

use crate::constants::CN0_THRESHOLD_LOCKED;
use crate::constants::CN0_THRESHOLD_LOST;
use crate::constants::L1CA_HZ;
use crate::constants::PRN_CODE_LEN;
use crate::gold_code::GoldCode;
use crate::plots::plot_iq_scatter;
use crate::plots::plot_time_graph;
use crate::types::GnssCorrelationParam;
use crate::util::calc_correlation;
use crate::util::doppler_shift;

const SP_CORR: f64 = 0.5;
const T_IDLE: f64 = 1.0;
const T_ACQ: f64 = 0.01; // 10msec acquisition time
const T_FPULLIN: f64 = 1.0;
const T_DLL: f64 = 0.01; // non-coherent integration time for DLL
const T_CN0: f64 = 1.0; // averaging time for C/N0
const B_FLL_WIDE: f64 = 10.0; // bandwidth of FLL wide Hz
const B_FLL_NARROW: f64 = 2.0; // bandwidth of FLL narrow Hz
const B_PLL: f64 = 10.0; // bandwidth of PLL filter Hz : 0.005 gives good results initially
const B_DLL: f64 = 0.5; // bandwidth of DLL filter Hz

const DOPPLER_SPREAD_HZ: f64 = 8000.0;
const DOPPLER_SPREAD_BINS: usize = 160 * 2;

#[derive(PartialEq)]
enum TrackState {
    TRACKING,
    ACQUISITION,
    IDLE,
}

pub struct GnssSatellite {
    // constants
    prn: usize,
    fc: f64,                 // carrier frequency
    fs: f64,                 // sampling frequency
    fi: f64,                 // intermediate frequency
    samples_per_code: usize, // e.g. 2046 for L1CA
    code_sec: f64,           // PRN duration in sec
    prn_code: Vec<Complex64>,
    prn_code_fft: Vec<Complex64>,
    fft_planner: FftPlanner<f64>,
    state: TrackState,
    ts_sec: f64, // current time

    // idle
    num_idle_samples: usize,

    // searching
    sum_p: Vec<Vec<f64>>,
    num_acq_samples: usize,

    // tracking
    doppler_hz: f64,
    code_off_sec: f64,
    cn0: f64,
    adr: f64,
    phi: f64,
    err_phase: f64,
    sum_corr_e: f64,
    sum_corr_l: f64,
    sum_corr_p: f64,
    sum_corr_n: f64,
    num_tracking_samples: usize,

    // plots
    last_log_ts: f64,
    last_plot_ts: f64,
    code_phase_offset_rolling_buffer: Vec<f64>,
    phi_error_rolling_buffer: Vec<f64>,
    doppler_hz_rolling_buffer: Vec<f64>,
    disc_p_rolling_buffer: Vec<Complex64>,
}

impl Drop for GnssSatellite {
    fn drop(&mut self) {
        self.update_all_plots(true);
    }
}

impl GnssSatellite {
    pub fn new(
        prn: usize,
        gold_code: &mut GoldCode,
        fs: f64,
        fi: f64,
        param: GnssCorrelationParam,
    ) -> Self {
        log::warn!(
            "{}",
            format!(
                "sat {prn:2}: new: cn0={:.1} dopp={:5} code_off={:4} phi={:.2}",
                param.cn0, param.doppler_hz, param.code_phase_offset, param.carrier_phase_shift
            )
            .green()
        );

        Self {
            prn,
            fft_planner: FftPlanner::new(),
            prn_code: gold_code.get_prn_code_upsampled_complex(prn),
            prn_code_fft: gold_code.get_prn_code_fft(prn),
            ts_sec: 0.0,

            fc: L1CA_HZ,
            fs,
            fi,
            code_sec: 0.001,
            samples_per_code: 2046,

            sum_p: vec![vec![0.0; 2046]; DOPPLER_SPREAD_BINS],
            num_acq_samples: 0,
            num_idle_samples: 0,
            num_tracking_samples: 0,
            cn0: param.cn0,
            doppler_hz: param.doppler_hz,
            phi: param.carrier_phase_shift,
            code_off_sec: param.code_phase_offset as f64 / PRN_CODE_LEN as f64 * 0.001 / 2.0, // XXX
            adr: 0.0,
            err_phase: 0.0,
            sum_corr_e: 0.0,
            sum_corr_l: 0.0,
            sum_corr_p: 0.0,
            sum_corr_n: 0.0,

            code_phase_offset_rolling_buffer: vec![],
            phi_error_rolling_buffer: vec![],
            doppler_hz_rolling_buffer: vec![],
            disc_p_rolling_buffer: vec![],
            last_plot_ts: 0.0,
            last_log_ts: 0.0,
            state: TrackState::TRACKING,
        }
    }

    fn idle_start(&mut self) {
        let state_str = if self.state == TrackState::ACQUISITION {
            format!("IDLE").yellow()
        } else {
            format!("LOST").red()
        };
        log::warn!(
            "sat-{}: {state_str} cn0={:.1} ts_sec={:.3}",
            self.prn,
            self.cn0,
            self.ts_sec,
        );
        self.state = TrackState::IDLE;
        self.num_idle_samples = 0;
    }

    fn idle_process(&mut self) {
        self.num_idle_samples += 1;
        if self.num_idle_samples as f64 * self.code_sec > T_IDLE {
            self.acquisition_start();
        }
    }

    fn acquisition_init(&mut self) {
        self.sum_p = vec![vec![0.0; self.samples_per_code]; DOPPLER_SPREAD_BINS];
        self.num_acq_samples = 0;
    }
    fn acquisition_start(&mut self) {
        self.acquisition_init();
        self.state = TrackState::ACQUISITION;
    }

    fn tracking_init(&mut self) {
        self.doppler_hz = 0.0;
        self.cn0 = 0.0;
        self.adr = 0.0;
        self.code_off_sec = 0.0;
        self.err_phase = 0.0;
        self.sum_corr_p = 0.0;
        self.sum_corr_e = 0.0;
        self.sum_corr_l = 0.0;
        self.sum_corr_n = 0.0;
        self.num_tracking_samples = 0;
    }

    fn tracking_start(
        &mut self,
        doppler_hz: f64,
        cn0: f64,
        code_off_sec: f64,
        code_offset_idx: usize,
    ) {
        log::warn!(
            "sat-{}: {} cn0={cn0:.1} dopp={doppler_hz:5.0} code_off={code_offset_idx} ts_sec={:.3}",
            self.prn,
            format!("LOCK").green(),
            self.ts_sec,
        );
        self.tracking_init();
        self.state = TrackState::TRACKING;

        self.code_off_sec = code_off_sec;
        self.doppler_hz = doppler_hz;
        self.cn0 = cn0;
    }

    fn integrate_correlation(&mut self, iq_vec_slice: &[Complex64], doppler_hz: f64) -> Vec<f64> {
        let mut iq_vec = iq_vec_slice.to_vec();

        assert_eq!(iq_vec.len(), self.prn_code_fft.len());

        doppler_shift(&mut iq_vec, self.fi + doppler_hz, 0.0, 0.0, self.fs);

        let corr = calc_correlation(&mut self.fft_planner, &iq_vec, &self.prn_code_fft);
        let corr_vec: Vec<_> = corr.iter().map(|v| v.norm_sqr()).collect();

        corr_vec
    }

    pub fn update_param(&mut self, param: &GnssCorrelationParam) {
        log::warn!(
            "sat {:2}: ---- cn0={:.1} dopp={:5} code_off={:5} phi={:5.2}",
            self.prn,
            param.cn0,
            param.doppler_hz,
            param.code_phase_offset,
            param.carrier_phase_shift,
        );
    }

    fn update_all_plots(&mut self, force: bool) {
        if !force && self.ts_sec - self.last_plot_ts <= 2.0 {
            return;
        }

        self.plot_iq_scatter();
        self.plot_code_phase_offset();
        self.plot_phi_error();
        self.plot_doppler_hz();

        self.last_plot_ts = self.ts_sec;
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

    fn plot_phi_error(&self) {
        plot_time_graph(
            self.prn,
            "phi-error",
            self.phi_error_rolling_buffer.as_slice(),
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

    fn plot_iq_scatter(&self) {
        let len = self.disc_p_rolling_buffer.len();
        let n = usize::min(len, 2000);
        plot_iq_scatter(self.prn, &&self.disc_p_rolling_buffer[len - n..len]);
    }

    fn acquisition_process(&mut self, iq_vec2: &Vec<Complex64>) {
        // only take the last minute worth of data
        let iq_vec = &iq_vec2[self.samples_per_code..];
        let step_hz = 2.0 * DOPPLER_SPREAD_HZ / DOPPLER_SPREAD_BINS as f64;

        for i in 0..DOPPLER_SPREAD_BINS {
            let doppler_hz = -DOPPLER_SPREAD_HZ + i as f64 * step_hz;
            let c_non_coherent = self.integrate_correlation(iq_vec, doppler_hz);
            assert_eq!(c_non_coherent.len(), self.samples_per_code);

            for j in 0..self.samples_per_code {
                self.sum_p[i][j] += c_non_coherent[j]; // norm_sqr()
            }
        }

        self.num_acq_samples += 1;
        assert!(self.num_acq_samples <= 10);

        if self.num_acq_samples as f64 * self.code_sec >= T_ACQ {
            let mut idx = 0;
            let mut idx_peak = 0;
            let mut p_max = 0.0;
            let mut p_peak = 0.0;
            let mut p_total = 0.0;

            for i in 0..DOPPLER_SPREAD_BINS {
                let mut p_sum = 0.0;
                let mut v_peak = 0.0;
                let mut j_peak = 0;
                for j in 0..self.samples_per_code {
                    p_sum += self.sum_p[i][j];
                    if self.sum_p[i][j] > v_peak {
                        v_peak = self.sum_p[i][j];
                        j_peak = j;
                    }
                }
                if p_sum > p_max {
                    idx = i;
                    p_max = p_sum;
                    p_peak = v_peak;
                    idx_peak = j_peak;
                }
                p_total += p_sum;
            }

            let doppler_hz = -DOPPLER_SPREAD_HZ + idx as f64 * step_hz;
            let code_off_sec = idx_peak as f64 / self.samples_per_code as f64 * self.code_sec;
            let p_avg = p_total / self.sum_p[idx].len() as f64 / DOPPLER_SPREAD_BINS as f64;
            let cn0 = 10.0 * ((p_peak - p_avg) / p_avg / self.code_sec).log10();

            if cn0 >= CN0_THRESHOLD_LOCKED {
                self.tracking_start(doppler_hz, cn0, code_off_sec, idx_peak);
            } else {
                self.idle_start();
            }
            self.acquisition_init();
        }
    }

    fn compute_correlation(
        &mut self,
        iq_vec2: &Vec<Complex64>,
        code_idx: i32,
    ) -> (Complex64, Complex64, Complex64, Complex64) {
        let n = self.samples_per_code as i32;
        assert!(-n < code_idx && code_idx < n);
        let lo = if code_idx > 0 { code_idx } else { n + code_idx };
        assert!(lo > 0);
        let lo_u = lo as usize;
        let hi_u = (lo + n) as usize;

        let mut signal = iq_vec2[lo_u..hi_u].to_vec();

        doppler_shift(&mut signal, self.doppler_hz, 0.0, self.phi, self.fs);

        // pos = int(sp_corr * T / len(code) * fs) + 1
        let pos = (SP_CORR * self.code_sec * self.fs / PRN_CODE_LEN as f64) as usize;
        assert_eq!(pos, 1);

        let mut corr_prompt = Complex64::default();
        let mut corr_early = Complex64::default();
        let mut corr_late = Complex64::default();
        let mut corr_neutral = Complex64::default();

        // PROMPT
        for j in 0..signal.len() {
            corr_prompt += signal[j] * self.prn_code[j];
        }
        corr_prompt /= signal.len() as f64;

        // EARLY:
        for j in 0..signal.len() - pos {
            corr_early += signal[j] * self.prn_code[pos + j];
        }
        corr_early /= (signal.len() - pos) as f64;

        // LATE:
        for j in 0..signal.len() - pos {
            corr_late += signal[pos + j] * self.prn_code[j];
        }
        corr_late /= (signal.len() - pos) as f64;

        // NEUTRAL:
        let pos_neutral: usize = 80;
        for j in 0..signal.len() - pos_neutral {
            corr_neutral += signal[j] * self.prn_code[pos_neutral + j];
        }
        corr_neutral /= (signal.len() - pos_neutral) as f64;

        (corr_prompt, corr_early, corr_late, corr_neutral)
    }

    fn run_fll(&mut self, c_e: Complex64, c_l: Complex64) {
        if self.num_tracking_samples < 2 {
            return;
        }

        let dot = c_e.re * c_l.re + c_e.im * c_l.im;
        let cross = c_e.re * c_l.im - c_e.im * c_l.re;

        if dot == 0.0 {
            return;
        }

        let b = if self.num_tracking_samples as f64 * self.code_sec < T_FPULLIN / 2.0 {
            B_FLL_WIDE // 10.0
        } else {
            B_FLL_NARROW // 2.-
        };
        let err_freq = (cross / dot).atan();
        // ~= -40 * err_freq / 2 / PI ~= -6.36 * err_freq
        // ~=  -8 * err_freq / 2 / PI ~= -1.27 * err_freq

        self.doppler_hz -= b / 0.25 * err_freq / 2.0 / PI;
    }

    fn run_pll(&mut self, c_p: Complex64) {
        if c_p.re == 0.0 {
            return;
        }
        let err_phase = (c_p.im / c_p.re).atan() / 2.0 / PI;
        let w = B_PLL / 0.53; // ~18.9
        assert!(self.code_sec == 0.001);
        // ~= 26 * (err_phase - err_phase_old) + 356 * err_phase * 0.001
        self.doppler_hz +=
            1.4 * w * (err_phase - self.err_phase) + w * w * err_phase * self.code_sec;
        self.err_phase = err_phase;
        self.phi_error_rolling_buffer.push(err_phase * 2.0 * PI);
    }

    fn run_dll(&mut self, c_e: Complex64, c_l: Complex64) {
        let n = usize::max(1, (T_DLL / self.code_sec) as usize);
        assert_eq!(n, 10);
        self.sum_corr_e += c_e.norm();
        self.sum_corr_l += c_l.norm();
        if self.num_tracking_samples % n == 0 {
            let e = self.sum_corr_e;
            let l = self.sum_corr_l;
            let err_code = (e - l) / (e + l) / 2.0 * self.code_sec / PRN_CODE_LEN as f64;
            self.code_off_sec -= B_DLL / 0.25 * err_code * self.code_sec * n as f64;
            self.sum_corr_e = 0.0;
            self.sum_corr_l = 0.0;
        }
    }

    fn update_cn0(&mut self, c_p: Complex64, c_n: Complex64) {
        self.sum_corr_p += c_p.norm_sqr();
        self.sum_corr_n += c_n.norm_sqr();

        if self.num_tracking_samples % (T_CN0 / self.code_sec) as usize == 0 {
            if self.sum_corr_n > 0.0 {
                let cn0 = 10.0 * (self.sum_corr_p / self.sum_corr_n / self.code_sec).log10();
                self.cn0 += 0.5 * (cn0 - self.cn0);
            }
            self.sum_corr_n = 0.0;
            self.sum_corr_p = 0.0;
        }
    }
    fn get_code_and_carrier_phase(&mut self) -> i32 {
        assert_eq!(self.code_sec, 0.001);

        let tau = self.code_sec;
        //let fc = self.fi + self.doppler_hz;
        self.adr += self.doppler_hz * tau; // accumulated Doppler
        self.code_off_sec -= self.doppler_hz / self.fc * tau; // carrier-aided code offset

        // code offset in samples
        let code_off = (self.code_off_sec * self.fs + 0.5) % self.samples_per_code as f64;
        let mut code_idx = code_off as i32;
        if code_idx < 0 {
            code_idx += self.samples_per_code as i32;
        }

        self.phi = self.fi * tau + self.adr + self.fc * code_idx as f64 / self.fs;
        //    -2.0 * PI * self.doppler_hz * (self.samples_per_code as f64 / self.fs + self.ts_sec)
        //        + self.phi;

        code_idx
    }

    fn log_periodically(&mut self, code_idx: i32) {
        if self.ts_sec - self.last_log_ts > 3.0 {
            log::warn!(
                "sat-{:2}: {} cn0={:.1} dopp={:5.0} code_idx={:4} phi={:5.2} ts_sec={:.3}",
                self.prn,
                format!("TRCK").green(),
                self.cn0,
                self.doppler_hz,
                code_idx,
                (self.phi % 1.0) * 2.0 * PI,
                self.ts_sec,
            );
            self.last_log_ts = self.ts_sec;
        }
    }
    fn tracking_process(&mut self, iq_vec2: &Vec<Complex64>) {
        assert_eq!(self.samples_per_code, 2046);
        assert_eq!(self.code_sec, 0.001);
        assert_eq!(self.fs, 2046000.0);
        assert_eq!(self.fc, 1575420000.0);

        let code_idx = self.get_code_and_carrier_phase();
        let (c_p, c_e, c_l, c_n) = self.compute_correlation(&iq_vec2, code_idx);

        if self.num_tracking_samples as f64 * self.code_sec < T_FPULLIN {
            self.run_fll(c_e, c_l);
        } else {
            self.run_pll(c_p);
        }
        self.run_dll(c_e, c_l);
        self.update_cn0(c_p, c_n);

        self.code_phase_offset_rolling_buffer.push(code_idx as f64);
        self.disc_p_rolling_buffer.push(c_p * 1000.0);
        self.doppler_hz_rolling_buffer.push(self.doppler_hz);
        self.update_all_plots(false);
        self.num_tracking_samples += 1;

        self.log_periodically(code_idx);

        if self.cn0 < CN0_THRESHOLD_LOST {
            self.idle_start();
        }
    }

    pub fn process_samples(&mut self, iq_vec2: &Vec<Complex64>, ts_sec: f64) {
        self.ts_sec = ts_sec;

        log::info!(
            "sat-{}: processing: ts_sec={:.4}: cn0={:.1} dopp={:.0} code_off_sec={:.6}",
            self.prn,
            self.ts_sec,
            self.cn0,
            self.doppler_hz,
            self.code_off_sec,
        );

        match self.state {
            TrackState::ACQUISITION => self.acquisition_process(&iq_vec2),
            TrackState::TRACKING => self.tracking_process(&iq_vec2),
            TrackState::IDLE => self.idle_process(),
        }
    }
}
