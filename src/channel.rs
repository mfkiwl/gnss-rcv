use colored::Colorize;
use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;
use plotters::prelude::*;
use rustfft::num_complex::Complex64;
use rustfft::FftPlanner;

const PI: f64 = std::f64::consts::PI;

use crate::code::Code;
use crate::constants::CN0_THRESHOLD_LOCKED;
use crate::constants::CN0_THRESHOLD_LOST;
use crate::constants::L1CA_HZ;
use crate::constants::PRN_CODE_LEN;
use crate::navigation::Navigation;
use crate::plots::plot_iq_scatter;
use crate::plots::plot_time_graph;
use crate::util::calc_correlation;
use crate::util::doppler_shift;

const SP_CORR: f64 = 0.5;
const T_IDLE: f64 = 1.0;
const T_ACQ: f64 = 0.01; // 10msec acquisition time
const T_FPULLIN: f64 = 1.0;
const T_NPULLIN: f64 = 1.5; // navigation data pullin time (s)
const T_DLL: f64 = 0.01; // non-coherent integration time for DLL
const T_CN0: f64 = 1.0; // averaging time for C/N0
const B_FLL_WIDE: f64 = 10.0; // bandwidth of FLL wide Hz
const B_FLL_NARROW: f64 = 2.0; // bandwidth of FLL narrow Hz
const B_PLL: f64 = 10.0; // bandwidth of PLL filter Hz
const B_DLL: f64 = 0.5; // bandwidth of DLL filter Hz

const DOPPLER_SPREAD_HZ: f64 = 8000.0;
const DOPPLER_SPREAD_BINS: usize = 160 * 2;
const HISTORY_NUM: usize = 20000;

#[derive(PartialEq)]
enum TrackState {
    TRACKING,
    ACQUISITION,
    IDLE,
}

#[derive(Default)]
struct Tracking {
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
    sum_p: Vec<Vec<f64>>,
}

#[derive(Default)]
pub struct History {
    pub last_log_ts: f64,
    pub last_plot_ts: f64,
    pub code_phase_offset_hist: Vec<f64>,
    pub phi_error_hist: Vec<f64>,
    pub doppler_hz_hist: Vec<f64>,
    pub corr_p_hist: Vec<Complex64>,
}

pub struct Channel {
    pub sv: SV,
    fc: f64,                 // carrier frequency
    fs: f64,                 // sampling frequency
    fi: f64,                 // intermediate frequency
    samples_per_code: usize, // e.g. 2046 for L1CA
    code_sec: f64,           // PRN duration in sec
    prn_code: Vec<Complex64>,
    prn_code_fft: Vec<Complex64>,
    fft_planner: FftPlanner<f64>,
    state: TrackState,
    pub ts_sec: f64, // current time

    num_idle_samples: usize,
    pub num_tracking_samples: usize,
    num_acq_samples: usize,

    pub hist: History,
    pub nav: Navigation,
    trk: Tracking,
}

impl Drop for Channel {
    fn drop(&mut self) {
        self.update_all_plots(true);
    }
}

impl Channel {
    pub fn new(constellation: Constellation, prn: u8, code: &mut Code, fs: f64, fi: f64) -> Self {
        Self {
            sv: SV::new(constellation, prn),
            fft_planner: FftPlanner::new(),
            prn_code: code.get_prn_code_upsampled_complex(prn),
            prn_code_fft: code.get_prn_code_fft(prn),
            ts_sec: 0.0,

            fc: L1CA_HZ,
            fs,
            fi,
            code_sec: 0.001,
            samples_per_code: 2046,

            num_acq_samples: 0,
            num_idle_samples: 0,
            num_tracking_samples: 0,

            state: TrackState::ACQUISITION,

            nav: Navigation::new(),
            hist: History::default(),
            trk: Tracking {
                sum_p: vec![vec![0.0; 2046]; DOPPLER_SPREAD_BINS],
                ..Default::default()
            },
        }
    }

    fn idle_start(&mut self) {
        if self.state == TrackState::TRACKING {
            log::warn!(
                "{}: {} cn0={:.1} ts_sec={:.3}",
                self.sv,
                format!("LOST").red(),
                self.trk.cn0,
                self.ts_sec,
            );
        } else {
            log::info!(
                "{}: IDLE cn0={:.1} ts_sec={:.3}",
                self.sv,
                self.trk.cn0,
                self.ts_sec,
            );
        }
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
        self.trk.sum_p = vec![vec![0.0; self.samples_per_code]; DOPPLER_SPREAD_BINS];
        self.num_acq_samples = 0;
    }
    fn acquisition_start(&mut self) {
        self.acquisition_init();
        self.state = TrackState::ACQUISITION;
    }

    fn tracking_init(&mut self) {
        self.trk.doppler_hz = 0.0;
        self.trk.cn0 = 0.0;
        self.trk.adr = 0.0;
        self.trk.code_off_sec = 0.0;
        self.trk.err_phase = 0.0;
        self.trk.sum_corr_p = 0.0;
        self.trk.sum_corr_e = 0.0;
        self.trk.sum_corr_l = 0.0;
        self.trk.sum_corr_n = 0.0;
        self.num_tracking_samples = 0;
        self.nav.init();
    }

    fn tracking_start(
        &mut self,
        doppler_hz: f64,
        cn0: f64,
        code_off_sec: f64,
        code_offset_idx: usize,
    ) {
        log::warn!(
            "{}: {} cn0={cn0:.1} dopp={doppler_hz:5.0} code_off={code_offset_idx} ts_sec={:.3}",
            self.sv,
            format!("LOCK").green(),
            self.ts_sec,
        );
        self.tracking_init();
        self.state = TrackState::TRACKING;

        self.trk.code_off_sec = code_off_sec;
        self.trk.doppler_hz = doppler_hz;
        self.trk.cn0 = cn0;
    }

    fn integrate_correlation(&mut self, iq_vec_slice: &[Complex64], doppler_hz: f64) -> Vec<f64> {
        let mut iq_vec = iq_vec_slice.to_vec();

        assert_eq!(iq_vec.len(), self.prn_code_fft.len());

        doppler_shift(&mut iq_vec, self.fi + doppler_hz, 0.0, self.fs);

        let corr = calc_correlation(&mut self.fft_planner, &iq_vec, &self.prn_code_fft);
        let corr_vec: Vec<_> = corr.iter().map(|v| v.norm_sqr()).collect();

        corr_vec
    }

    fn update_all_plots(&mut self, force: bool) {
        if !force && self.ts_sec - self.hist.last_plot_ts <= 2.0 {
            return;
        }

        self.plot_iq_scatter();
        self.plot_code_phase_offset();
        self.plot_phi_error();
        self.plot_doppler_hz();

        self.hist.last_plot_ts = self.ts_sec;
    }

    fn plot_code_phase_offset(&self) {
        plot_time_graph(
            self.sv,
            "code-phase-offset",
            self.hist.code_phase_offset_hist.as_slice(),
            50.0,
            &BLUE,
        );
    }

    fn plot_phi_error(&self) {
        plot_time_graph(
            self.sv,
            "phi-error",
            self.hist.phi_error_hist.as_slice(),
            0.5,
            &BLACK,
        );
    }

    fn plot_doppler_hz(&self) {
        plot_time_graph(
            self.sv,
            "doppler-hz",
            &self.hist.doppler_hz_hist.as_slice(),
            10.0,
            &BLACK,
        );
    }

    fn plot_iq_scatter(&self) {
        let len = self.hist.corr_p_hist.len();
        let n = usize::min(len, 2000);
        plot_iq_scatter(self.sv, &&self.hist.corr_p_hist[len - n..len]);
    }

    fn acquisition_process(&mut self, iq_vec2: &Vec<Complex64>) {
        // only take the last minute worth of data
        let iq_vec_slice = &iq_vec2[self.samples_per_code..];
        let step_hz = 2.0 * DOPPLER_SPREAD_HZ / DOPPLER_SPREAD_BINS as f64;

        for i in 0..DOPPLER_SPREAD_BINS {
            let doppler_hz = -DOPPLER_SPREAD_HZ + i as f64 * step_hz;
            let c_non_coherent = self.integrate_correlation(iq_vec_slice, doppler_hz);
            assert_eq!(c_non_coherent.len(), self.samples_per_code);

            for j in 0..self.samples_per_code {
                self.trk.sum_p[i][j] += c_non_coherent[j];
            }
        }

        self.num_acq_samples += 1;

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
                    p_sum += self.trk.sum_p[i][j];
                    if self.trk.sum_p[i][j] > v_peak {
                        v_peak = self.trk.sum_p[i][j];
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
            let p_avg = p_total / self.trk.sum_p[idx].len() as f64 / DOPPLER_SPREAD_BINS as f64;
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
    ) -> (Complex64, Complex64, Complex64, Complex64) {
        let n = self.samples_per_code as i32;
        let code_idx = *self.hist.code_phase_offset_hist.last().unwrap() as i32;
        assert!(-n < code_idx && code_idx < n);

        //       [-------][-------][---------]
        // t=n   [^(.......)      ]                code_idx=0
        // t=n+1          [       ^(.......) ]     code_idx=-1

        let lo = if code_idx >= 0 {
            code_idx
        } else {
            n + code_idx
        };
        assert!(lo >= 0);
        let lo_u = lo as usize;
        let hi_u = (lo + n) as usize;

        let mut signal = iq_vec2[lo_u..hi_u].to_vec();

        doppler_shift(&mut signal, self.trk.doppler_hz, self.trk.phi, self.fs);

        let pos = (SP_CORR * self.code_sec * self.fs / PRN_CODE_LEN as f64) as usize;

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

    fn run_fll(&mut self) {
        if self.num_tracking_samples < 2 {
            return;
        }
        let len = self.hist.corr_p_hist.len();
        let c1 = self.hist.corr_p_hist[len - 1];
        let c2 = self.hist.corr_p_hist[len - 2];
        let dot = c1.re * c2.re + c1.im * c2.im;
        let cross = c1.re * c2.im - c1.im * c2.re;

        if dot == 0.0 {
            return;
        }

        let b = if self.num_tracking_samples as f64 * self.code_sec < T_FPULLIN / 2.0 {
            B_FLL_WIDE // 10.0
        } else {
            B_FLL_NARROW // 2.-
        };
        let err_freq = (cross / dot).atan() / 2.0 / PI;

        self.trk.doppler_hz -= b / 0.25 * err_freq;
    }

    fn run_pll(&mut self, c_p: Complex64) {
        if c_p.re == 0.0 {
            return;
        }
        let err_phase = (c_p.im / c_p.re).atan() / 2.0 / PI;
        let w = B_PLL / 0.53; // ~18.9
        self.trk.doppler_hz +=
            1.4 * w * (err_phase - self.trk.err_phase) + w * w * err_phase * self.code_sec;
        self.trk.err_phase = err_phase;
        self.hist.phi_error_hist.push(err_phase * 2.0 * PI);
    }

    fn run_dll(&mut self, c_e: Complex64, c_l: Complex64) {
        let n = usize::max(1, (T_DLL / self.code_sec) as usize);
        assert_eq!(n, 10);
        self.trk.sum_corr_e += c_e.norm();
        self.trk.sum_corr_l += c_l.norm();
        if self.num_tracking_samples % n == 0 {
            let e = self.trk.sum_corr_e;
            let l = self.trk.sum_corr_l;
            let err_code = (e - l) / (e + l) / 2.0 * self.code_sec / PRN_CODE_LEN as f64;
            self.trk.code_off_sec -= B_DLL / 0.25 * err_code * self.code_sec * n as f64;
            self.trk.sum_corr_e = 0.0;
            self.trk.sum_corr_l = 0.0;
        }
    }

    fn update_cn0(&mut self, c_p: Complex64, c_n: Complex64) {
        self.trk.sum_corr_p += c_p.norm_sqr();
        self.trk.sum_corr_n += c_n.norm_sqr();

        if self.num_tracking_samples % (T_CN0 / self.code_sec) as usize == 0 {
            if self.trk.sum_corr_n > 0.0 {
                let cn0 =
                    10.0 * (self.trk.sum_corr_p / self.trk.sum_corr_n / self.code_sec).log10();
                self.trk.cn0 += 0.5 * (cn0 - self.trk.cn0);
            }
            self.trk.sum_corr_n = 0.0;
            self.trk.sum_corr_p = 0.0;
        }
    }
    fn get_code_and_carrier_phase(&mut self) -> i32 {
        let tau = self.code_sec;
        let fc = self.fi + self.trk.doppler_hz;
        self.trk.adr += self.trk.doppler_hz * tau; // accumulated Doppler
        self.trk.code_off_sec -= self.trk.doppler_hz / self.fc * tau; // carrier-aided code offset

        // code offset in samples
        let code_off = (self.trk.code_off_sec * self.fs + 0.5) % self.samples_per_code as f64;
        let mut code_idx = code_off as i32;
        if code_idx < 0 {
            code_idx += self.samples_per_code as i32;
        }

        self.trk.phi = self.fi * tau + self.trk.adr + fc * code_idx as f64 / self.fs;

        self.hist.code_phase_offset_hist.push(code_idx as f64);

        code_idx
    }

    fn log_periodically(&mut self) {
        let code_idx = self.hist.code_phase_offset_hist.last().unwrap();
        if self.ts_sec - self.hist.last_log_ts > 3.0 {
            log::warn!(
                "{}: {} cn0={:.1} dopp={:5.0} code_idx={:4.0} phi={:5.2} ts_sec={:.3}",
                self.sv,
                format!("TRCK").green(),
                self.trk.cn0,
                self.trk.doppler_hz,
                code_idx,
                (self.trk.phi % 1.0) * 2.0 * PI,
                self.ts_sec,
            );
            self.hist.last_log_ts = self.ts_sec;
        }
    }

    fn hist_trim(&mut self) {
        if self.hist.doppler_hz_hist.len() > HISTORY_NUM {
            self.hist.doppler_hz_hist.rotate_left(1);
            self.hist.doppler_hz_hist.pop();
        }
        if self.hist.phi_error_hist.len() > HISTORY_NUM {
            self.hist.phi_error_hist.rotate_left(1);
            self.hist.phi_error_hist.pop();
        }
        if self.hist.corr_p_hist.len() > HISTORY_NUM {
            self.hist.corr_p_hist.rotate_left(1);
            self.hist.corr_p_hist.pop();
        }
        if self.hist.code_phase_offset_hist.len() > HISTORY_NUM {
            self.hist.code_phase_offset_hist.rotate_left(1);
            self.hist.code_phase_offset_hist.pop();
        }
    }

    fn tracking_process(&mut self, iq_vec2: &Vec<Complex64>) {
        self.get_code_and_carrier_phase();
        let (c_p, c_e, c_l, c_n) = self.compute_correlation(&iq_vec2);
        self.hist.corr_p_hist.push(c_p);

        if self.num_tracking_samples as f64 * self.code_sec < T_FPULLIN {
            self.run_fll();
        } else {
            self.run_pll(c_p);
        }

        self.run_dll(c_e, c_l);
        self.update_cn0(c_p, c_n);

        if self.num_tracking_samples as f64 * self.code_sec >= T_NPULLIN {
            self.nav_decode();
        }

        self.hist.doppler_hz_hist.push(self.trk.doppler_hz);
        self.hist_trim();
        self.update_all_plots(false);
        self.num_tracking_samples += 1;
        self.log_periodically();

        if self.trk.cn0 < CN0_THRESHOLD_LOST {
            self.idle_start();
        }
    }

    pub fn process_samples(&mut self, iq_vec2: &Vec<Complex64>, ts_sec: f64) {
        self.ts_sec = ts_sec;

        log::info!(
            "{}: processing: ts_sec={:.4}: cn0={:.1} dopp={:.0} code_off_sec={:.6}",
            self.sv,
            self.ts_sec,
            self.trk.cn0,
            self.trk.doppler_hz,
            self.trk.code_off_sec,
        );

        match self.state {
            TrackState::ACQUISITION => self.acquisition_process(&iq_vec2),
            TrackState::TRACKING => self.tracking_process(&iq_vec2),
            TrackState::IDLE => self.idle_process(),
        }
    }
}
