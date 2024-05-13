use colored::Colorize;
use plotters::prelude::*;
use rustfft::num_complex::Complex64;

const PI: f64 = std::f64::consts::PI;

use crate::constants::L1CA_HZ;
use crate::constants::PRN_CODE_LEN;
use crate::plots::plot_iq_scatter;
use crate::plots::plot_time_graph;
use crate::types::GnssCorrelationParam;
use crate::types::IQSample;
use crate::util::doppler_shift;

const SP_CORR: f64 = 0.5;
const T_FPULLIN: f64 = 1.0;
const T_DLL: f64 = 0.01; // non-coherent integration time for DLL (s)
const T_CN0: f64 = 1.0; // averaging time for C/N0 (s)
const B_FLL_WIDE: f64 = 10.0; // bw in Hz
const B_FLL_NARROW: f64 = 2.0; // bw in Hz
const B_PLL: f64 = 10.0; // bandwidth of PLL filter (Hz)
const B_DLL: f64 = 0.5; // band-width of DLL filter (Hz)

#[derive(PartialEq)]
enum TrackState {
    LOCKED,
    SEARCHING,
}

pub struct GnssSatellite {
    prn: usize,
    samples_per_code: usize,

    fc: f64,       // carrier frequency
    fs: f64,       // sampling frequency
    code_sec: f64, // PRN duration in sec
    prn_code: Vec<Complex64>,
    doppler_hz: f64,
    code_offset_sec: f64,
    phi: f64,
    cn0: f64,
    adr: f64,
    state: TrackState,
    err_phase: f64,
    sum_corr_e: f64,
    sum_corr_l: f64,
    sum_corr_p: f64,
    sum_corr_n: f64,

    last_plot_ts: f64,
    num_locked_samples: usize,

    code_phase_offset_rolling_buffer: Vec<f64>,
    carrier_phase_shift_rolling_buffer: Vec<f64>,
    doppler_hz_rolling_buffer: Vec<f64>,
    iq_scatter_rolling_buffer: Vec<Complex64>,
}

impl Drop for GnssSatellite {
    fn drop(&mut self) {
        self.update_all_plots(true, self.last_plot_ts);
    }
}

impl GnssSatellite {
    pub fn new(prn: usize, prn_code: Vec<Complex64>, param: GnssCorrelationParam) -> Self {
        log::warn!(
            "{}",
            format!(
                "sat {}: new: doppler={} phase_shift={} cn0={:.2} phi={:.2}",
                prn,
                param.doppler_hz,
                param.code_phase_offset,
                param.cn0,
                param.carrier_phase_shift
            )
            .green()
        );
        Self {
            prn,
            prn_code,

            fc: L1CA_HZ,
            fs: 2046.0 * 1000.0, // sampling frequency
            code_sec: 0.001,
            samples_per_code: 2046,
            num_locked_samples: 0,

            cn0: param.cn0,
            doppler_hz: param.doppler_hz,
            phi: param.carrier_phase_shift,
            code_offset_sec: param.code_phase_offset as f64 / PRN_CODE_LEN as f64 * 0.001 / 2.0, // XXX
            adr: 0.0,
            err_phase: 0.0,
            sum_corr_e: 0.0,
            sum_corr_l: 0.0,
            sum_corr_p: 0.0,
            sum_corr_n: 0.0,

            code_phase_offset_rolling_buffer: vec![],
            carrier_phase_shift_rolling_buffer: vec![],
            doppler_hz_rolling_buffer: vec![],
            iq_scatter_rolling_buffer: vec![],

            last_plot_ts: 0.0,
            state: TrackState::LOCKED,
        }
    }

    pub fn update_param(&mut self, param: &GnssCorrelationParam) {
        log::warn!(
            "sat {}: exists:  doppler_hz={} code_phase_shift={} cn0={:.2}",
            self.prn,
            param.doppler_hz,
            param.code_phase_offset,
            param.cn0
        );
    }

    fn update_all_plots(&mut self, force: bool, ts_sec: f64) {
        if !force && ts_sec - self.last_plot_ts <= 2.0 {
            return;
        }

        self.plot_iq_scatter();
        self.plot_code_phase_offset();
        self.plot_carrier_phase_shift();
        self.plot_doppler_hz();

        self.last_plot_ts = ts_sec;
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

    fn plot_iq_scatter(&self) {
        let len = self.iq_scatter_rolling_buffer.len();
        let n = usize::min(len, 1000);
        plot_iq_scatter(self.prn, &self.iq_scatter_rolling_buffer[len - n..len]);
    }

    fn process_searching(&mut self, sample: &IQSample) {
        log::info!(
            "sat-{}: searching: ts_sec={:.4} sec: cn0={:.1} doppler={} code_off_sec={:.6} phi={:.2}",
            self.prn,
            sample.ts_sec,
            self.cn0,
            self.doppler_hz,
            self.code_offset_sec,
            self.phi,
        );
    }

    pub fn correlate_vec(&self, a: &Vec<Complex64>, b: &Vec<Complex64>) -> Complex64 {
        let mut sum = Complex64 { re: 0.0, im: 0.0 };
        for i in 0..a.len() {
            sum += a[i] * b[i].conj();
        }
        sum / a.len() as f64
    }

    fn compute_correlation(
        &mut self,
        sample: &IQSample,
        i: usize,
        phi: f64,
    ) -> (Complex64, Complex64, Complex64, Complex64) {
        let mut signal = sample.iq_vec[i..i + self.samples_per_code].to_vec();

        doppler_shift(&mut signal, self.doppler_hz, sample.ts_sec, phi, self.fs);

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
            corr_neutral += signal[j] * self.prn_code[pos + j];
        }
        corr_neutral /= (signal.len() - pos_neutral) as f64;

        self.iq_scatter_rolling_buffer.push(corr_prompt * 1000.0);

        (corr_prompt, corr_early, corr_late, corr_neutral)
    }

    fn run_fll(&mut self, c_e: Complex64, c_l: Complex64) {
        if self.num_locked_samples < 2 {
            return;
        }

        let dot = c_e.re * c_l.re + c_e.im * c_l.im;
        let cross = c_e.re * c_l.im - c_e.im * c_l.re;

        if dot == 0.0 {
            return;
        }

        let b = if self.num_locked_samples as f64 * self.code_sec < T_FPULLIN / 2.0 {
            B_FLL_WIDE
        } else {
            B_FLL_NARROW
        };
        let err_freq = (cross / dot).atan();
        self.doppler_hz -= b / 0.25 * err_freq / 2.0 / PI;
    }

    fn run_pll(&mut self, c_p: Complex64) {
        if c_p.re == 0.0 {
            return;
        }
        let err_phase = (c_p.im / c_p.re).atan() / 2.0 / PI;
        let w = B_PLL / 0.53;
        self.doppler_hz +=
            1.4 * w * (err_phase - self.err_phase) + w * w * err_phase * self.code_sec;
        self.err_phase = err_phase;
    }

    fn run_dll(&mut self, c_e: Complex64, c_l: Complex64) {
        let n = usize::max(1, (T_DLL / self.code_sec) as usize);
        self.sum_corr_e += c_e.norm();
        self.sum_corr_l += c_l.norm();
        if self.num_locked_samples % n == 0 {
            let e = self.sum_corr_e;
            let l = self.sum_corr_l;
            let err_code = (e - l) / (e + l) / 2.0 * self.code_sec / PRN_CODE_LEN as f64;
            self.code_offset_sec -= B_DLL / 0.25 * err_code * self.code_sec * n as f64;
            self.sum_corr_e = 0.0;
            self.sum_corr_l = 0.0;
        }
    }

    fn update_cn0(&mut self, c_p: Complex64, c_n: Complex64) {
        self.sum_corr_p += c_p.norm_sqr();
        self.sum_corr_n += c_n.norm_sqr();

        if self.num_locked_samples % (T_CN0 / self.code_sec) as usize == 0 {
            if self.sum_corr_n > 0.0 {
                let cn0 = 10.0 * (self.sum_corr_p / self.sum_corr_n / self.code_sec).log10();
                self.cn0 += 0.5 * (cn0 - self.cn0);
                self.sum_corr_n = 0.0;
                self.sum_corr_p = 0.0;
                //log::warn!("sat-{}: cn0={:.2}", self.prn, self.cn0);
            }
        }
    }

    fn process_locked(&mut self, sample: &IQSample) {
        assert_eq!(self.code_sec, 0.001);
        assert_eq!(self.fs, 2046000.0);
        assert_eq!(self.fc, 1575420000.0);
        assert_eq!(self.samples_per_code, 2046);

        let tau = self.code_sec;
        self.adr += self.doppler_hz * tau; // accumulated Doppler
        self.code_offset_sec -= self.doppler_hz / self.fc * tau; // carrier-aided code offset

        // code offset in samples
        let mut code_offset = (self.code_offset_sec * self.fs + 0.5) % self.samples_per_code as f64;
        let phi = (self.adr + self.doppler_hz * code_offset / self.fs) * 2.0 * PI;

        let mut i: i32 = code_offset as i32;
        if i < 0 {
            i += self.samples_per_code as i32;
        }
        assert!(i >= 0);
        let (c_p, c_e, c_l, c_n) = self.compute_correlation(sample, i as usize, phi);

        if self.num_locked_samples as f64 * self.code_sec < T_FPULLIN {
            self.run_fll(c_e, c_l);
        } else {
            self.run_pll(c_p);
        }
        self.run_dll(c_e, c_l);
        self.update_cn0(c_p, c_n);

        if code_offset < 0.0 {
            code_offset += self.samples_per_code as f64;
        }
        self.carrier_phase_shift_rolling_buffer.push(phi);
        self.code_phase_offset_rolling_buffer.push(code_offset);
        self.doppler_hz_rolling_buffer.push(self.doppler_hz);
        self.update_all_plots(false, sample.ts_sec);
        self.num_locked_samples += 1;
    }

    pub fn process_samples(&mut self, sample: &IQSample) {
        log::info!(
            "sat-{}: processing: ts_sec={:.4} sec: cn0={:.1} doppler={} code_off_sec={:.6} phi={:.2}",
            self.prn,
            sample.ts_sec,
            self.cn0,
            self.doppler_hz,
            self.code_offset_sec,
            self.phi,
        );

        match self.state {
            TrackState::SEARCHING => self.process_searching(sample),
            TrackState::LOCKED => self.process_locked(sample),
        }
    }
}
