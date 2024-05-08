use colored::Colorize;
use rustfft::{num_complex::Complex64, FftPlanner};

use crate::constants::ACQUISITION_WINDOW_MSEC;
use crate::gold_code::GoldCode;
use crate::types::GnssCorrelationParam;
use crate::types::IQSample;
use crate::util::calc_correlation;
use crate::util::doppler_shift;
use crate::util::get_2nd_max;
use crate::util::get_max_with_idx;
use crate::util::get_num_samples_per_msec;

const PI: f64 = std::f64::consts::PI;

const DOPPLER_SPREAD_HZ: f64 = 8000.0;
const DOPPLER_SPREAD_BINS: u32 = 20;
const SNR_THRESHOLD: f64 = 3.0;

pub fn integrate_correlation(
    fft_planner: &mut FftPlanner<f64>,
    prn_code_fft: &Vec<Complex64>,
    sample: &IQSample,
    num_msec: usize,
    doppler_hz: f64,
) -> (Vec<f64>, Vec<Complex64>) {
    let num_samples_per_msec = get_num_samples_per_msec();
    let mut b_corr_float = vec![0f64; get_num_samples_per_msec()];
    let mut b_corr_complex = vec![Complex64::default(); get_num_samples_per_msec()];

    for idx in 0..num_msec {
        let range_lo = (idx + 0) * num_samples_per_msec;
        let range_hi = (idx + 1) * num_samples_per_msec;
        let mut iq_vec_sample = Vec::from(&sample.iq_vec[range_lo..range_hi]);
        assert_eq!(iq_vec_sample.len(), prn_code_fft.len());

        let shift_sample_sec = (idx * num_samples_per_msec) as f64 / sample.sample_rate as f64;
        doppler_shift(
            doppler_hz,
            shift_sample_sec,
            &mut iq_vec_sample,
            0.0,
            sample.sample_rate,
        );

        let corr = calc_correlation(fft_planner, &iq_vec_sample, &prn_code_fft);
        for i in 0..corr.len() {
            b_corr_float[i] += corr[i].norm();
            b_corr_complex[i] += corr[i];
        }
    }
    (b_corr_float, b_corr_complex)
}

fn find_best_doppler_shift_correlation(
    gold_code: &GoldCode,
    fft_planner: &mut FftPlanner<f64>,
    sample: &IQSample,
    sat_id: usize,
    num_msec: usize,
    estimate_hz: f64,
    spread_hz: f64,
) -> GnssCorrelationParam {
    let num_samples_per_msec = get_num_samples_per_msec();
    assert_eq!(sample.iq_vec.len(), num_samples_per_msec * num_msec);

    let mut best_param = GnssCorrelationParam::default();
    let prn_code_fft = gold_code.get_prn_code_fft(sat_id);

    let lo_hz = estimate_hz - spread_hz as f64;

    for i in 0..DOPPLER_SPREAD_BINS {
        let doppler_hz = lo_hz + i as f64 * 2.0 * spread_hz / DOPPLER_SPREAD_BINS as f64;
        let (corr_non_coherent, corr_coherent) =
            integrate_correlation(fft_planner, &prn_code_fft, sample, num_msec, doppler_hz);

        let b_corr_norm = corr_non_coherent.iter().map(|&x| x * x).sum::<f64>();

        if b_corr_norm > best_param.corr_norm {
            let b_corr_second = get_2nd_max(&corr_non_coherent);
            let (code_phase_offset, b_corr_peak) = get_max_with_idx(&corr_non_coherent);
            // XXX: this results in faster acquisition and processing. Why?
            //let b_peak_to_second = 10.0 * (b_corr_peak / b_corr_second).log10();
            let b_peak_to_second = 10.0 * ((b_corr_peak - b_corr_second) / b_corr_second).log10();
            let polar = corr_coherent[code_phase_offset].to_polar();
            best_param.snr = b_peak_to_second;
            best_param.doppler_hz = doppler_hz as f64;
            best_param.code_phase_offset = code_phase_offset;
            best_param.corr_norm = b_corr_norm;
            best_param.carrier_phase_shift = polar.1; // in radians
            assert!(-PI <= polar.1 && polar.1 <= PI);
        }
    }
    best_param
}

pub fn try_acquisition_one_sat(
    gold_code: &GoldCode,
    fft_planner: &mut FftPlanner<f64>,
    sat_id: usize,
    sample: &IQSample,
) -> Option<GnssCorrelationParam> {
    assert_eq!(
        sample.iq_vec.len(),
        get_num_samples_per_msec() * ACQUISITION_WINDOW_MSEC
    );
    let mut spread_hz = DOPPLER_SPREAD_HZ;
    let mut best_param = GnssCorrelationParam::default();

    while spread_hz > DOPPLER_SPREAD_BINS as f64 {
        let param = find_best_doppler_shift_correlation(
            gold_code,
            fft_planner,
            sample,
            sat_id,
            ACQUISITION_WINDOW_MSEC,
            best_param.doppler_hz,
            spread_hz,
        );
        if param.snr <= best_param.snr {
            break;
        }
        spread_hz = spread_hz / DOPPLER_SPREAD_BINS as f64;
        best_param = param;

        log::debug!(
            "BEST: sat_id={} -- estimate_hz={} spread_hz={:3} snr={:.3} phase_idx={}",
            sat_id,
            best_param.doppler_hz,
            spread_hz,
            best_param.snr,
            best_param.code_phase_offset
        );
    }
    if best_param.snr >= SNR_THRESHOLD {
        log::info!(
            " sat_id: {} -- doppler_hz: {:5} phase_idx: {:4} snr: {}",
            format!("{:2}", sat_id).yellow(),
            best_param.doppler_hz,
            best_param.code_phase_offset / 2,
            format!("{:.2}", best_param.snr).green(),
        );
        Some(best_param)
    } else {
        None
    }
}
