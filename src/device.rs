use rustfft::num_complex::Complex64;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

pub struct RtlSdrDevice {
    controller: rtlsdr_mt::Controller,
    iq_vec: Vec<Complex64>,
    reader: rtlsdr_mt::Reader,
    exit_req: Arc<AtomicBool>,
    num_samples: u64,
}

impl RtlSdrDevice {
    pub fn new(exit_req: Arc<AtomicBool>) -> Result<RtlSdrDevice, ()> {
        let devices = rtlsdr_mt::devices();

        for dev in devices {
            log::warn!("found rtl-sdr: {:?}", dev);
        }

        let (ctl, reader) = rtlsdr_mt::open(0)?;
        let mut m = Self {
            controller: ctl,
            reader,
            iq_vec: vec![],
            exit_req,
            num_samples: 0,
        };

        let mut tunes = rtlsdr_mt::TunerGains::default();
        let gains = m.controller.tuner_gains(&mut tunes);
        log::warn!("gain: {:?}", gains);

        m.controller.enable_agc().expect("Failed to enable agc");
        m.controller
            .set_bias_tee(1)
            .expect("Failed to set bias tee");
        m.controller
            .set_center_freq(1_575_420_000)
            .expect("Failed to change center freq");
        m.controller
            .set_sample_rate(2046 * 1000)
            .expect("Failed to change sample rate");

        Ok(m)
    }

    pub fn start_reading(&mut self) {
        // 0: use default num of buffer and buffer size
        self.reader
            .read_async(0, 0, |array| {
                let mut re_sum = 0.0;
                let mut im_sum = 0.0;
                for i in (0..array.len()).step_by(2) {
                    let re = array[i + 0] as f64 - 126.0 / 128.0;
                    let im = array[i + 1] as f64 - 126.0 / 128.0;
                    re_sum += array[i + 0] as f64;
                    im_sum += array[i + 1] as f64;
                    self.iq_vec.push(Complex64 { re, im });
                    self.num_samples += 1;
                }
                log::warn!(
                    "avg: {} {}",
                    2.0 * re_sum / array.len() as f64,
                    2.0 * im_sum / array.len() as f64
                );
                if self.exit_req.load(Ordering::SeqCst) || self.iq_vec.len() > 2046 * 1000 * 10 {
                    log::warn!("rtlsdr: stopping read. num_samples={}", self.num_samples);
                    self.controller.cancel_async_read();
                }
            })
            .unwrap();
    }

    pub fn read_iq_data(
        &mut self,
        _off_samples: usize,
        num_samples: usize,
    ) -> Result<Vec<Complex64>, Box<dyn std::error::Error>> {
        if cfg!(target_os = "macos") {
            assert!(self.iq_vec.len() >= num_samples);

            let iq_vec = self.iq_vec[0..num_samples].to_vec();
            let _ = self.iq_vec.drain(0..num_samples);

            Ok(iq_vec)
        } else {
            Ok(vec![])
        }
    }
}
