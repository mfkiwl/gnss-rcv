pub struct RtlSdrDevice {}

impl RtlSdrDevice {
    pub fn init(&mut self) {
        log::warn!("RTL_SDR:");
        #[cfg(target_os = "linux")]
        {
            log::warn!("RTL_SDR__");
            let devices = rtlsdr_mt::devices();
            for dev in devices {
                log::warn!("found rtl-sdr: {}", dev);
            }
            let (mut ctl, mut reader) = rtlsdr_mt::open(0).unwrap();

            ctl.enable_agc().unwrap();
            ctl.set_ppm(-2).unwrap();
            //ctl.set_bandwidth();
            ctl.set_center_freq(1575.42e6).unwrap();
            ctl.set_sample_rate(2046 * 1000);
        }
    }
}
