use crate::{
    almanac::Almanac,
    channel::Channel,
    ephemeris::Ephemeris,
    util::{bmatch_n, bmatch_r, getbitu, hex_str, pack_bits, xor_bits},
};
use colored::Colorize;
use gnss_rtk::prelude::Epoch;

const SECS_PER_WEEK: u32 = 7 * 24 * 60 * 60;
const SDR_MAX_NSYM: usize = 18000;
const SDR_MAX_DATA: usize = 4096;

const THRESHOLD_SYNC: f64 = 0.4;
const THRESHOLD_LOST: f64 = 0.03;

#[derive(PartialEq)]
enum SyncState {
    NORMAL,
    REVERSED,
    NONE,
}
impl Default for SyncState {
    fn default() -> Self {
        SyncState::NORMAL
    }
}

#[derive(Default)]
pub struct Navigation {
    ssync: usize,
    fsync: usize,
    sync_state: SyncState,
    seq: u32,
    num_err: usize,
    syms: Vec<u8>,
    tsyms: Vec<f64>,
    data: Vec<u8>,
    time_data_sec: f64,
    count_ok: usize,
    count_err: usize,
    pub eph: Ephemeris,
}

impl Navigation {
    pub fn new() -> Self {
        Self {
            sync_state: SyncState::NORMAL,
            syms: vec![0; SDR_MAX_NSYM],
            tsyms: vec![0.0; SDR_MAX_NSYM],
            data: vec![0u8; SDR_MAX_DATA],
            ..Default::default()
        }
    }

    pub fn init(&mut self) {
        self.ssync = 0;
        self.fsync = 0;
        self.sync_state = SyncState::NORMAL;
        self.seq = 0;
        self.num_err = 0;

        for i in 0..SDR_MAX_NSYM {
            self.syms[i] = 0;
            self.tsyms[i] = 0.0;
        }
    }
}

impl Channel {
    fn nav_mean_ip(&self, n: usize) -> f64 {
        let mut p = 0.0;
        let len = self.hist.corr_p_hist.len();
        for i in 0..n {
            // weird math
            let c = self.hist.corr_p_hist[len - i - 1];
            p += (c.re / c.norm() - p) / (1.0 + i as f64);
        }
        p
    }
    fn nav_add_symbol(&mut self, sym: u8, ts: f64) {
        self.nav.syms.rotate_left(1);
        *self.nav.syms.last_mut().unwrap() = sym;

        self.nav.tsyms.rotate_left(1);
        *self.nav.tsyms.last_mut().unwrap() = ts;
    }

    fn nav_get_frame_sync_state(&self, preambule: &[u8]) -> SyncState {
        let bits = &self.nav.syms[SDR_MAX_NSYM - 308..];
        let preambule_len = preambule.len();

        assert!(bits.len() >= preambule_len);

        if bmatch_n(preambule, &bits[0..preambule_len])
            && bmatch_n(preambule, &bits[300..300 + preambule_len])
        {
            log::info!("{}: FRAME SYNC (N): ts={:.3}", self.sv, self.ts_sec);
            return SyncState::NORMAL;
        }
        if bmatch_r(preambule, &bits[0..preambule_len])
            && bmatch_r(preambule, &bits[300..300 + preambule_len])
        {
            log::info!("{}: FRAME SYNC (R): ts={:.3}", self.sv, self.ts_sec);
            return SyncState::REVERSED;
        }

        SyncState::NONE
    }

    fn nav_sync_symbol(&mut self, num: usize) -> bool {
        if self.nav.ssync == 0 {
            let n = if num <= 2 { 1 } else { 2 };
            let len = self.hist.corr_p_hist.len();

            let mut p = 0.0;
            for i in 0..2 * n {
                let code = if i < n { -1.0 } else { 1.0 };
                let corr = self.hist.corr_p_hist[len - 2 * n + i];
                p += corr.re * code / corr.norm(); // costly!
            }

            p /= 2.0 * n as f64;

            if p.abs() >= THRESHOLD_SYNC {
                self.nav.ssync = self.num_tracking_samples - n;
                log::info!("{}: SYNC: p={:.5} ssync={}", self.sv, p, self.nav.ssync);
            }
        } else if (self.num_tracking_samples - self.nav.ssync) % num == 0 {
            let p = self.nav_mean_ip(num);
            if p.abs() >= THRESHOLD_LOST {
                let sym: u8 = if p >= 0.0 { 1 } else { 0 };
                self.nav_add_symbol(sym, self.ts_sec);
                return true;
            } else {
                self.nav.ssync = 0;
                self.nav.sync_state = SyncState::NORMAL;
                log::info!("{}: SYNC {} p={}", self.sv, format!("LOST").red(), p)
            }
        }
        false
    }

    fn nav_decode_lnav_subframe1(&mut self) {
        let buf = &self.nav.data[0..];

        self.nav.eph.nav_decode_lnav_subframe1(buf, self.sv);
    }

    fn nav_decode_lnav_subframe2(&mut self) {
        let buf = &self.nav.data[0..];

        self.nav.eph.nav_decode_lnav_subframe2(buf, self.sv);
    }

    fn nav_decode_lnav_subframe3(&mut self) {
        let buf = &self.nav.data[0..];

        self.nav.eph.nav_decode_lnav_subframe3(buf, self.sv);
    }

    fn nav_decode_lnav_subframe4(&mut self) {
        let buf = &self.nav.data[0..];
        self.nav.eph.tow = getbitu(buf, 30, 17) * 6;
        let data_id = getbitu(buf, 60, 2);
        let svid = getbitu(buf, 62, 6);

        if data_id == 1 {
            if 25 <= svid && svid <= 32 {
                let mut alm = Almanac::default();
                alm.nav_decode_alm(buf, svid);
            } else if svid == 63 {
                /* page 25 */
                const ARRAY_IDX: [u32; 32] = [
                    68, 72, 76, 80, 90, 94, 98, 102, 106, 110, 120, 124, 128, 132, 136, 140, 150,
                    154, 158, 162, 166, 170, 180, 184, 188, 192, 196, 200, 210, 214, 218, 222,
                ];

                for sv in 1..=32 {
                    let pos = ARRAY_IDX[sv - 1];
                    let _svconf = getbitu(buf, pos, 4);
                }

                const ARRAY_SVH_IDX: [u32; 8] = [228, 240, 246, 252, 258, 270, 276, 282];
                for sv in 25..=32 {
                    let pos = ARRAY_SVH_IDX[sv - 25];
                    let svh = getbitu(buf, pos, 6);
                    if svh != 0 {
                        log::warn!("{}: subframe-4: sv {} is unhealthy", self.sv, sv)
                    }
                }
            } else if svid == 56 { /* page 18 */
                // handle iono, utc and leap seconds
            }
        }

        log::warn!(
            "{}: subframe-4: data_id={data_id} svid={svid} tow={}",
            self.sv,
            self.nav.eph.tow
        );
    }

    fn nav_decode_lnav_subframe5(&mut self) {
        let buf = &self.nav.data[0..];
        self.nav.eph.tow = getbitu(buf, 30, 17) * 6;
        let data_id = getbitu(buf, 60, 2);
        let svid = getbitu(buf, 62, 4);

        if data_id == 1 {
            if 1 <= svid && svid <= 24 {
                let mut alm = Almanac::default();
                alm.nav_decode_alm(buf, svid);
            } else if svid == 51 {
                //toas=getbitu(buff,i,8)*4096; i+=8;
            }
        }

        log::warn!(
            "{}: subframe-5: data_id={data_id} svid={svid} tow={}",
            self.sv,
            self.nav.eph.tow
        );
    }

    fn nav_decode_lnav_subframe(&mut self) -> u32 {
        let buf = &self.nav.data[0..];
        let subframe_id = getbitu(buf, 49, 3);
        let _alert = getbitu(buf, 47, 1);
        let _anti_spoof = getbitu(buf, 48, 1);
        self.nav.eph.tlm = getbitu(buf, 8, 14);

        match subframe_id {
            1 => self.nav_decode_lnav_subframe1(),
            2 => self.nav_decode_lnav_subframe2(),
            3 => self.nav_decode_lnav_subframe3(),
            4 => self.nav_decode_lnav_subframe4(),
            5 => self.nav_decode_lnav_subframe5(),
            _ => log::warn!("{}: invalid subframe id={subframe_id}", self.sv),
        }
        if self.nav.eph.week != 0 {
            self.nav.eph.ts_sec = self.ts_sec;
            let week_to_secs = self.nav.eph.week * SECS_PER_WEEK;
            let tow_secs_gpst = week_to_secs + self.nav.eph.tow;
            let toe_secs_gpst = week_to_secs + self.nav.eph.toe;

            self.nav.eph.tow_gpst = Epoch::from_gpst_seconds(tow_secs_gpst.into());
            self.nav.eph.toe_gpst = Epoch::from_gpst_seconds(toe_secs_gpst.into());
            log::warn!(
                "{}: ---- tow={} tgd={} toe={}",
                self.sv,
                self.nav.eph.tow_gpst,
                self.nav.eph.tgd,
                self.nav.eph.toe_gpst
            );
        }

        subframe_id
    }

    fn nav_decode_lnav(&mut self, sync: SyncState) {
        let mut buf = [0u8; 300];
        let rev = if sync == SyncState::NORMAL { 0 } else { 1 };
        let syms_len = self.nav.syms.len();
        let syms = &self.nav.syms[syms_len - 308..syms_len - 8];

        for i in 0..300 {
            buf[i] = syms[i] ^ rev;
        }

        if Self::nav_test_lnav_parity(&buf[0..]) {
            log::info!("{}: PARITY OK", self.sv);
            self.nav.fsync = self.num_tracking_samples;
            self.nav.sync_state = sync;
            pack_bits(&buf, 0, &mut self.nav.data);

            self.nav.seq = getbitu(&self.nav.data[0..], 30, 17); // tow (x 6s)
            self.nav.count_ok += 1;
            self.nav.time_data_sec = self.ts_sec - 20e-3 * 308.0;

            let id = self.nav_decode_lnav_subframe();

            let hex_str = hex_str(&self.nav.data[0..], 300);
            log::info!("{}: LNAV: id={id} -- {}", self.sv, hex_str);
        } else {
            self.nav.fsync = 0;
            self.nav.sync_state = SyncState::NORMAL;
            self.nav.count_err += 1;

            log::warn!("{}: PARITY ERROR", self.sv);
        }
    }

    fn nav_test_lnav_parity(syms: &[u8]) -> bool {
        let mask = [
            0x2EC7CD2, 0x1763E69, 0x2BB1F34, 0x15D8F9A, 0x1AEC7CD, 0x22DEA27,
        ];
        assert_eq!(syms.len(), 300);

        let mut data: u32 = 0;
        for i in 0..10 {
            for j in 0..30 {
                data = (data << 1) | syms[i * 30 + j] as u32;
            }
            if data & (1 << 30) != 0 {
                data = data ^ 0x3FFFFFC0;
            }
            for j in 0..6 {
                let v0 = (data >> 6) & mask[j];
                let v1: u8 = ((data >> (5 - j)) & 1) as u8;
                if xor_bits(v0) != v1 {
                    return false;
                }
            }
        }
        true
    }

    fn nav_decode_sbas(&mut self) {
        log::warn!("{}: SBAS frame", self.sv);
    }

    pub fn nav_decode(&mut self) {
        let preambule: [u8; 8] = [1, 0, 0, 0, 1, 0, 1, 1];
        if self.sv.prn >= 120 && self.sv.prn <= 158 {
            self.nav_decode_sbas();
            return;
        }

        if !self.nav_sync_symbol(20) {
            return;
        }

        if self.nav.fsync > 0 {
            if self.num_tracking_samples == self.nav.fsync + 6000 {
                let sync_state = self.nav_get_frame_sync_state(&preambule[0..]);
                if sync_state == self.nav.sync_state {
                    self.nav_decode_lnav(sync_state);
                } else {
                    self.nav.fsync = 0;
                    self.nav.sync_state = SyncState::NORMAL;
                }
            }
        } else if self.num_tracking_samples >= 20 * 308 + 1000 {
            let sync = self.nav_get_frame_sync_state(&preambule[0..]);
            if sync != SyncState::NONE {
                self.nav_decode_lnav(sync);
            }
        }
    }
}
