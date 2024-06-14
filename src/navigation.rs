use std::sync::Mutex;

use crate::{
    almanac::Almanac,
    channel::Channel,
    constants::{P2_24, P2_27, P2_30, P2_50},
    ephemeris::Ephemeris,
    util::{bits_equal, bits_opposed, getbits, getbits2, getbitu, hex_str, setbitu, xor_bits},
};
use colored::Colorize;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::Epoch;
use once_cell::sync::Lazy;

const SECS_PER_WEEK: u32 = 7 * 24 * 60 * 60;
const SDR_MAX_NSYM: usize = 18000;

const THRESHOLD_SYNC: f64 = 0.4; // 0.02
const THRESHOLD_LOST: f64 = 0.03; // 0.002

static GPS_ALMANAC: Lazy<Mutex<Vec<Almanac>>> =
    Lazy::new(|| Mutex::new(vec![Almanac::default(); 32]));

#[derive(PartialEq, Debug)]
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
    ssync: usize, // beginning of nav frame in num_trk_samples
    fsync: usize, // set to num_trk_samples after nav parity OK
    sync_state: SyncState,
    bits: Vec<u8>, // navigation bits
    count_parity_err: usize,
    pub eph: Ephemeris,
}

impl Navigation {
    pub fn new(sv: SV) -> Self {
        Self {
            sync_state: SyncState::NORMAL,
            bits: vec![0; SDR_MAX_NSYM],
            eph: Ephemeris::new(sv),
            ..Default::default()
        }
    }

    pub fn init(&mut self) {
        self.ssync = 0;
        self.fsync = 0;
        self.sync_state = SyncState::NORMAL;
        self.bits.fill(0);
    }
}

impl Channel {
    fn nav_mean_ip(&self, n: usize) -> f64 {
        let mut p = 0.0;
        let len = self.hist.corr_p.len();

        for i in 0..n {
            // weird math
            let c = self.hist.corr_p[len - n + i];
            //p += (c.re / c.norm() - p) / (1 + i) as f64;
            p += c.re / c.norm();
        }
        p / n as f64
    }
    fn nav_add_bit(&mut self, bit: u8) {
        self.nav.bits.rotate_left(1);
        *self.nav.bits.last_mut().unwrap() = bit;
    }

    fn nav_get_frame_sync_state(&self, preambule: &[u8]) -> SyncState {
        let bits = &self.nav.bits[SDR_MAX_NSYM - 308..];
        let bits_beg = &bits[0..preambule.len()];
        let bits_end = &bits[300..300 + preambule.len()];
        let mut sync_state = SyncState::NONE;

        if bits_equal(preambule, bits_beg) && bits_equal(preambule, bits_end) {
            sync_state = SyncState::NORMAL;
        } else if bits_opposed(preambule, bits_beg) && bits_opposed(preambule, bits_end) {
            sync_state = SyncState::REVERSED;
        }
        if sync_state != SyncState::NONE {
            log::info!(
                "{}: FRAME SYNC {sync_state:?}: ts={:.3}",
                self.sv,
                self.ts_sec
            );
        }

        sync_state
    }

    fn nav_sync_symbol(&mut self, num: usize) -> bool {
        if self.nav.ssync == 0 {
            let n = if num <= 2 { 1 } else { num - 1 };
            let len = self.hist.corr_p.len();

            let mut p = 0.0;
            let mut r = 0.0;
            for i in 0..2 * n {
                let code = if i < n { -1.0 } else { 1.0 };
                let corr = self.hist.corr_p[len - 2 * n + i];
                let corr_re = corr.re / corr.norm(); // XXX: shouldn't be required

                p += corr_re * code;
                r += corr_re.abs();
            }

            p /= 2.0 * n as f64;
            r /= 2.0 * n as f64;

            if p.abs() >= r && r >= THRESHOLD_SYNC {
                self.nav.ssync = self.num_trk_samples - n;
                log::info!("{}: SYNC: p={:.5} ssync={}", self.sv, p, self.nav.ssync);
            }
        } else if (self.num_trk_samples - self.nav.ssync) % num == 0 {
            let p = self.nav_mean_ip(num);
            if p.abs() >= THRESHOLD_LOST {
                let sym: u8 = if p >= 0.0 { 1 } else { 0 };
                self.nav_add_bit(sym);
                return true;
            } else {
                self.nav.ssync = 0;
                self.nav.sync_state = SyncState::NORMAL;
                log::info!("{}: SYNC {} p={}", self.sv, format!("LOST").red(), p)
            }
        }
        false
    }

    fn nav_decode_lnav_subframe1(&mut self, buf: &[u8]) {
        self.nav.eph.nav_decode_lnav_subframe1(buf, self.sv);
    }

    fn nav_decode_lnav_subframe2(&mut self, buf: &[u8]) {
        self.nav.eph.nav_decode_lnav_subframe2(buf, self.sv);
    }

    fn nav_decode_lnav_subframe3(&mut self, buf: &[u8]) {
        self.nav.eph.nav_decode_lnav_subframe3(buf, self.sv);
    }

    fn nav_decode_lnav_subframe4(&mut self, buf: &[u8]) {
        self.nav.eph.tow = getbitu(buf, 30, 17) * 6;
        let data_id = getbitu(buf, 60, 2);
        let svid = getbitu(buf, 62, 6);

        if data_id == 1 {
            let alm_array = GPS_ALMANAC.lock().unwrap();
            if 25 <= svid && svid <= 32 {
                let mut alm = alm_array[svid as usize - 1];
                alm.nav_decode_alm(buf, svid);
                log::warn!("{}: {:?}", self.sv, alm);
            } else if svid == 63 {
                /* page 25 */
                const ARRAY_SVCONF_IDX: [usize; 32] = [
                    68, 72, 76, 80, 90, 94, 98, 102, 106, 110, 120, 124, 128, 132, 136, 140, 150,
                    154, 158, 162, 166, 170, 180, 184, 188, 192, 196, 200, 210, 214, 218, 222,
                ];

                for sv in 1..=32 {
                    let mut alm = alm_array[sv - 1];
                    let pos = ARRAY_SVCONF_IDX[sv - 1];

                    alm.svconf = getbitu(buf, pos, 4);
                }

                const ARRAY_SVH_IDX: [usize; 8] = [228, 240, 246, 252, 258, 270, 276, 282];
                for sv in 25..=32 {
                    let mut alm = alm_array[sv - 1];
                    let pos = ARRAY_SVH_IDX[sv - 25];
                    alm.svh = getbitu(buf, pos, 6);
                    if alm.svh != 0 {
                        log::warn!("{}: subframe-4: sv {} is unhealthy", self.sv, sv)
                    }
                }
            } else if svid == 56 {
                /* page 18 */
                // handle iono, utc and leap seconds
                let mut ion = [0.0; 8];

                ion[0] = getbits(buf, 68, 8) as f64 * P2_30;
                ion[1] = getbits(buf, 76, 8) as f64 * P2_27;
                ion[2] = getbits(buf, 90, 8) as f64 * P2_24;
                ion[3] = getbits(buf, 98, 8) as f64 * P2_24;
                ion[4] = getbits(buf, 106, 8) as f64 * 2.0_f64.powi(11);
                ion[5] = getbits(buf, 120, 8) as f64 * 2.0_f64.powi(14);
                ion[6] = getbits(buf, 128, 8) as f64 * 2.0_f64.powi(16);
                ion[7] = getbits(buf, 136, 8) as f64 * 2.0_f64.powi(16);

                let mut utc: [f64; 4] = [0.0; 4];

                utc[0] = getbits2(buf, 180, 24, 210, 8) as f64 * P2_30;
                utc[1] = getbits(buf, 150, 24) as f64 * P2_50;
                utc[2] = getbits(buf, 218, 8) as f64 * 2.0_f64.powi(12);
                utc[3] = getbits(buf, 226, 8) as f64;
            }
        }

        log::warn!(
            "{}: {}: data_id={data_id} svid={svid} tow={}",
            self.sv,
            format!("subframe-4").blue(),
            self.nav.eph.tow
        );
    }

    fn nav_decode_lnav_subframe5(&mut self, buf: &[u8]) {
        self.nav.eph.tow = getbitu(buf, 30, 17) * 6;
        let data_id = getbitu(buf, 60, 2);
        let svid = getbitu(buf, 62, 4);

        if data_id == 1 {
            let alm_array = GPS_ALMANAC.lock().unwrap();
            if 1 <= svid && svid <= 24 {
                let mut alm = alm_array[svid as usize];
                alm.nav_decode_alm(buf, svid);
                log::warn!("{}: {:?}", self.sv, alm);
            } else if svid == 51 {
                let toas = getbitu(buf, 68, 8) * 4096;
                let week = getbitu(buf, 76, 8) + 2048;

                const ARRAY_SVH_IDX: [usize; 24] = [
                    90, 96, 102, 108, 120, 126, 132, 138, 150, 156, 162, 168, 180, 186, 192, 198,
                    210, 216, 222, 228, 240, 246, 252, 258,
                ];
                for sv in 1..=24 {
                    let mut alm = alm_array[sv - 1];
                    let pos = ARRAY_SVH_IDX[sv - 25];
                    alm.svh = getbitu(buf, pos, 6);
                    if alm.svh != 0 {
                        log::warn!("{}: subframe-4: sv {} is unhealthy", self.sv, sv)
                    }
                }
                for sv in 1..=32 {
                    let mut alm = alm_array[sv - 1];
                    alm.week = week;
                    alm.toas = toas;
                }
            } else {
                log::warn!("XXX unknown svid={}", svid);
            }
        }

        log::warn!(
            "{}: {}: data_id={data_id} svid={svid} tow={}",
            self.sv,
            format!("subframe-5").blue(),
            self.nav.eph.tow
        );
    }

    fn nav_decode_lnav_subframe(&mut self, buf: &[u8]) -> u32 {
        let preamble = getbitu(buf, 0, 8);
        assert_eq!(preamble, 0x8b);
        self.nav.eph.tlm = getbitu(buf, 8, 14);
        let _isf = getbitu(buf, 22, 1);
        let _rsvd = getbitu(buf, 23, 1);
        let _alert = getbitu(buf, 47, 1);
        let _anti_spoof = getbitu(buf, 48, 1);
        let subframe_id = getbitu(buf, 49, 3);
        let zero = getbitu(buf, 58, 2);
        assert_eq!(zero, 0);

        match subframe_id {
            1 => self.nav_decode_lnav_subframe1(buf),
            2 => self.nav_decode_lnav_subframe2(buf),
            3 => self.nav_decode_lnav_subframe3(buf),
            4 => self.nav_decode_lnav_subframe4(buf),
            5 => self.nav_decode_lnav_subframe5(buf),
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
                "{}: tow={:?} tgd={:+e} toe={:?}",
                self.sv,
                self.nav.eph.tow_gpst,
                self.nav.eph.tgd,
                self.nav.eph.toe_gpst
            );
        }

        subframe_id
    }

    fn nav_decode_lnav(&mut self, sync: SyncState) {
        let rev = if sync == SyncState::NORMAL { 0 } else { 1 };
        let syms_len = self.nav.bits.len();
        let syms = &self.nav.bits[syms_len - 308..syms_len - 8];
        let bits: Vec<_> = syms.iter().map(|v| v ^ rev).collect();
        let mut nav_data = vec![0; 300];

        if Self::nav_test_lnav_parity(&bits, &mut nav_data) {
            self.nav.fsync = self.num_trk_samples;
            self.nav.sync_state = sync;

            let id = self.nav_decode_lnav_subframe(&nav_data);
            let hex_str = hex_str(&nav_data[0..300]);
            log::info!("{}: LNAV: id={id} -- {hex_str}", self.sv);
        } else {
            self.nav.fsync = 0;
            self.nav.sync_state = SyncState::NORMAL;
            self.nav.count_parity_err += 1;

            log::warn!("{}: PARITY ERROR", self.sv);
        }
    }

    fn nav_test_lnav_parity(bits: &Vec<u8>, nav_data: &mut [u8]) -> bool {
        const MASK: [u32; 6] = [
            0x2EC7CD2, 0x1763E69, 0x2BB1F34, 0x15D8F9A, 0x1AEC7CD, 0x22DEA27,
        ];
        assert_eq!(bits.len(), 300);

        let mut data: u32 = 0;
        for i in 0..10 {
            for j in 0..30 {
                data = (data << 1) | bits[i * 30 + j] as u32;
            }
            if data & (1 << 30) != 0 {
                data ^= 0x3FFFFFC0;
            }
            for j in 0..6 {
                let v0 = (data >> 6) & MASK[j];
                let v1: u8 = ((data >> (5 - j)) & 1) as u8;
                if xor_bits(v0) != v1 {
                    return false;
                }
            }
            setbitu(nav_data, 30 * i, 24, (data >> 6) & 0xFFFFFF);
            setbitu(nav_data, 30 * i + 24, 6, 0);
        }
        true
    }

    fn nav_decode_sbas(&mut self) {
        log::warn!("{}: SBAS frame", self.sv);
    }

    pub fn nav_decode(&mut self) {
        const PREAMBULE: [u8; 8] = [1, 0, 0, 0, 1, 0, 1, 1];
        let preambule = &PREAMBULE[0..];

        if self.sv.prn >= 120 && self.sv.prn <= 158 {
            self.nav_decode_sbas();
            return;
        }

        if !self.nav_sync_symbol(20) {
            return;
        }

        if self.nav.fsync > 0 {
            if self.num_trk_samples == self.nav.fsync + 300 * 20 {
                let sync = self.nav_get_frame_sync_state(preambule);
                if sync == self.nav.sync_state {
                    self.nav_decode_lnav(sync);
                }
            } else if self.num_trk_samples > self.nav.fsync + 300 * 20 {
                self.nav.fsync = 0;
                self.nav.ssync = 0;
                self.nav.sync_state = SyncState::NORMAL;
            }
        } else if self.num_trk_samples >= 20 * 308 + 1000 {
            let sync = self.nav_get_frame_sync_state(preambule);
            if sync != SyncState::NONE {
                self.nav_decode_lnav(sync);
            }
        }
    }
}
