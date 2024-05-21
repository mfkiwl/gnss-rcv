use crate::{
    satellite::GnssSatellite,
    util::{bmatch_n, bmatch_r, getbitu, hex_str, pack_bits, xor_bits},
};
use colored::Colorize;

const SDR_MAX_NSYM: usize = 18000;
const SDR_MAX_DATA: usize = 4096;
const THRESHOLD_SYNC: f64 = 0.4; // 0.04;
const THRESHOLD_LOST: f64 = 0.03; // 0.003;

#[derive(PartialEq)]
enum SyncState {
    NORMAL,
    REVERSED,
    NONE,
}

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
}

impl Navigation {
    pub fn new() -> Self {
        Self {
            ssync: 0,
            fsync: 0,
            sync_state: SyncState::NORMAL,
            seq: 0,
            num_err: 0,
            syms: vec![0; SDR_MAX_NSYM],
            tsyms: vec![0.0; SDR_MAX_NSYM],
            data: vec![0u8; SDR_MAX_DATA],
            time_data_sec: 0.0,
            count_ok: 0,
            count_err: 0,
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

impl GnssSatellite {
    fn nav_mean_ip(&self, n: usize) -> f64 {
        let mut p = 0.0;
        let len = self.corr_p_hist.len();
        for i in 0..n {
            // weird math
            let c = self.corr_p_hist[len - i - 1];
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
        let bits_len = bits.len();
        assert!(bits_len >= preambule_len);

        if bmatch_n(preambule, &bits[0..preambule_len])
            && bmatch_n(preambule, &bits[300..300 + preambule_len])
        {
            log::warn!("sat-{}: FRAME SYNC (N): ts={:.3}", self.prn, self.ts_sec);
            return SyncState::NORMAL;
        }
        if bmatch_r(preambule, &bits[0..preambule_len])
            && bmatch_r(preambule, &bits[0..preambule_len])
        {
            log::warn!("sat-{}: FRAME SYNC (R): ts={:.3}", self.prn, self.ts_sec);
            return SyncState::REVERSED;
        }

        SyncState::NONE
    }

    fn nav_sync_symbol(&mut self, num: usize) -> bool {
        if self.nav.ssync == 0 {
            let n = if num <= 2 { 1 } else { 2 };
            let len = self.corr_p_hist.len();
            let mut v = vec![];
            let mut p = 0.0;
            for i in 0..2 * n {
                let code = if i < n { -1.0 } else { 1.0 };
                let corr = self.corr_p_hist[len - 2 * n + i];
                p += corr.re * code / corr.norm(); // costly!
                v.push(corr.re / corr.norm());
            }

            p /= 2.0 * n as f64;

            if p.abs() >= THRESHOLD_SYNC {
                self.nav.ssync = self.num_tracking_samples - n;
                log::warn!(
                    "sat-{:2} SYNC: p={:.5} ssync={}",
                    self.prn,
                    p,
                    self.nav.ssync
                );
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
                log::warn!("sat-{}: SYNC {} p={}", self.prn, format!("LOST").red(), p)
            }
        }
        false
    }

    fn nav_decode_lnav(&mut self, sync: SyncState) {
        let mut buf = vec![];
        let rev = if sync == SyncState::NORMAL { 0 } else { 1 };
        let syms_len = self.nav.syms.len();
        let syms = &self.nav.syms[syms_len - 308..syms_len - 8];

        for i in 0..300 {
            buf.push(syms[i] ^ rev)
        }

        let ts_sec = self.ts_sec - 20e-3 * 308.0;
        if Self::nav_test_lnav_parity(&buf[0..]) {
            log::warn!("sat-{}: PARITY OK", self.prn);
            self.nav.fsync = self.num_tracking_samples;
            self.nav.sync_state = sync;
            pack_bits(&buf, 0, &mut self.nav.data);
            self.nav.seq = getbitu(&self.nav.data[0..], 30 as usize, 17 as usize); // tow (x 6s)
            self.nav.count_ok += 1;
            self.nav.time_data_sec = ts_sec;

            let hex_str = hex_str(&self.nav.data[0..], 300);
            log::warn!("sat-{}: LNAV: {}", self.prn, hex_str);
        } else {
            self.nav.fsync = 0;
            self.nav.sync_state = SyncState::NORMAL;
            self.nav.count_err += 1;

            log::warn!("sat-{}: PARITY ERROR", self.prn);
        }
    }

    fn nav_test_lnav_parity(syms: &[u8]) -> bool {
        let mask = [
            0x2EC7CD2, 0x1763E69, 0x2BB1F34, 0x15D8F9A, 0x1AEC7CD, 0x22DEA27,
        ];

        let mut buff: u32 = 0;
        for i in 0..10 {
            for j in 0..30 {
                buff = (buff << 1) | syms[i * 30 + j] as u32;
            }
            if buff & (1 << 30) != 0 {
                buff = buff ^ 0x3FFFFFC0;
            }
            for j in 0..6 {
                let v0: u32 = (buff >> 6) & mask[j];
                let v1: u8 = ((buff >> (5 - j)) & 1) as u8;
                if xor_bits(v0) != v1 {
                    return false;
                }
            }
        }
        true
    }

    pub fn nav_decode(&mut self) {
        let preamb: Vec<u8> = [1, 0, 0, 0, 1, 0, 1, 1].to_vec();
        if self.prn >= 120 && self.prn <= 158 {
            //decode_SBAS(ch);
            //return;
            assert!(false);
        }

        if !self.nav_sync_symbol(20) {
            return;
        }

        if self.nav.fsync > 0 {
            if self.num_tracking_samples == self.nav.fsync + 6000 {
                let sync_state = self.nav_get_frame_sync_state(&preamb[0..]);
                if sync_state == self.nav.sync_state {
                    self.nav_decode_lnav(sync_state);
                } else {
                    self.nav.fsync = 0;
                    self.nav.sync_state = SyncState::NORMAL;
                }
            }
        } else if self.num_tracking_samples >= 20 * 308 + 1000 {
            let sync = self.nav_get_frame_sync_state(&preamb[0..]);
            if sync != SyncState::NONE {
                self.nav_decode_lnav(sync);
            }
        }
    }
}
