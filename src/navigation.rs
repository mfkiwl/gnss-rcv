use crate::{
    channel::Channel,
    util::{
        bmatch_n, bmatch_r, getbits, getbits2, getbitu, getbitu2, hex_str, pack_bits, xor_bits,
    },
};
use colored::Colorize;

const SDR_MAX_NSYM: usize = 18000;
const SDR_MAX_DATA: usize = 4096;
const THRESHOLD_SYNC: f64 = 0.4; // 0.04;
const THRESHOLD_LOST: f64 = 0.03; // 0.003;

const P2_5: f64 = 0.03125; /* 2^-5 */
const P2_19: f64 = 1.907348632812500E-06; /* 2^-19 */
const P2_29: f64 = 1.862645149230957E-09; /* 2^-29 */
const P2_31: f64 = 4.656612873077393E-10; /* 2^-31 */
const P2_33: f64 = 1.164153218269348E-10; /* 2^-33 */
const P2_43: f64 = 1.136868377216160E-13; /* 2^-43 */
const P2_55: f64 = 2.775557561562891E-17; /* 2^-55 */
const SC2RAD: f64 = 3.1415926535898; /* semi-circle to radian (IS-GPS) */

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
struct Ephemeris {
    update: bool,
    tow_gpst: f64,
    tlm: u32,

    iode: u32,
    iodc: u32, /* IODE,IODC */
    sva: u32,  /* SV accuracy (URA index) */
    svh: u32,  /* SV health (0:ok) */
    week: u32, /* GPS/QZS: gps week, GAL: galileo week */
    code: u32, /* GPS/QZS: code on L2, GAL/CMP: data sources */
    flag: u32, /* GPS/QZS: L2 P data flag, CMP: nav type */
    /* GPS/QZS:tgd[0]=TGD */
    /* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 */
    /* CMP    :tgd[0]=BGD1,tgd[1]=BGD2 */
    f0: f64,
    f1: f64,
    f2: f64,  /* SV clock parameters (af0,af1,af2) */
    tgd: f64, /* group delay parameters */
    omg0: f64,
    omgd: f64,
    omg: f64,
    cic: f64,
    cis: f64,
    crc: f64,
    crs: f64,
    cus: f64,
    cuc: f64,
    idot: f64,
    i0: f64,
    m0: f64,
    a: f64,
    e: f64,
    deln: f64,
    toes: f64,
    fit: f64,
    /*
    gtime_t toe,toc,ttr; /* Toe,Toc,T_trans */
                        /* SV orbit parameters */
    double A,e,i0,OMG0,omg,M0,deln,OMGd,idot;
    double crc,crs,cuc,cus,cic,cis;
    double toes;        /* Toe (s) in week */
    double fit;         /* fit interval (h) */


                        */
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
    eph: Ephemeris,
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
        let bits_len = bits.len();
        assert!(bits_len >= preambule_len);

        if bmatch_n(preambule, &bits[0..preambule_len])
            && bmatch_n(preambule, &bits[300..300 + preambule_len])
        {
            log::warn!("{}: FRAME SYNC (N): ts={:.3}", self.sv, self.ts_sec);
            return SyncState::NORMAL;
        }
        if bmatch_r(preambule, &bits[0..preambule_len])
            && bmatch_r(preambule, &bits[0..preambule_len])
        {
            log::warn!("{}: FRAME SYNC (R): ts={:.3}", self.sv, self.ts_sec);
            return SyncState::REVERSED;
        }

        SyncState::NONE
    }

    fn nav_sync_symbol(&mut self, num: usize) -> bool {
        if self.nav.ssync == 0 {
            let n = if num <= 2 { 1 } else { 2 };
            let len = self.hist.corr_p_hist.len();
            let mut v = vec![];
            let mut p = 0.0;
            for i in 0..2 * n {
                let code = if i < n { -1.0 } else { 1.0 };
                let corr = self.hist.corr_p_hist[len - 2 * n + i];
                p += corr.re * code / corr.norm(); // costly!
                v.push(corr.re / corr.norm());
            }

            p /= 2.0 * n as f64;

            if p.abs() >= THRESHOLD_SYNC {
                self.nav.ssync = self.num_tracking_samples - n;
                log::warn!("{}: SYNC: p={:.5} ssync={}", self.sv, p, self.nav.ssync);
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
                log::warn!("{}: SYNC {} p={}", self.sv, format!("LOST").red(), p)
            }
        }
        false
    }

    fn nav_decode_lnav_subframe1(&mut self) {
        let data = &self.nav.data[0..];

        self.nav.eph.week = getbitu(data, 60, 10) + 1024;
        self.nav.eph.code = getbitu(data, 70, 2);
        self.nav.eph.sva = getbitu(data, 72, 4);
        self.nav.eph.svh = getbitu(data, 76, 6);
        self.nav.eph.iodc = getbitu2(data, 82, 2, 210, 8);
        self.nav.eph.flag = getbitu(data, 90, 1);
        self.nav.eph.tgd = getbits(data, 196, 8) as f64 * P2_31;
        self.nav.eph.f2 = getbits(data, 240, 8) as f64 * P2_55;
        self.nav.eph.f1 = getbits(data, 248, 16) as f64 * P2_43;
        self.nav.eph.f0 = getbits(data, 270, 22) as f64 * P2_31;
    }

    fn nav_decode_lnav_subframe2(&mut self) {
        let data = &self.nav.data[0..];
        let oldiode = self.nav.eph.iode;

        self.nav.eph.iode = getbitu(data, 60, 8);
        self.nav.eph.crs = getbits(data, 68, 16) as f64 * P2_5;
        self.nav.eph.deln = getbits(data, 90, 16) as f64 * P2_43 * SC2RAD;
        self.nav.eph.m0 = getbits2(data, 106, 8, 120, 24) as f64 * P2_31 * SC2RAD;
        self.nav.eph.cuc = getbits(data, 150, 16) as f64 * P2_29;
        self.nav.eph.e = getbitu2(data, 166, 8, 180, 24) as f64 * P2_33;
        self.nav.eph.cus = getbits(data, 210, 16) as f64 * P2_29;
        let sqrt_a = getbitu2(data, 226, 8, 240, 24) as f64 * P2_19;
        self.nav.eph.toes = getbitu(data, 270, 16) as f64 * 16.0;
        self.nav.eph.fit = getbitu(data, 286, 1) as f64;
        self.nav.eph.a = sqrt_a * sqrt_a;

        /* ephemeris update flag */
        if oldiode != self.nav.eph.iode {
            self.nav.eph.update = true;
        }
    }

    fn nav_decode_lnav_subframe3(&mut self) {
        let data = &self.nav.data[0..];
        let oldiode = self.nav.eph.iode;

        self.nav.eph.cic = getbits(data, 60, 16) as f64 * P2_29;
        self.nav.eph.omg0 = getbits2(data, 76, 8, 90, 24) as f64 * P2_31 * SC2RAD;
        self.nav.eph.cis = getbits(data, 120, 16) as f64 * P2_29;
        self.nav.eph.i0 = getbits2(data, 136, 8, 150, 24) as f64 * P2_31 * SC2RAD;
        self.nav.eph.crc = getbits(data, 180, 16) as f64 * P2_5;
        self.nav.eph.omg = getbits2(data, 196, 8, 210, 24) as f64 * P2_31 * SC2RAD;
        self.nav.eph.omgd = getbits(data, 240, 24) as f64 * P2_43 * SC2RAD;
        self.nav.eph.iode = getbitu(data, 270, 8);
        self.nav.eph.idot = getbits(data, 278, 14) as f64 * P2_43 * SC2RAD;

        /* ephemeris update flag */
        if oldiode != self.nav.eph.iode {
            self.nav.eph.update = true;
        }
    }

    fn nav_decode_lnav_subframe4(&mut self) {}
    fn nav_decode_lnav_subframe5(&mut self) {}

    fn nav_decode_lnav_subframe(&mut self) -> u32 {
        let data = &self.nav.data[0..];
        let subframe_id = getbitu(data, 49, 3);
        self.nav.eph.tow_gpst = getbitu(data, 30, 17) as f64 * 6.0;
        let alert = getbitu(data, 47, 1);
        let anti_spoof = getbitu(data, 48, 1);
        self.nav.eph.tlm = getbitu(data, 8, 14);

        log::warn!(
            "{}: subframe-id={subframe_id} tow={:.2} as={anti_spoof} alert={alert}",
            self.sv,
            self.nav.eph.tow_gpst,
        );

        match subframe_id {
            1 => self.nav_decode_lnav_subframe1(),
            2 => self.nav_decode_lnav_subframe2(),
            3 => self.nav_decode_lnav_subframe3(),
            4 => self.nav_decode_lnav_subframe4(),
            5 => self.nav_decode_lnav_subframe5(),
            _ => log::warn!("{}: invalid subframe id={subframe_id}", self.sv),
        }
        subframe_id
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
            log::info!("{}: PARITY OK", self.sv);
            self.nav.fsync = self.num_tracking_samples;
            self.nav.sync_state = sync;
            pack_bits(&buf, 0, &mut self.nav.data);

            self.nav.seq = getbitu(&self.nav.data[0..], 30, 17); // tow (x 6s)
            self.nav.count_ok += 1;
            self.nav.time_data_sec = ts_sec;

            let id = self.nav_decode_lnav_subframe();

            let hex_str = hex_str(&self.nav.data[0..], 300);
            log::warn!("{}: LNAV: id={id} -- {}", self.sv, hex_str);
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

        let mut data: u32 = 0;
        for i in 0..10 {
            for j in 0..30 {
                data = (data << 1) | syms[i * 30 + j] as u32;
            }
            if data & (1 << 30) != 0 {
                data = data ^ 0x3FFFFFC0;
            }
            for j in 0..6 {
                let v0: u32 = (data >> 6) & mask[j];
                let v1: u8 = ((data >> (5 - j)) & 1) as u8;
                if xor_bits(v0) != v1 {
                    return false;
                }
            }
        }
        true
    }

    pub fn nav_decode(&mut self) {
        let preamb: Vec<u8> = [1, 0, 0, 0, 1, 0, 1, 1].to_vec();
        if self.sv.prn >= 120 && self.sv.prn <= 158 {
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
