use colored::Colorize;
use gnss_rs::sv::SV;
use gnss_rtk::prelude::Epoch;

use crate::{
    constants::{P2_5, P2_19, P2_29, P2_31, P2_33, P2_43, P2_55, SC2RAD},
    util::{getbits, getbits2, getbitu, getbitu2},
};

#[derive(Default, Clone, Copy)]
pub struct Ephemeris {
    pub sv: SV,
    pub tow: u32,
    pub cn0: f64,
    pub code_off_sec: f64,
    pub ts_sec: f64, // receiver time for 1st subframe
    pub tow_gpst: Epoch,
    pub toe_gpst: Epoch, // cf toe
    pub tlm: u32,

    pub iode: u32,    // Issue of Data, Ephemeris
    pub iodc: u32,    // Issue of Data, Clock
    pub sva: u32,     // SV accuracy (URA index)
    pub svh: u32,     // SV health (0:ok)
    pub week: u32,    // GPS/QZS: gps week, GAL: galileo week
    pub code: u32,    // GPS/QZS: code on L2, GAL/CMP: data sources
    pub flag: u32,    // GPS/QZS: L2 P data flag, CMP: nav type
    pub tgd: f64,     // GPS: Estimated Group Delay Differential
    pub f0: f64,      // SV Clock Bias Correction Coefficient
    pub f1: f64,      // SV Clock Drift Correction Coefficient
    pub f2: f64,      // Drift Rate Correction Coefficient
    pub omg: f64,     // Argument of Perigee
    pub omg0: f64,    // Longitude of Ascending Node of Orbit Plane at Weekly Epoch
    pub omg_dot: f64, // Rate of Right Ascension
    pub cic: f64, // Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination
    pub cis: f64, // Amplitude of the Sine   Harmonic Correction Term to the Angle of Inclination
    pub crc: f64, // Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius
    pub crs: f64, // Amplitude of the Sine   Harmonic Correction Term to the Orbit Radius
    pub cuc: f64, // Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude
    pub cus: f64, // Amplitude of the Sine   Harmonic Correction Term to the Argument of Latitude
    pub i_dot: f64, // Rate of Inclination Angle
    pub i0: f64,  // Inclination Angle at Reference Time
    pub m0: f64,  // Mean Anomaly at Reference Time
    pub a: f64,   // semi major axis
    pub ecc: f64, // Eccentricity
    pub deln: f64, // Mean Motion Difference From Computed Value
    pub toc: u32, // Time of Clock
    pub toe: u32, // Reference Time Ephemeris
    pub fit: u32, // fit interval (h)
}

impl Ephemeris {
    pub fn new(sv: SV) -> Self {
        Self {
            sv,
            ..Default::default()
        }
    }
    pub fn nav_decode_lnav_subframe1(&mut self, buf: &[u8], sv: SV) {
        self.tow = getbitu(buf, 30, 17) * 6;
        self.week = getbitu(buf, 60, 10) + 2048;
        self.code = getbitu(buf, 70, 2);
        self.sva = getbitu(buf, 72, 4);
        self.svh = getbitu(buf, 76, 6);

        self.iodc = getbitu2(buf, 82, 2, 210, 8);
        self.flag = getbitu(buf, 90, 1);
        self.tgd = getbits(buf, 196, 8) as f64 * P2_31;
        self.toc = getbitu(buf, 218, 16) * 16;
        self.f2 = getbits(buf, 240, 8) as f64 * P2_55;
        self.f1 = getbits(buf, 248, 16) as f64 * P2_43;
        self.f0 = getbits(buf, 270, 22) as f64 * P2_31;

        log::warn!(
            "{sv}: {} tow={} week={} sva={} svh={} iodc={} tgd={:+e} toc={} a0={:+e} a1={:+e} a2={:+e}",
            "subframe-1".to_string().blue(),
            self.tow,
            self.week,
            self.sva,
            self.svh,
            self.iodc,
            self.tgd,
            self.toc,
            self.f0,
            self.f1,
            self.f2
        );
    }

    pub fn nav_decode_lnav_subframe2(&mut self, buf: &[u8], sv: SV) {
        self.tow = getbitu(buf, 30, 17) * 6;
        self.iode = getbitu(buf, 60, 8);
        self.crs = getbits(buf, 68, 16) as f64 * P2_5;
        self.deln = getbits(buf, 90, 16) as f64 * P2_43 * SC2RAD;
        self.m0 = getbits2(buf, 106, 8, 120, 24) as f64 * P2_31 * SC2RAD;
        self.ecc = getbitu2(buf, 166, 8, 180, 24) as f64 * P2_33;
        self.cuc = getbits(buf, 150, 16) as f64 * P2_29;
        self.cus = getbits(buf, 210, 16) as f64 * P2_29;
        let sqrt_a = getbitu2(buf, 226, 8, 240, 24) as f64 * P2_19;
        self.toe = getbitu(buf, 270, 16) * 16;
        self.fit = getbitu(buf, 286, 1);
        self.a = sqrt_a * sqrt_a;

        log::warn!(
            "{sv}: {} tow={} a={} iode={} crs={} crc={} cuc={:+e} cus={:+e} ecc={} m0={} toe={}",
            "subframe-2".to_string().blue(),
            self.tow,
            self.a,
            self.iode,
            self.crs,
            self.crc,
            self.cuc,
            self.cus,
            self.ecc,
            self.m0,
            self.toe,
        );
    }

    pub fn nav_decode_lnav_subframe3(&mut self, buf: &[u8], sv: SV) {
        self.tow = getbitu(buf, 30, 17) * 6;
        self.cic = getbits(buf, 60, 16) as f64 * P2_29;
        self.cis = getbits(buf, 120, 16) as f64 * P2_29;
        self.omg0 = getbits2(buf, 76, 8, 90, 24) as f64 * P2_31 * SC2RAD;
        self.i0 = getbits2(buf, 136, 8, 150, 24) as f64 * P2_31 * SC2RAD;
        self.crc = getbits(buf, 180, 16) as f64 * P2_5;
        self.omg = getbits2(buf, 196, 8, 210, 24) as f64 * P2_31 * SC2RAD;
        self.omg_dot = getbits(buf, 240, 24) as f64 * P2_43 * SC2RAD;
        self.iode = getbitu(buf, 270, 8);
        self.i_dot = getbits(buf, 278, 14) as f64 * P2_43 * SC2RAD;

        log::warn!(
            "{sv}: {} tow={} cic={:+e} cis={:+e} omg={} omg0={} omgd={:+e} i0={} idot={:+e}",
            "subframe-3".to_string().blue(),
            self.tow,
            self.cic,
            self.cis,
            self.omg,
            self.omg0,
            self.omg_dot,
            self.i0,
            self.i_dot
        );
    }
}
