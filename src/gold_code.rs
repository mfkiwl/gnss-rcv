use crate::constants::NUM_GPS_SATS;
use crate::constants::PRN_CODE_LEN;
use rustfft::{num_complex::Complex64, FftPlanner};
use std::collections::HashMap;

const G1_TAP: [usize; 2] = [2, 9];
const G2_TAP: [usize; 6] = [1, 2, 5, 7, 8, 9];
const PRN_TO_G2_TAP: [(usize, usize); NUM_GPS_SATS] = [
    (2, 6),
    (3, 7),
    (4, 8),
    (5, 9),
    (1, 9),
    (2, 10),
    (1, 8),
    (2, 9),
    (3, 10),
    (2, 3),
    (3, 4),
    (5, 6),
    (6, 7),
    (7, 8),
    (8, 9),
    (9, 10),
    (1, 4),
    (2, 5),
    (3, 6),
    (4, 7),
    (5, 8),
    (6, 9),
    (1, 3),
    (4, 6),
    (5, 7),
    (6, 8),
    (7, 9),
    (8, 10),
    (1, 6),
    (2, 7),
    (3, 8),
    (4, 9),
];

pub struct GoldCode {
    upscaled_codes_complex: Vec<Vec<Complex64>>,
    prn_code_fft_map: HashMap<usize, Vec<Complex64>>,
}

impl GoldCode {
    pub fn new() -> Self {
        let mut s = Self {
            upscaled_codes_complex: vec![vec![]; NUM_GPS_SATS],
            prn_code_fft_map: HashMap::<usize, Vec<Complex64>>::new(),
        };

        s.init();
        s
    }

    pub fn init(&mut self) {
        let mut fft_planner: FftPlanner<f64> = FftPlanner::new();

        for prn in 1..NUM_GPS_SATS + 1 {
            let one_prn_code = Self::gen_code(prn);
            let mut prn_code_upsampled: Vec<_> = one_prn_code
                .iter()
                .map(|&x| Complex64 {
                    re: if x == 0 { -1.0 } else { 1.0 },
                    im: 0.0,
                })
                .flat_map(|x| [x, x])
                .collect();
            assert_eq!(prn_code_upsampled.len(), PRN_CODE_LEN * 2);

            self.upscaled_codes_complex[prn - 1] = prn_code_upsampled.clone();

            let fft_fw = fft_planner.plan_fft_forward(prn_code_upsampled.len());
            fft_fw.process(&mut prn_code_upsampled);

            self.prn_code_fft_map.insert(prn, prn_code_upsampled); // fft done
        }
    }

    pub fn get_fft_code(&self, prn: usize) -> Vec<Complex64> {
        self.prn_code_fft_map.get(&prn).unwrap().clone()
    }

    fn gen_code(prn: usize) -> Vec<u8> {
        let mut g1 = [1u8; 10];
        let mut g2 = [1u8; 10];
        let mut g = vec![];

        for _i in 0..PRN_CODE_LEN {
            let p = PRN_TO_G2_TAP.get(prn - 1).unwrap();
            let v = (g1[9] + g2.get(p.0 - 1).unwrap() + g2.get(p.1 - 1).unwrap()) % 2;
            g.push(v);

            let v = G1_TAP.iter().map(|&x| g1[x]).sum::<u8>() % 2;
            g1[9] = v;
            g1.rotate_right(1);

            let v = G2_TAP.iter().map(|&x| g2[x]).sum::<u8>() % 2;
            g2[9] = v;
            g2.rotate_right(1);
        }
        g
    }

    pub fn print_gold_codes() {
        println!("generating gold codes");
        for i in 1..NUM_GPS_SATS + 1 {
            let g = Self::gen_code(i);
            println!("  code-{:02}: {:?}", i, &g[0..20]);
        }
    }
}
