use crate::constants::NUM_GPS_SATS;
use crate::constants::PRN_CODE_LEN;
use rustfft::{num_complex::Complex64, FftPlanner};
use std::collections::HashMap;

pub struct Code {
    upscaled_codes_complex: Vec<Vec<Complex64>>,
    prn_code_fft_map: HashMap<usize, Vec<Complex64>>,
}

impl Code {
    pub fn new() -> Self {
        let mut s = Self {
            upscaled_codes_complex: vec![vec![]; 255],
            prn_code_fft_map: HashMap::<usize, Vec<Complex64>>::new(),
        };

        s.init();
        s
    }

    pub fn get_prn_code_upsampled_complex(&mut self, prn: u8) -> Vec<Complex64> {
        self.upscaled_codes_complex[prn as usize - 1].clone()
    }

    pub fn add_upscale_code(&mut self, prn: usize) {
        let mut fft_planner: FftPlanner<f64> = FftPlanner::new();

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

    pub fn init(&mut self) {
        for prn in 1..NUM_GPS_SATS + 1 {
            self.add_upscale_code(prn);
        }
        for prn in 120..158 + 1 {
            // SBAS
            self.add_upscale_code(prn);
        }
    }

    pub fn get_prn_code_fft(&self, prn: u8) -> Vec<Complex64> {
        let prn_usize = prn as usize;
        self.prn_code_fft_map.get(&prn_usize).unwrap().clone()
    }

    pub fn gen_code(prn: usize) -> Vec<u8> {
        const G2_DELAY: [usize; 210] = [
            5, 6, 7, 8, 17, 18, 139, 140, 141, 251, /*   1- 10 */
            252, 254, 255, 256, 257, 258, 469, 470, 471, 472, /*  11- 20 */
            473, 474, 509, 512, 513, 514, 515, 516, 859, 860, /*  21- 30 */
            861, 862, 863, 950, 947, 948, 950, 67, 103, 91, /*  31- 40 */
            19, 679, 225, 625, 946, 638, 161, 1001, 554, 280, /*  41- 50 */
            710, 709, 775, 864, 558, 220, 397, 55, 898, 759, /*  51- 60 */
            367, 299, 1018, 729, 695, 780, 801, 788, 732, 34, /*  61- 70 */
            320, 327, 389, 407, 525, 405, 221, 761, 260, 326, /*  71- 80 */
            955, 653, 699, 422, 188, 438, 959, 539, 879, 677, /*  81- 90 */
            586, 153, 792, 814, 446, 264, 1015, 278, 536, 819, /*  91-100 */
            156, 957, 159, 712, 885, 461, 248, 713, 126, 807, /* 101-110 */
            279, 122, 197, 693, 632, 771, 467, 647, 203, 145, /* 111-120 */
            175, 52, 21, 237, 235, 886, 657, 634, 762, 355, /* 121-130 */
            1012, 176, 603, 130, 359, 595, 68, 386, 797, 456, /* 131-140 */
            499, 883, 307, 127, 211, 121, 118, 163, 628, 853, /* 141-150 */
            484, 289, 811, 202, 1021, 463, 568, 904, 670, 230, /* 151-160 */
            911, 684, 309, 644, 932, 12, 314, 891, 212, 185, /* 161-170 */
            675, 503, 150, 395, 345, 846, 798, 992, 357, 995, /* 171-180 */
            877, 112, 144, 476, 193, 109, 445, 291, 87, 399, /* 181-190 */
            292, 901, 339, 208, 711, 189, 263, 537, 663, 942, /* 191-200 */
            173, 900, 30, 500, 935, 556, 373, 85, 652, 310, /* 201-210 */
        ];
        let mut g1 = [0i8; PRN_CODE_LEN];
        let mut g2 = [0i8; PRN_CODE_LEN];
        let mut r1 = [-1i8; 10];
        let mut r2 = [-1i8; 10];
        let mut g = vec![];
        for i in 0..PRN_CODE_LEN {
            g1[i] = r1[9];
            g2[i] = r2[9];
            let c1 = r1[2] * r1[9];
            let c2 = r2[1] * r2[2] * r2[5] * r2[7] * r2[8] * r2[9];
            r1.rotate_right(1);
            r2.rotate_right(1);
            r1[0] = c1;
            r2[0] = c2;
        }
        let mut j = PRN_CODE_LEN - G2_DELAY[prn - 1];
        for i in 0..PRN_CODE_LEN {
            let v = -g1[i] * g2[j % PRN_CODE_LEN];
            g.push(if v >= 0 { 1 } else { 0 });
            j += 1;
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
