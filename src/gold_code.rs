const G1_TAP: [usize; 2] = [2, 9];
const G2_TAP: [usize; 6] = [1, 2, 5, 7, 8, 9];
const GOLD_CODE_LEN : usize = 1023;
const PRN_TO_G2_TAP: [(usize, usize); 32] = [
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

pub fn gen_code(prn: usize) -> Vec<usize> {
    let mut g1 = [1; 10];
    let mut g2 = [1; 10];
    let mut g = vec![];

    for _i in 0..GOLD_CODE_LEN - 1 {
        let p = PRN_TO_G2_TAP.get(prn - 1).unwrap();
        let v = (g1[9] + g2.get(p.0 - 1).unwrap() + g2.get(p.1 - 1).unwrap()) % 2;
        g.push(v);

        let v = G1_TAP.iter().map(|&x| g1[x]).sum::<usize>() % 2;
        g1[9] = v;
        g1.rotate_right(1);

        let v = G2_TAP.iter().map(|&x| g2[x]).sum::<usize>() % 2;
        g2[9] = v;
        g2.rotate_right(1);
    }
    g
}

pub fn gen_gold_codes() {
    for i in 1..32 {
        let g = gen_code(i);
        println!("  code-{:02}: {:?}", i, &g[0..16]);
    }
}
