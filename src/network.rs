use rustfft::num_complex::Complex64;
use std::collections::VecDeque;
use std::io::Read;
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;

pub struct RtlSdrTcp {
    iq_deque: Arc<Mutex<VecDeque<Vec<Complex64>>>>,
    num_samples_total: Arc<Mutex<usize>>,
    num_samples: Arc<Mutex<usize>>,
    num_sleep: u64,
}

impl Drop for RtlSdrTcp {
    fn drop(&mut self) {
        log::warn!(
            "rtlsdrtcp: stopping read. num_samples={}",
            self.num_samples.lock().unwrap()
        );
    }
}

impl RtlSdrTcp {
    pub fn new(hostname: &String) -> std::io::Result<RtlSdrTcp> {
        let mut socket = TcpStream::connect(hostname.clone())?;

        let m = RtlSdrTcp {
            iq_deque: Arc::new(Mutex::new(VecDeque::new())),
            num_samples_total: Arc::new(Mutex::new(0)),
            num_samples: Arc::new(Mutex::new(0)),
            num_sleep: 0,
        };

        let iq_deq = m.iq_deque.clone();
        let num_samples_total = m.num_samples_total.clone();
        let num_samples = m.num_samples.clone();

        thread::spawn(move || loop {
            let mut data = [0u8; 2036 * 2];
            let mut v = vec![Complex64::default(); data.len()];
            socket.read_exact(&mut data).expect("read failure");

            for i in 0..data.len() / 2 {
                let re = data[2 * i + 0] as f64 - 126.0 / 128.0;
                let im = data[2 * i + 1] as f64 - 126.0 / 128.0;
                v[i] = Complex64 { re, im };
            }

            let n = v.len();
            iq_deq.lock().unwrap().push_back(v);
            *num_samples.lock().unwrap() += n;
            *num_samples_total.lock().unwrap() += n;
        });

        Ok(m)
    }

    pub fn read_iq_data(
        &mut self,
        num_samples: usize,
    ) -> Result<Vec<Complex64>, Box<dyn std::error::Error>> {
        loop {
            if *self.num_samples.lock().unwrap() >= num_samples {
                break;
            }
            thread::sleep(std::time::Duration::from_millis(1));
            self.num_sleep += 1;
        }
        let mut vec = vec![];
        let mut iq_deq = self.iq_deque.lock().unwrap();

        let v_front = iq_deq.front_mut().unwrap();
        let n = usize::min(num_samples, v_front.len());
        for i in 0..n {
            vec.push(v_front[i]);
        }
        let _ = v_front.drain(0..n);
        if v_front.len() == 0 {
            let _ = iq_deq.pop_front();
        }

        if n < num_samples {
            let v_front = iq_deq.front_mut().unwrap();
            for i in n..num_samples {
                vec.push(v_front[i - n]);
            }
            let _ = v_front.drain(0..num_samples - n);
        }

        *self.num_samples.lock().unwrap() -= num_samples;

        Ok(vec)
    }
}
