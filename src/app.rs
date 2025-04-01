use log::info;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;

const WIDTH: usize = 600;
const HEIGHT: usize = 600;
const SIDE_PANEL_WIDTH: usize = 250;

pub struct GnssRcvApp {
    iq_file: String,
    needs_stop: Arc<AtomicBool>,
}

impl Default for GnssRcvApp {
    fn default() -> Self {
        Self {
            iq_file: "resources/nov_3_time_18_48_st_ives".to_owned(),
            needs_stop: Arc::new(AtomicBool::new(false)),
        }
    }
}

impl GnssRcvApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Default::default()
    }

    fn stop_async(&mut self) {
        self.needs_stop.store(true, Ordering::SeqCst);
        info!("stop_async");
    }

    fn start_async(&mut self, _ctx: &egui::Context) {
        info!("start_async");
    }
}

pub fn egui_main() {
    info!("egui_main");
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([(SIDE_PANEL_WIDTH + WIDTH + 50) as f32, (HEIGHT + 50) as f32]),
        ..eframe::NativeOptions::default()
    };
    eframe::run_native(
        "gnss-rcv",
        native_options,
        Box::new(|cc| Ok(Box::new(GnssRcvApp::new(cc)))),
    )
    .unwrap();
}

impl eframe::App for GnssRcvApp {
    fn update(&mut self, _ctx: &egui::Context, _frame: &mut eframe::Frame) {}
}
