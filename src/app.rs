use log::info;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;

const WIDTH: usize = 600;
const HEIGHT: usize = 600;
const SIDE_PANEL_WIDTH: usize = 250;

pub struct GnssRcvApp {
    iq_file: String,
    iq_file_choice: usize,
    iq_type_choice: usize,
    needs_stop: Arc<AtomicBool>,
}

impl Default for GnssRcvApp {
    fn default() -> Self {
        Self {
            iq_file: "resources/nov_3_time_18_48_st_ives".to_owned(),
            iq_file_choice: 0,
            iq_type_choice: 0,
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
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let vec_str = [
            "nov_3_time_18_48_st_ives".to_owned(),
            "gpssim_2xi16".to_owned(),
            "L1_20211202_084700_4MHz_IQ.bin".to_owned(),
            "GPS-L1-2022-03-27.sigmf-data".to_owned(),
        ];
        let type_str = [
           "2xf32".to_owned(),
           "2xi16".to_owned(),
        ];
        egui::TopBottomPanel::top("top_panel")
            .resizable(false)
            .min_height(25.0)
            .show(ctx, |ui| {
                egui::Grid::new("TopGrid").show(ui, |ui| {
                    egui::ComboBox::from_label("Pick file")
                        .width(230.0)
                        .selected_text(vec_str[self.iq_file_choice].clone())
                        .show_ui(ui, |ui| {
                            for (i, s) in vec_str.iter().enumerate() {
                                let value = ui.selectable_value(&mut self.iq_file_choice, i, s);
                                if value.clicked() {
                                    self.iq_file_choice = i;
                                    self.iq_file = format!("resources/{}", vec_str[i]);
                                }
                            }
                        });
                    ui.horizontal(|ui| {
                        ui.add(
                            egui::TextEdit::singleline(&mut self.iq_file)
                                .desired_width(f32::INFINITY)
                                .clip_text(false),
                        );
                    });
                    ui.horizontal(|ui| {
                        egui::ComboBox::from_label("type")
                        .width(30.0)
                        .selected_text(type_str[self.iq_type_choice].clone())
                        .show_ui(ui, |ui| {
                            for (i, s) in type_str.iter().enumerate() {
                                let value = ui.selectable_value(&mut self.iq_type_choice, i, s);
                                if value.clicked() {
                                    self.iq_type_choice = i;
                                }
                            }
                        });
                    });
                    ui.end_row();
                });
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.heading("PRN info");
            });
        });

        egui::TopBottomPanel::bottom("bottom_panel")
            .resizable(true)
            .min_height(150.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    ui.heading("log file here");
                });
            });
    }
}
