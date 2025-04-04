use egui_extras::{Column, TableBuilder};
use egui_extras::{Size, StripBuilder};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::thread;

use gnss_rs::constellation::Constellation;
use gnss_rs::sv::SV;

use crate::channel::State;
use crate::receiver::Receiver;
use crate::recording::IQFileType;
use crate::state::GnssState;

const WIDTH: usize = 800;
const HEIGHT: usize = 600;

pub struct GnssRcvApp {
    iq_file: String,
    iq_file_type: IQFileType,
    iq_file_choice: usize,
    iq_type_choice: usize,
    needs_stop: Arc<AtomicBool>,
    active: Arc<AtomicBool>,
    pub_state: Arc<Mutex<GnssState>>,
}

impl Default for GnssRcvApp {
    fn default() -> Self {
        Self {
            iq_file: "resources/nov_3_time_18_48_st_ives".to_owned(),
            iq_file_type: IQFileType::TypePairFloat32,
            iq_file_choice: 0,
            iq_type_choice: 0,
            active: Arc::new(AtomicBool::new(false)),
            needs_stop: Arc::new(AtomicBool::new(false)),
            pub_state: Arc::new(Mutex::new(GnssState::new())),
        }
    }
}

fn async_receive(
    active: Arc<AtomicBool>,
    needs_stop: Arc<AtomicBool>,
    file: PathBuf,
    iq_file_type: IQFileType,
    pub_state: Arc<Mutex<GnssState>>,
) {
    log::info!("start_receiving");

    active.store(true, Ordering::SeqCst);

    let mut receiver = Receiver::new(
        false,
        "",
        &file,
        &iq_file_type,
        2046000.0,
        0.0,
        0,
        "L1CA",
        "",
        needs_stop.clone(),
        pub_state,
    );

    log::info!("run_loop");

    receiver.run_loop(0);

    active.store(false, Ordering::SeqCst);
    log::info!("start_receiving: done");
}

impl GnssRcvApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Default::default()
    }

    fn stop_async(&mut self) {
        self.needs_stop.store(true, Ordering::SeqCst);
        log::info!("stop_async");
    }

    fn start_async(&mut self, ctx: &egui::Context) {
        log::info!("start_async");
        self.needs_stop.store(false, Ordering::SeqCst);

        let active = self.active.clone();
        let needs_stop = self.needs_stop.clone();
        let iq_file = self.iq_file.clone();
        let iq_file_type = self.iq_file_type.clone();
        let pub_state = self.pub_state.clone();
        let ctx_clone = ctx.clone();

        let update_func = move || {
            ctx_clone.request_repaint();
            log::info!("repaint requested");
        };

        self.pub_state
            .lock()
            .unwrap()
            .set_update_func(Box::new(update_func.clone()));

        thread::spawn(move || {
            log::info!("thread_start");
            async_receive(active, needs_stop, iq_file.into(), iq_file_type, pub_state);
            log::info!("thread_stop");
        });
    }
}

pub fn egui_main() {
    log::warn!("egui_main");
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([WIDTH as f32, HEIGHT as f32]),
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
            "nov_3_time_18_48_st_ives",
            "gpssim_2xi16",
            "L1_20211202_084700_4MHz_IQ.bin",
            "GPS-L1-2022-03-27.sigmf-data",
        ];
        let type_str = ["2xf32", "2xi16"];

        egui::TopBottomPanel::top("top_panel")
            .resizable(false)
            .min_height(25.0)
            .show(ctx, |ui| {
                egui::Grid::new("TopGrid").show(ui, |ui| {
                    egui::ComboBox::from_label("Pick file")
                        .width(230.0)
                        .selected_text(vec_str[self.iq_file_choice])
                        .show_ui(ui, |ui| {
                            for (i, s) in vec_str.iter().enumerate() {
                                let value =
                                    ui.selectable_value(&mut self.iq_file_choice, i, s.to_string());
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
                            .selected_text(type_str[self.iq_type_choice])
                            .show_ui(ui, |ui| {
                                for (i, s) in type_str.iter().enumerate() {
                                    let value = ui.selectable_value(
                                        &mut self.iq_type_choice,
                                        i,
                                        s.to_string(),
                                    );
                                    if value.clicked() {
                                        self.iq_type_choice = i;
                                    }
                                }
                            });
                    });
                    ui.end_row();
                    let button_text = if self.active.load(Ordering::SeqCst) {
                        "stop"
                    } else {
                        "start"
                    };
                    if ui
                        .add_sized([80.0, 25.], egui::Button::new(button_text.to_owned()))
                        .clicked()
                    {
                        if self.active.load(Ordering::SeqCst) {
                            self.stop_async();
                        } else {
                            self.start_async(ctx);
                        }
                    }
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

        egui::CentralPanel::default().show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                StripBuilder::new(ui)
                    .size(Size::remainder().at_least(100.0)) // for the table
                    .vertical(|mut strip| {
                        strip.cell(|ui| {
                            egui::ScrollArea::horizontal().show(ui, |ui| {
                                self.table_ui(ui);
                            });
                        });
                    });
            });
        });
    }
}

impl GnssRcvApp {
    fn table_ui(&mut self, ui: &mut egui::Ui) {
        let available_height = ui.available_height();
        let table = TableBuilder::new(ui)
            .resizable(true)
            .striped(true)
            .cell_layout(egui::Layout::left_to_right(egui::Align::Center))
            .column(Column::auto())
            .column(Column::auto().at_least(30.0).resizable(true))
            .column(Column::auto())
            .column(Column::remainder())
            .column(Column::remainder())
            .min_scrolled_height(0.0)
            .max_scroll_height(available_height);

        table
            .header(20.0, |mut header| {
                header.col(|ui| {
                    ui.strong("PRN");
                });
                header.col(|ui| {
                    ui.strong("dB-Hz");
                });
                header.col(|ui| {
                    ui.strong("state");
                });
                header.col(|ui| {
                    ui.strong("almanac");
                });
                header.col(|ui| {
                    ui.strong("detail-2");
                });
            })
            .body(|mut body| {
                for row_index in 1..=32 {
                    let row_height = 20.0;
                    let sv = SV::new(Constellation::GPS, row_index);
                    let pub_state = self.pub_state.lock().unwrap();
                    let channel = pub_state.channels.get(&sv);

                    if channel.is_none() {
                        continue;
                    }
                    let state = channel.unwrap().state.clone();
                    if state != State::Tracking {
                        continue;
                    }
                    let cn0 = channel.unwrap().cn0;

                    body.row(row_height, |mut row| {
                        row.col(|ui| {
                            ui.label(format!("sv-{}", row_index).to_string());
                        });
                        row.col(|ui| {
                            ui.label(format!("{:.1}", cn0).to_string());
                        });
                        row.col(|ui| {
                            ui.label("TRK");
                        });
                        row.col(|ui| {
                            ui.label(".-_-_-.");
                        });
                        row.col(|ui| {
                            ui.label("oOoOo");
                        });
                    });
                }
            });
    }
}
