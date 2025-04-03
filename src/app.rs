use egui_extras::{Column, TableBuilder};
use egui_extras::{Size, StripBuilder};
use log::info;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;

const WIDTH: usize = 800;
const HEIGHT: usize = 600;

pub struct GnssRcvApp {
    iq_file: String,
    iq_file_choice: usize,
    iq_type_choice: usize,
    needs_stop: Arc<AtomicBool>,
    active: bool,
}

impl Default for GnssRcvApp {
    fn default() -> Self {
        Self {
            iq_file: "resources/nov_3_time_18_48_st_ives".to_owned(),
            iq_file_choice: 0,
            iq_type_choice: 0,
            active: false,
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

    fn start_async(&mut self) {
        info!("start_async");
    }
}

pub fn egui_main() {
    info!("egui_main");
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
                    let button_text = if self.active { "stop" } else { "start" };
                    if ui
                        .add_sized([80.0, 25.], egui::Button::new(button_text.to_owned()))
                        .clicked()
                    {
                        if self.active {
                            self.stop_async();
                            self.active = false;
                        } else {
                            self.start_async();
                            self.active = true;
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
                    ui.strong("ephemeris");
                });
                header.col(|ui| {
                    ui.strong("almanac");
                });
                header.col(|ui| {
                    ui.strong("detail-2");
                });
            })
            .body(|mut body| {
                for row_index in 0..32 {
                    let row_height = 20.0;
                    body.row(row_height, |mut row| {
                        row.col(|ui| {
                            ui.label(format!("sv-{}", row_index).to_string());
                        });
                        row.col(|ui| {
                            ui.label(format!("11"));
                        });
                        row.col(|ui| {
                            ui.label(format!("xyz"));
                        });
                        row.col(|ui| {
                            ui.label(".-_-_-.");
                        });
                        row.col(|ui| {
                            ui.label(format!("oOoOo"));
                        });
                    });
                }
            });
    }
}
