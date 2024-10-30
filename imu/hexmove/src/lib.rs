use log::error;
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, ExtendedId, Id, Socket};
use std::sync::{Arc, RwLock};
use std::thread;

#[derive(Debug, Default, Clone)]
pub struct ImuData {
    pub x_angle: f32,
    pub y_angle: f32,
    pub z_angle: f32,
    pub x_velocity: f32,
    pub y_velocity: f32,
    pub z_velocity: f32,
}

pub struct ImuReader {
    socket: Arc<CanSocket>,
    data: Arc<RwLock<ImuData>>,
    running: Arc<RwLock<bool>>,
}

impl ImuReader {
    pub fn new(
        interface: &str,
        serial_number: u8,
        model: u8,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let socket = Arc::new(CanSocket::open(interface)?);
        let data = Arc::new(RwLock::new(ImuData::default()));
        let running = Arc::new(RwLock::new(true));

        let imu_reader = ImuReader {
            socket: socket.clone(),
            data: Arc::clone(&data),
            running: Arc::clone(&running),
        };

        imu_reader.start_reading_thread(serial_number, model);

        Ok(imu_reader)
    }

    fn start_reading_thread(&self, serial_number: u8, model: u8) {
        let data = Arc::clone(&self.data);
        let running = Arc::clone(&self.running);
        let socket = Arc::clone(&self.socket);

        thread::spawn(move || {
            while *running.read().unwrap() {
                match socket.read_frame() {
                    Ok(CanFrame::Data(data_frame)) => {
                        let received_data = data_frame.data();
                        let id = data_frame.id();

                        let base_id =
                            0x0B000000 | (serial_number as u32) << 16 | (model as u32) << 8;

                        if id == Id::Extended(ExtendedId::new(base_id | 0xB1).unwrap()) {
                            let x_angle = i16::from_le_bytes([received_data[0], received_data[1]])
                                as f32
                                * 0.01;
                            let y_angle = i16::from_le_bytes([received_data[2], received_data[3]])
                                as f32
                                * 0.01;
                            let z_angle = i16::from_le_bytes([received_data[4], received_data[5]])
                                as f32
                                * 0.01;

                            let mut imu_data = data.write().unwrap();
                            imu_data.x_angle = x_angle;
                            imu_data.y_angle = y_angle;
                            imu_data.z_angle = z_angle;
                        }

                        if id == Id::Extended(ExtendedId::new(base_id | 0xB2).unwrap()) {
                            let x_velocity =
                                i16::from_le_bytes([received_data[0], received_data[1]]) as f32
                                    * 0.01;
                            let y_velocity =
                                i16::from_le_bytes([received_data[2], received_data[3]]) as f32
                                    * 0.01;
                            let z_velocity =
                                i16::from_le_bytes([received_data[4], received_data[5]]) as f32
                                    * 0.01;

                            let mut imu_data = data.write().unwrap();
                            imu_data.x_velocity = x_velocity;
                            imu_data.y_velocity = y_velocity;
                            imu_data.z_velocity = z_velocity;
                        }
                    }
                    Ok(CanFrame::Remote(_)) => {
                        // Ignore remote frames
                    }
                    Ok(CanFrame::Error(_)) => {
                        // Ignore error frames
                    }
                    Err(e) => {
                        error!("Error reading IMU data: {}", e);
                    }
                }
            }
        });
    }

    pub fn get_data(&self) -> ImuData {
        self.data.read().unwrap().clone()
    }

    pub fn stop(&self) {
        let mut running = self.running.write().unwrap();
        *running = false;
    }
}

impl Drop for ImuReader {
    fn drop(&mut self) {
        self.stop();
    }
}
