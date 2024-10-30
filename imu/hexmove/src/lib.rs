use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Duration;
use log::{info, error};
use socketcan::{CanDataFrame, CanFrame, CanSocket, EmbeddedFrame, ExtendedId, Id, Socket};

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
    socket: CanSocket,
    data: Arc<RwLock<ImuData>>,
    running: Arc<RwLock<bool>>,
}

impl ImuReader {
    pub fn new(interface: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let socket = CanSocket::open(interface)?;
        let data = Arc::new(RwLock::new(ImuData::default()));
        let running = Arc::new(RwLock::new(true));

        let imu_reader = ImuReader {
            socket,
            data: Arc::clone(&data),
            running: Arc::clone(&running),
        };

        imu_reader.start_reading_thread();

        Ok(imu_reader)
    }

    fn start_reading_thread(&self) {
        let socket = self.socket;
        let data = Arc::clone(&self.data);
        let running = Arc::clone(&self.running);

        thread::spawn(move || {
            while *running.read().unwrap() {
                match socket.read_frame() {
                    Ok(CanFrame::Data(data_frame)) => {
                        let received_data = data_frame.data();
                        let mut data = data.write().unwrap();
                        match data_frame.id() {
                            ExtendedId::new(0x0B0101B1) => {
                                data.x_angle = i16::from_le_bytes([received_data[0], received_data[1]]) as f32 * 0.01;
                                data.y_angle = i16::from_le_bytes([received_data[2], received_data[3]]) as f32 * 0.01;
                                data.z_angle = i16::from_le_bytes([received_data[4], received_data[5]]) as f32 * 0.01;
                                info!("Angular Position: X={}°, Y={}°, Z={}°", data.x_angle, data.y_angle, data.z_angle);
                            }
                            ExtendedId::new(0x0B0101B2).unwrap() => {
                                data.x_velocity = i16::from_le_bytes([received_data[0], received_data[1]]) as f32 * 0.01;
                                data.y_velocity = i16::from_le_bytes([received_data[2], received_data[3]]) as f32 * 0.01;
                                data.z_velocity = i16::from_le_bytes([received_data[4], received_data[5]]) as f32 * 0.01;
                                info!("Angular Velocity: X={}°/s, Y={}°/s, Z={}°/s", data.x_velocity, data.y_velocity, data.z_velocity);
                            }
                            _ => {}
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
