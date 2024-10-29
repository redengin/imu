use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;
use socketcan::{CANSocket, CANFrame};

/// Struct to hold IMU data
#[derive(Debug, Default, Clone)]
pub struct ImuData {
    pub angle_x: f32,
    pub angle_y: f32,
    pub angle_z: f32,
    pub timestamp: u16,
}

impl ImuData {
    pub fn new() -> Self {
        ImuData {
            angle_x: 0.0,
            angle_y: 0.0,
            angle_z: 0.0,
            timestamp: 0,
        }
    }
}

/// Global IMU data shared between threads
static mut IMU_DATA: Option<Arc<Mutex<ImuData>>> = None;

/// Function to construct CAN ID
fn construct_can_id(device_type: u8, model: u8, number: u8, function: u8) -> u32 {
    ((device_type as u32) << 24)
        | ((model as u32) << 16)
        | ((number as u32) << 8)
        | (function as u32)
}

/// Initialize and start the IMU thread
pub fn start_imu_thread() -> Result<(), String> {
    // Initialize shared IMU data
    let imu_data = Arc::new(Mutex::new(ImuData::new()));

    unsafe {
        IMU_DATA = Some(Arc::clone(&imu_data));
    }

    // Device identifiers (modify these as per your device)
    let device_type = 0x0B; // IMU device type
    let model = 0x01;       // Device model
    let number = 0x01;      // Device number

    // Open CAN socket
    let can_socket = CANSocket::open("can0").map_err(|e| e.to_string())?;

    // Send Angle Information Setting command to start data transmission every 10ms
    let angle_setting_can_id = construct_can_id(device_type, model, number, 0x11);
    let angle_setting_frame = CANFrame::new_extended(
        angle_setting_can_id,
        &[10], // 10ms update interval
        false,
        false,
    ).map_err(|e| e.to_string())?;
    can_socket.write_frame(&angle_setting_frame).map_err(|e| e.to_string())?;

    // Spawn a thread to read IMU data
    thread::spawn(move || {
        // Open CAN socket in the thread
        let can_socket = match CANSocket::open("can0") {
            Ok(socket) => socket,
            Err(e) => {
                eprintln!("Failed to open CAN socket: {}", e);
                return;
            }
        };

        loop {
            // Read frame
            match can_socket.read_frame() {
                Ok(frame) => {
                    // Check if the frame is from the IMU with function code 0xB1 (Sensor Angle Information)
                    let function_code = (frame.id() & 0xFF) as u8;
                    if function_code == 0xB1 {
                        // Parse data
                        let data = frame.data();
                        if data.len() >= 8 {
                            let angle_x = i16::from_le_bytes([data[0], data[1]]) as f32 * 0.01;
                            let angle_y = i16::from_le_bytes([data[2], data[3]]) as f32 * 0.01;
                            let angle_z = i16::from_le_bytes([data[4], data[5]]) as f32 * 0.01;
                            let timestamp = u16::from_le_bytes([data[6], data[7]]);

                            // Update shared IMU data
                            unsafe {
                                if let Some(ref imu_data) = IMU_DATA {
                                    let mut imu_data = imu_data.lock().unwrap();
                                    imu_data.angle_x = angle_x;
                                    imu_data.angle_y = angle_y;
                                    imu_data.timestamp = timestamp;
                                }
                            }
                        }
                    }
                },
                Err(e) => {
                    eprintln!("Error reading CAN frame: {}", e);
                    thread::sleep(Duration::from_millis(10));
                },
            }
            // Sleep briefly to prevent tight loop
            thread::sleep(Duration::from_millis(1));
        }
    });

    Ok(())
}

/// Function to get the latest IMU data
pub fn get_imu_data() -> Result<ImuData, String> {
    unsafe {
        if let Some(ref imu_data) = IMU_DATA {
            let imu_data = imu_data.lock().unwrap();
            Ok(imu_data.clone())
        } else {
            Err("IMU data not initialized".to_string())
        }
    }
}
