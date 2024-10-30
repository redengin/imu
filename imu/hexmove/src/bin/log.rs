use hexmove::ImuReader;
use std::thread;
use std::time::Duration;

fn main() {
    // Create a new ImuReader for the 'can0' interface
    let imu_reader = match ImuReader::new("can0", 1, 1) {
        Ok(reader) => reader,
        Err(e) => {
            eprintln!("Failed to initialize IMU reader: {}", e);
            return;
        }
    };

    // Continuously read and print IMU data
    loop {
        let data = imu_reader.get_data();
        println!(
            "Angular Position: X={}°, Y={}°, Z={}° | Angular Velocity: X={}°/s, Y={}°/s, Z={}°/s",
            data.x_angle,
            data.y_angle,
            data.z_angle,
            data.x_velocity,
            data.y_velocity,
            data.z_velocity
        );

        // Sleep for a short duration to avoid spamming the console
        thread::sleep(Duration::from_millis(500));
    }
}
