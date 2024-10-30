"""Example usage of the Hexmove IMU."""

import time

from imu import HexmoveImuReader


def main() -> None:
    # Initialize the IMU reader for the 'can0' interface and imu with serial number 1 and model 1
    try:
        imu_reader = HexmoveImuReader("can0", 1, 1)
    except Exception as e:
        print(f"Failed to initialize IMU reader: {e}")
        return

    try:
        while True:
            # Get the current IMU data
            data = imu_reader.get_data()
            print(
                f"Angular Position: X={data.x_angle}°, Y={data.y_angle}°, Z={data.z_angle}° | "
                f"Angular Velocity: X={data.x_velocity}°/s, Y={data.y_velocity}°/s, Z={data.z_velocity}°/s"
            )

            # Sleep for a short duration to avoid spamming the console
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping IMU reader...")
    finally:
        imu_reader.stop()


if __name__ == "__main__":
    main()
