#![no_std]
#[cfg(feature="std")]
    extern crate std;

#[cfg(feature="high_precision")]
type Float = f64;
#[cfg(not(feature="high_precision"))]
type Float = f32;


// --- Basic Types ---
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Vector3 {
    pub x: Float,
    pub y: Float,
    pub z: Float,
}
impl Vector3 {
    pub fn new(x: Float, y: Float, z: Float) -> Self {
        Self { x, y, z }
    }
}
#[cfg(feature="std")]
impl fmt::Display for Vector3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Vector3(x={}, y={}, z={})", self.x, self.y, self.z)
    }
}
// TODO provide no_std to_string()

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Quaternion {
    pub w: Float,
    pub x: Float,
    pub y: Float,
    pub z: Float,
}
#[cfg(feature="std")]
impl fmt::Display for Quaternion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Quaternion(w={}, x={}, y={}, z={})", self.w, self.x, self.y, self.z)
    }
}
// TODO provide no_std to_string()


impl Quaternion {
    // TODO use optimized math cargo
    pub fn rotate(&self, vector: Vector3) -> Vector3 {
        // Rotate a vector by a quaternion using the formula:
        // v' = q * v * q^-1
        // Where q^-1 is the conjugate since we assume unit quaternions
        let qw = self.w;
        let qx = self.x;
        let qy = self.y;
        let qz = self.z;
        let vx = vector.x;
        let vy = vector.y;
        let vz = vector.z;

        // Calculate rotation using quaternion multiplication
        let x = (1.0 - 2.0 * qy * qy - 2.0 * qz * qz) * vx
            + (2.0 * qx * qy - 2.0 * qz * qw) * vy
            + (2.0 * qx * qz + 2.0 * qy * qw) * vz;
        let y = (2.0 * qx * qy + 2.0 * qz * qw) * vx
            + (1.0 - 2.0 * qx * qx - 2.0 * qz * qz) * vy
            + (2.0 * qy * qz - 2.0 * qx * qw) * vz;
        let z = (2.0 * qx * qz - 2.0 * qy * qw) * vx
            + (2.0 * qy * qz + 2.0 * qx * qw) * vy
            + (1.0 - 2.0 * qx * qx - 2.0 * qy * qy) * vz;

        Vector3 { x, y, z }
    }
}


// --- Standard IMU Data ---
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuData {
    /// Acceleration including gravity (m/s²)
    pub accelerometer: Option<Vector3>,
    /// Angular velocity (deg/s)
    pub gyroscope: Option<Vector3>,
    /// Magnetic field vector (micro Tesla, µT)
    pub magnetometer: Option<Vector3>,
    /// Temperature (°C)
    pub temperature: Option<Float>,
// FIXME enable these if they are useful
    // /// Orientation as a unit quaternion (WXYZ order)
    // pub orientation: Option<Quaternion>,
    // /// Orientation as Euler angles (deg)
    // pub euler: Option<Vector3>,
    // /// Linear acceleration (acceleration without gravity) (m/s²)
    // pub linear_acceleration: Option<Vector3>,
    // /// Estimated gravity vector (m/s²)
    // pub gravity: Option<Vector3>,
    // /// Calibration status
    // pub calibration_status: Option<u8>,
}

pub trait ImuReader {
    /// Retrieves the latest available IMU data.
    /// assumes IMU driver has logged the error reason
    fn get_data(&self) -> Option<ImuData>;

    /// place the IMU into sensing mode
    /// ImuReader initialization should leave device in low-power mode
    fn start(&self);

    /// place the IMU into low-power more (disable sensing)
    fn stop(&self);
}

#[derive(Debug, Clone, Copy)]
pub enum ImuFrequency {
    Hz0_2,  // 0.2 Hz
    Hz0_5,  // 0.5 Hz
    Hz1,    // 1 Hz
    Hz2,    // 2 Hz
    Hz5,    // 5 Hz
    Hz10,   // 10 Hz
    Hz20,   // 20 Hz
    Hz50,   // 50 Hz
    Hz100,  // 100 Hz
    Hz200,  // 200 Hz
    Single, // Single reading
    None,   // No readings
}
