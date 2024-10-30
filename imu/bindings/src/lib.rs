use pyo3::prelude::*;
use pyo3_stub_gen::define_stub_info_gatherer;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pymethods};
use hexmove::{start_imu_thread as hexmove_start_imu_thread, get_imu_data as hexmove_get_imu_data, ImuData as HexmoveImuData};
use std::sync::{Arc, Mutex};

#[gen_stub_pyclass]
#[pyclass]
struct PyHexmoveIMU {
    imu_data: Arc<Mutex<HexmoveImuData>>,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyHexmoveIMU {
    #[new]
    fn new() -> PyResult<Self> {
        hexmove_start_imu_thread().map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        let imu_data = Arc::new(Mutex::new(HexmoveImuData::new()));
        Ok(PyHexmoveIMU { imu_data })
    }

    fn get_imu_data(&self) -> PyResult<PyHexmoveImuData> {
        hexmove_get_imu_data().map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string())).map(|data| data.into())
    }

    fn __repr__(&self) -> PyResult<String> {
        Ok(format!("PyHexmoveIMU"))
    }
}

#[gen_stub_pyclass]
#[pyclass]
#[derive(Clone)]
struct PyHexmoveImuData {
    #[pyo3(get)]
    angle_x: f32,
    #[pyo3(get)]
    angle_y: f32,
    #[pyo3(get)]
    angle_z: f32,
    #[pyo3(get)]
    timestamp: u16,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyHexmoveImuData {
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!(
            "ImuData(angle_x={:.2}, angle_y={:.2}, angle_z={:.2}, timestamp={})",
            self.angle_x, self.angle_y, self.angle_z, self.timestamp
        ))
    }
}

impl From<HexmoveImuData> for PyHexmoveImuData {
    fn from(data: HexmoveImuData) -> Self {
        PyHexmoveImuData { angle_x: data.angle_x, angle_y: data.angle_y, angle_z: data.angle_z, timestamp: data.timestamp }
    }
}

#[pymodule]
fn bindings(m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<PyHexmoveIMU>()?;
    m.add_class::<PyHexmoveImuData>()?;
    Ok(())
}

define_stub_info_gatherer!(stub_info);

