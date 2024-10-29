use pyo3::prelude::*;
use hexmove::{
    ImuData as HexmoveImuData,
    start_imu_thread as start_hexmove_imu_thread,
    get_imu_data as get_hexmove_imu_data,
};

#[pymodule]
fn imu_module(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<HexmoveImuData>()?;
    m.add_function(wrap_pyfunction!(start_hexmove_imu_thread_py, m)?)?;
    m.add_function(wrap_pyfunction!(get_hexmove_imu_data_py, m)?)?;
    Ok(())
}

#[pyfunction]
fn start_hexmove_imu_thread_py() -> PyResult<()> {
    start_hexmove_imu_thread().map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e))
}

#[pyfunction]
fn get_hexmove_imu_data_py() -> PyResult<HexmoveImuData> {
    get_hexmove_imu_data().map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e))
}
