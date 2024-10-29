use pyo3::prelude::*;
use pyo3_stub_gen::define_stub_info_gatherer;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pymethods};


#[pymodule]
fn bindings(m: &Bound<PyModule>) -> PyResult<()> {
    Ok(())
}

define_stub_info_gatherer!(stub_info);
