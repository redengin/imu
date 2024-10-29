# mypy: disable-error-code="import-untyped"
#!/usr/bin/env python
"""Setup script for the project."""

import glob
import re
import subprocess

from setuptools import find_packages, setup
from setuptools.command.build_ext import build_ext
from setuptools_rust import Binding, RustExtension

with open("README.md", "r", encoding="utf-8") as f:
    long_description: str = f.read()


with open("imu/requirements.txt", "r", encoding="utf-8") as f:
    requirements: list[str] = f.read().splitlines()


with open("imu/requirements-dev.txt", "r", encoding="utf-8") as f:
    requirements_dev: list[str] = f.read().splitlines()


with open("imu/__init__.py", "r", encoding="utf-8") as fh:
    version_re = re.search(r"^__version__ = \"([^\"]*)\"", fh.read(), re.MULTILINE)
assert version_re is not None, "Could not find version in imu/__init__.py"
version: str = version_re.group(1)

package_data = [f"imu/{name}" for name in ("py.typed", "requirements.txt", "requirements-dev.txt")]
package_data.append("Cargo.toml")
for ext in ("pyi", "rs", "toml", "so"):
    package_data.extend(glob.iglob(f"imu/**/*.{ext}", recursive=True))


class RustBuildExt(build_ext):
    def run(self) -> None:
        # Run the stub generator
        subprocess.run(["cargo", "run", "--bin", "stub_gen"], check=True)
        # Call the original build_ext command
        super().run()


setup(
    name="kscale-imu",
    version=version,
    description="Python interface for interacting with IMUs",
    author="Wesley Maa",
    url="https://github.com/kscalelabs/imu",
    rust_extensions=[
        RustExtension(
            target="imu.bindings",
            path="imu/bindings/Cargo.toml",
            binding=Binding.PyO3,
        ),
    ],
    setup_requires=["setuptools-rust"],
    zip_safe=False,
    long_description=long_description,
    long_description_content_type="text/markdown",
    python_requires=">=3.11",
    install_requires=requirements,
    extras_require={"dev": requirements_dev},
    include_package_data=True,
    package_data={"kscale-imu": package_data},
    packages=find_packages(include=["imu"]),
    cmdclass={"build_ext": RustBuildExt},
)
