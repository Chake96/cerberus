from conan import ConanFile
from conan.tools.cmake import CMakeDeps, cmake_layout, CMakeToolchain, CMake
from conans import tools

class CerberusPkg(ConanFile):
    settings = {
        "build_type",
    }
    requires = [
        "abseil/20211102.0", "gtest/cci.20210126", "boost/1.79.0", "ftxui/3.0.0", 
        "onetbb/2021.3.0", "si/[>=2.5.0]", "spdlog/[>=1.10.0]","libsystemd/251.4",
        "fmt/[>=9.1.0]","cli11/2.2.0", "opencv/[>=4.5.0]", "libcap/[>=2.62]",
        "libusb/[>=1.0.26]"
    ]

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables['CMAKE_EXPORT_COMPILE_COMMANDS'] = "ON"
        tc.generate()
        deps = CMakeDeps(self)
        deps.generate()

        # self.run("cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Debug")

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
