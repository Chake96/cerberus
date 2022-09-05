from conan import ConanFile
from conan.tools.cmake import CMakeDeps, cmake_layout, CMakeToolchain, CMake
from conans import tools

class CerberusPkg(ConanFile):
    settings = {
        "build_type",
    }
    requires = ["abseil/20211102.0", "gtest/cci.20210126", "boost/1.79.0", 
    "ftxui/3.0.0", "onetbb/2021.3.0", "mp-units/0.7.0", "libusb/[>=1.0.26]", 
    "spdlog/[>=1.10.0]", "libsystemd/251.4","fmt/[>=9.1.0]","cli11/2.2.0"]

    def layout(self):
        cmake_layout(self)

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables['CMAKE_EXPORT_COMPILE_COMMANDS'] = "ON"
        tc.generate()
        deps = CMakeDeps(self)
        deps.generate()

    def build(self):
        cmake = CMake(self)
        # self.run("cmake .. -DCMAKE_BUILD_TYPE=Debug") #to export compile commands for Clang-tidy/intellisense
        cmake.configure()
        cmake.build()
