from conan import ConanFile
from conan.tools.cmake import CMakeDeps, cmake_layout, CMakeToolchain, CMake

class CerberusPkg(ConanFile):
    settings = "os", "compiler", "build_type", "arch", "cppstd"
    requires = ["abseil/20211102.0", "gtest/cci.20210126", "boost/1.79.0", "ftxui/3.0.0", "onetbb/2021.3.0", "si/[>=2.5.0]", "libusb/[>=1.0.26]", "spdlog/[>=1.10.0]", "libsystemd/251.4","fmt/[>=9.1.0]","cli11/2.2.0"]

    def layout(self):
        cmake_layout(self)

    def generate(self):
        # toolchain = CMakeToolchain(self)
        # # toolchain.variables["tests"] = False
        # toolchain.generate()
        CMakeToolchain(self).generate()
        CMakeDeps(self).generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()


    # def generate(self):
    #     deps = CMakeDeps(self)
    #     # By default, ``deps.configuration`` will be ``self.settings.build_type``
    #     if self.options["hello"].shared:
    #         # Assuming the current project ``CMakeLists.txt`` defines the ReleasedShared configuration.
    #         deps.configuration = "ReleaseShared"
    #     deps.generate()