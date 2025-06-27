from email.policy import default
from conan import ConanFile
from conan.tools.cmake import cmake_layout


class ExampleRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain", "cmake_find_package", "cmake"
    
    def configure(self):
        # osqp
        self.options["osqp"].shared = True
        self.options["libunwind"].shared = True
        # ceres
        self.options["ceres-solver"].shared = True
        self.options["ceres-solver"].use_glog = True
        self.options["libunwind"].shared = True
        self.options["glog"].shared = True
        self.options["glog"].with_threads = True
        self.options["gflags"].shared = True
        # opencv
        self.options["opencv"].shared = True
        self.options["jasper"].shared = True
        self.options["openexr"].shared = True
        self.options["libdeflate"].shared = True
        self.options["libwebp"].shared = True

    def requirements(self):
        self.requires("osqp/0.6.3")
        self.requires("opencv/3.4.20")
        self.requires("ceres-solver/1.14.0")

    def layout(self):
        cmake_layout(self)
