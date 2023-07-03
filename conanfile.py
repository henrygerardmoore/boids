# -*- coding: utf-8 -*-
from conans import ConanFile, CMake


class GameConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    requires = ("opengl/system", "sfml/2.5.1", "spdlog/1.10.0", "catch2/3.3.2")
    generators = "cmake_find_package", "cmake_paths"

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
