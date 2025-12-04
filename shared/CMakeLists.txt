cmake_minimum_required(VERSION 3.8)
project(shared_defs)

# No library needed, just header-only
include(GNUInstallDirs)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
