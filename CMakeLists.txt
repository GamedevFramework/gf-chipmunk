cmake_minimum_required(VERSION 3.10)

project(GFCP
  VERSION 0.1.0
  LANGUAGES CXX
)

include(GNUInstallDirs)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(gf REQUIRED COMPONENTS graphics)
find_package(Chipmunk REQUIRED)

add_executable(example_gf_chipmunk
  example/main.cc
  gfcp/Physics.cc
)

target_compile_features(example_gf_chipmunk
  PUBLIC
    cxx_std_14
)

target_include_directories(example_gf_chipmunk
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(example_gf_chipmunk
  gf::graphics
  Chipmunk::Chipmunk
)
