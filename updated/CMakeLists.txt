cmake_minimum_required(VERSION 3.18)
# CUDA 11.5 emits Ampere SASS and PTX. Execution on Ada was tested;
# the driver-selected code path was not measured.
# Override with e.g. -DCMAKE_CUDA_ARCHITECTURES=89 for a supporting toolkit.
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 86)
endif()
project(OpenFusionUpdated LANGUAGES CXX CUDA)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(OPENFUSION_VOLUME_SIZE 512 CACHE STRING "Voxels per axis (original: 512)")
if(NOT OPENFUSION_VOLUME_SIZE MATCHES "^(16|32|64|128|256|512|1024)$")
  message(FATAL_ERROR "Choose a power of two from 16 through 1024")
endif()
find_package(CUDAToolkit REQUIRED)
set(ORIGINAL "${CMAKE_CURRENT_SOURCE_DIR}/../source/Source")
if(NOT EXISTS "${ORIGINAL}/src/General.cu")
  message(FATAL_ERROR "Initialize the pinned source submodule: git submodule update --init")
endif()
add_library(openfusion_core STATIC
  src/General.cpp src/ICP.cpp src/ICP.cu src/VolumeIntegrator.cu
  src/Raycaster.cu src/CameraHandler.cu src/CameraAbstract.cpp
  src/CameraFromFile.cpp "${ORIGINAL}/src/General.cu")
target_include_directories(openfusion_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/compat" "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_include_directories(openfusion_core SYSTEM PUBLIC
  "${ORIGINAL}/include" "${ORIGINAL}/include/Utils")
target_compile_definitions(openfusion_core PUBLIC
  OPENFUSION_HEADLESS=1 OPENFUSION_VOLUME_SIZE=${OPENFUSION_VOLUME_SIZE})
target_compile_options(openfusion_core PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-lineinfo>)
target_link_libraries(openfusion_core PUBLIC CUDA::cudart)
option(OPENFUSION_BUILD_TOOLS "Build replay runner and GPU regression checks" ON)
if(OPENFUSION_BUILD_TOOLS)
  add_executable(openfusion_headless tools/openfusion_headless.cpp)
  target_link_libraries(openfusion_headless PRIVATE openfusion_core)
  enable_testing()
  add_test(NAME gpu_regression COMMAND openfusion_headless --self-test
    --output "${CMAKE_CURRENT_BINARY_DIR}/test-output")
  set_tests_properties(gpu_regression PROPERTIES TIMEOUT 180)
  add_test(NAME gpu_demo COMMAND openfusion_headless --demo
    --output "${CMAKE_CURRENT_BINARY_DIR}/demo-output")
  set_tests_properties(gpu_demo PROPERTIES TIMEOUT 180 FIXTURES_SETUP recording)
  add_test(NAME gpu_file_replay COMMAND openfusion_headless
    --recording "${CMAKE_CURRENT_BINARY_DIR}/demo-output/demo/recording"
    --frames 12 --output "${CMAKE_CURRENT_BINARY_DIR}/replay-output")
  set_tests_properties(gpu_file_replay PROPERTIES TIMEOUT 180
    FIXTURES_REQUIRED recording DEPENDS gpu_demo)
endif()
