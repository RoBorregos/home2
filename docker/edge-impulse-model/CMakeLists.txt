cmake_minimum_required(VERSION 3.8)
project(ei_sdk LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Glob .cpp sources only from subdirectories confirmed to be cross-platform
file(GLOB_RECURSE EI_SOURCES
    "edge-impulse-sdk/classifier/*.cpp"
    "edge-impulse-sdk/dsp/*.cpp"
    "edge-impulse-sdk/dsp/*.c"
    "edge-impulse-sdk/porting/posix/*.c"
    "edge-impulse-sdk/porting/posix/*.cpp"
    "tflite-model/*.cpp"
)

# Exclude CMSIS explicitly (ARM specific hardware optimizations. Causes issues on x86_64)
# Exclude non-posix ports (zephyr, mbed, etc.) since we target POSIX systems.
file(GLOB_RECURSE EI_EXTRAS 
    "edge-impulse-sdk/tensorflow/*.cpp" 
    "edge-impulse-sdk/tensorflow/*.cc" 
    "edge-impulse-sdk/tensorflow/*.c"
)

add_library(ei_sdk STATIC ${EI_SOURCES} ${EI_EXTRAS})

# Suppress massive warning logs from external code
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(ei_sdk PRIVATE -w)
endif()

# Expose model-parameters and EI paths as PUBLIC include directories
target_include_directories(ei_sdk PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/model-parameters"
    "${CMAKE_CURRENT_SOURCE_DIR}/tflite-model"
    "${CMAKE_CURRENT_SOURCE_DIR}/edge-impulse-sdk"
    "${CMAKE_CURRENT_SOURCE_DIR}/edge-impulse-sdk/classifier"
    "${CMAKE_CURRENT_SOURCE_DIR}/edge-impulse-sdk/dsp"
    "${CMAKE_CURRENT_SOURCE_DIR}/edge-impulse-sdk/porting"
    "${CMAKE_CURRENT_SOURCE_DIR}/edge-impulse-sdk/third_party"
)

# Global macros needed by SDK
target_compile_definitions(ei_sdk PUBLIC EI_CLASSIFIER_SENSOR=EI_CLASSIFIER_SENSOR_MICROPHONE)
