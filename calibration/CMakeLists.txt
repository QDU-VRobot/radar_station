cmake_minimum_required(VERSION 2.6...3.18)

project(calibration CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_NAME}_SOURCE)
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCE})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} camera)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} camera)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_definitions(${PROJECT_NAME} PRIVATE -D_FORTIFY_SOURCE=2)
    target_compile_options(${PROJECT_NAME} PRIVATE -fstack-protector-strong -fstack-clash-protection)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(${PROJECT_NAME} PRIVATE -D_GLIBCXX_ASSERTIONS -D_FORTIFY_SOURCE=1)
    target_compile_options(${PROJECT_NAME} PRIVATE -Og -fanalyzer -Wextra -Wpedantic)
endif()
