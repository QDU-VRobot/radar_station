cmake_minimum_required(VERSION 3.18)

project(trtyolo CXX CUDA)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()
option(BUILD_SHARED_LIBS "build shared libraries" ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(CUDA REQUIRED)
find_package(TensorRT REQUIRED)

set(CMAKE_CUDA_FLAGS -use_fast_math)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE ${PROJECT_NAME}_DEPLOY
    ${CMAKE_CURRENT_SOURCE_DIR}/trtyolo/core/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/trtyolo/utils/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/trtyolo/infer/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/trtyolo/infer/*.cu
)
file(GLOB_RECURSE ${PROJECT_NAME}_PLUGIN
    ${CMAKE_CURRENT_SOURCE_DIR}/plugin/common/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugin/common/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/plugin/efficientRotatedNMSPlugin/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugin/efficientRotatedNMSPlugin/*.cu
    ${CMAKE_CURRENT_SOURCE_DIR}/plugin/efficientIdxNMSPlugin/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plugin/efficientIdxNMSPlugin/*.cu
)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_NAME}_SOURCE)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCE} ${${PROJECT_NAME}_DEPLOY} ${${PROJECT_NAME}_PLUGIN})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/trtyolo ${CMAKE_CURRENT_SOURCE_DIR}/plugin ${CUDA_INCLUDE_DIRS} ${TensorRT_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES} ${TensorRT_LIBS})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_definitions(${PROJECT_NAME} PRIVATE -D_FORTIFY_SOURCE=2)
    target_compile_options(${PROJECT_NAME} PRIVATE -fstack-protector-strong -fstack-clash-protection)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(${PROJECT_NAME} PRIVATE -D_GLIBCXX_ASSERTIONS -D_FORTIFY_SOURCE=1)
    target_compile_options(${PROJECT_NAME} PRIVATE -fanalyzer -Wextra -Wpedantic)
endif()
