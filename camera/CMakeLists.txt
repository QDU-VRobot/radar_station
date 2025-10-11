cmake_minimum_required(VERSION 2.6...3.18)

project(camera CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()
option(BUILD_SHARED_LIBS "build shared libraries" ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(MVS REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_NAME}_SOURCE)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/hikcamera hikcamera_SOURCE)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCE} ${hikcamera_SOURCE})
target_include_directories(${PROJECT_NAME} PUBLIC ${MVS_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/hikcamera)
target_link_libraries(${PROJECT_NAME} ${MVS_LIBS})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_definitions(${PROJECT_NAME} PRIVATE -D_FORTIFY_SOURCE=2)
    target_compile_options(${PROJECT_NAME} PRIVATE -fstack-protector-strong -fstack-clash-protection)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(${PROJECT_NAME} PRIVATE -D_FORTIFY_SOURCE=1)
    target_compile_options(${PROJECT_NAME} PRIVATE -Og -fanalyzer -Wextra -Wpedantic)
endif()
