cmake_minimum_required(VERSION 3.11)
project(managers)

add_library(${PROJECT_NAME} STATIC)

target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/imu_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vqf.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ws2812_manager.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC module xr)
