cmake_minimum_required(VERSION 3.11)
project(module)

add_library(${PROJECT_NAME} STATIC)

target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/feyman_mcs10/feyman_mcs10.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/wit_imu_jy901b/mod_wit_imu.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/yesense_yis_imu/yis_imu.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ws2812/ws2812_strip.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC xr)
