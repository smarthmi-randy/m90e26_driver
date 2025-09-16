cmake_minimum_required(VERSION 3.21)

# Subdirectories
add_subdirectory(drivers)

# Include headers
zephyr_include_directories(include/drivers/sensor)