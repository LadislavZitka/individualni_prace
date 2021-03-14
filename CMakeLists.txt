LINK_LIBRARIES(raspicam)

project(Main_Project)

find_library(wiringPi_LIB wiringPi)
find_package(raspicam REQUIRED)
find_package(OpenCV REQUIRED)
add_executable(${PROJECT_NAME} "main.cpp")

target_link_libraries(${PROJECT_NAME} ${wiringPi_LIB})
IF (OpenCV_FOUND)
target_link_libraries (${PROJECT_NAME} opencv_core  opencv_imgcodecs raspicam_cv)
ENDIF()
