find_package(OpenCV REQUIRED)
add_executable(shell-app main.cpp Map.cpp AckermanKinematicModel.cpp Pid.cpp)
target_link_libraries(shell-app ${OpenCV_LIBS})
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
