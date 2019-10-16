set(GTEST_SHUFFLE 1)
find_package(OpenCV REQUIRED)

add_executable(
    cpp-test
    main.cpp
    test.cpp
)

target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include 
                                           ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(cpp-test PUBLIC gtest)
target_link_libraries(shell-app ${OpenCV_LIBS})
