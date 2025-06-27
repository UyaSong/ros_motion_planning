set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}")
find_package(Python3 COMPONENTS Development NumPy)

catkin_add_executable_with_gtest(test_savitzky_golay_filter test_savitzky_golay_filter.cpp)
if(TARGET test_savitzky_golay_filter)
  target_link_libraries(test_savitzky_golay_filter
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    Python3::Python
    Python3::NumPy
    path_processor
  )
endif()
