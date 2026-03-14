add_executable(test_fl_exe test_fl.cpp)
target_link_libraries(test_fl_exe PRIVATE correct openldacs)
target_include_directories(test_fl_exe PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(test_rl_exe test_rl.cpp)
target_link_libraries(test_rl_exe PRIVATE correct openldacs)
target_include_directories(test_rl_exe PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(wave waveform.cpp)
target_link_libraries(wave PRIVATE correct openldacs)
target_include_directories(wave PRIVATE ${CMAKE_SOURCE_DIR}/include)
