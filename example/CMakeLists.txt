add_executable(test_exe test.cpp)
target_link_libraries(test_exe PRIVATE correct openldacs)
target_include_directories(test_exe PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(wave waveform.cpp)
target_link_libraries(wave PRIVATE correct openldacs)
target_include_directories(wave PRIVATE ${CMAKE_SOURCE_DIR}/include)
