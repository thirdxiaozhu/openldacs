add_executable(rs rs.cpp)
target_link_libraries(rs PRIVATE openldacs )
target_include_directories(rs PRIVATE ${CMAKE_SOURCE_DIR}/include ${OPENLDACS_THIRDPARTY_DIR}/openldacs-correct/build/include)

add_executable(usrp_tx usrp_tx.cpp)
target_link_libraries(usrp_tx PRIVATE openldacs )
target_include_directories(rs PRIVATE ${CMAKE_SOURCE_DIR}/include ${OPENLDACS_THIRDPARTY_DIR}/openldacs-correct/build/include)
