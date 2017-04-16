if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

configure_file(test_contact_placing.sh.in ${CMAKE_CURRENT_BINARY_DIR}/test_contact_placing.sh)

add_executable(test_contact_placing
  test_contact_placing.cpp
  )
target_link_libraries(test_contact_placing hmc2 ${OPENHRP_LIBRARIES})