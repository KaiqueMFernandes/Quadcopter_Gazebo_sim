#add_executable(test test.cpp)
#target_link_libraries(test PRIVATE physics_engine)

# Inside test/CMakeLists.txt
add_executable(physics_unit_test test.cpp)

# Link to the library we built in the root
target_link_libraries(physics_unit_test robot_physics_lib Eigen3::Eigen)

# If you want the test output to go to a specific 'bin' or 'test' folder:
# set_target_properties(physics_unit_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")