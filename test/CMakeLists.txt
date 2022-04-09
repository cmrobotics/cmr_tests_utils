ament_add_gtest(node_communication_test node_communication_test.cpp)
target_link_libraries(node_communication_test cmr_tests_utils_lib)
ament_target_dependencies(node_communication_test ${dependencies})
target_include_directories(node_communication_test PRIVATE "../include" "include")
