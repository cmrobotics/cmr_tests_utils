ament_add_gtest(node_communication_test node_communication_test.cpp)
target_link_libraries(node_communication_test ${LIBRARY_NAME})
ament_target_dependencies(node_communication_test ${dependencies})
target_include_directories(node_communication_test PRIVATE "../include" "include")

ament_add_gtest(spinners_test spinners_test.cpp)
target_link_libraries(spinners_test ${LIBRARY_NAME})
ament_target_dependencies(spinners_test ${dependencies})
target_include_directories(spinners_test PRIVATE "../include" "include")
