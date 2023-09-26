set(dependencies
  ${dependencies}
  "cmr_clients_utils"
  "example_interfaces"
)

find_package(cmr_clients_utils REQUIRED)

set(TEST_NAME pub_sub_communication_test)
ament_add_gtest(${TEST_NAME} ${TEST_NAME}.cpp)
target_link_libraries(${TEST_NAME} ${LIBRARY_NAME})
ament_target_dependencies(${TEST_NAME} ${dependencies})
target_include_directories(${TEST_NAME} PRIVATE "../include" "include")

set(TEST_NAME spinners_test)
ament_add_gtest(${TEST_NAME} ${TEST_NAME}.cpp)
target_link_libraries(${TEST_NAME} ${LIBRARY_NAME})
ament_target_dependencies(${TEST_NAME} ${dependencies})
target_include_directories(${TEST_NAME} PRIVATE "../include" "include")

set(TEST_NAME action_communication_test)
ament_add_gtest(${TEST_NAME} ${TEST_NAME}.cpp)
target_link_libraries(${TEST_NAME} ${LIBRARY_NAME})
ament_target_dependencies(${TEST_NAME} ${dependencies})
target_include_directories(${TEST_NAME} PRIVATE "../include" "include")

set(TEST_NAME service_communication_test)
ament_add_gtest(${TEST_NAME} ${TEST_NAME}.cpp)
target_link_libraries(${TEST_NAME} ${LIBRARY_NAME})
ament_target_dependencies(${TEST_NAME} ${dependencies})
target_include_directories(${TEST_NAME} PRIVATE "../include" "include")

set(TEST_NAME transforms_test)
ament_add_gtest(${TEST_NAME} ${TEST_NAME}.cpp)
target_link_libraries(${TEST_NAME} ${LIBRARY_NAME})
ament_target_dependencies(${TEST_NAME} ${dependencies})
target_include_directories(${TEST_NAME} PRIVATE "../include" "include")

set(TEST_NAME simulated_robot_test)
ament_add_gtest(${TEST_NAME} ${TEST_NAME}.cpp)
target_link_libraries(${TEST_NAME} ${LIBRARY_NAME})
ament_target_dependencies(${TEST_NAME} ${dependencies})
target_include_directories(${TEST_NAME} PRIVATE "../include" "include")