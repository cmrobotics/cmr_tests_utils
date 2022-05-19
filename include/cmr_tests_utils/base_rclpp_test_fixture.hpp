#ifndef BASE_RCLCPP_TEST_FIXTURE_HPP
#define BASE_RCLCPP_TEST_FIXTURE_HPP

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

namespace {

template <class TestFixture>
int run_test(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);
  int all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return all_successful;
}

class BaseRclcppTestFixture : public ::testing::Test
{
public:
  void SetUp() override {}

  void TearDown() override {}
};

}

#endif