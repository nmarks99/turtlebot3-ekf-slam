#include <catch2/catch_test_macros.hpp>
#include "nuslam/circle_fitting.hpp"

TEST_CASE("CircleFitting()", "[CircleFitting]")
{
  CircleFitting cf(1);
  int result = cf.two_sum(2);
  REQUIRE(result == 3);
  std::cout << "result = " << result << std::endl;
}
