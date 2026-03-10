// Minimal GTest example for a line estimator
#include <cstdint>
#include <gtest/gtest.h>

TEST(LineEstimator, BasicDiscreteEstimation) {
    uint16_t left_value = 0;
    uint16_t right_value = 1024;
    auto result = LineEstimator::estimate_discrete(left_value, right_value);
    EXPECT_EQ(result, /* expected pose */);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
