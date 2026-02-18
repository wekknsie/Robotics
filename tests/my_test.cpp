#include <gtest/gtest.h>

// Simple addition function for demonstration.
float add(float a, float b) {
    return a + b;
}

TEST(AdditionTest, AddsPositiveNumbers) {
    EXPECT_FLOAT_EQ(add(5.0f, 10.0f), 15.0f);
    EXPECT_FLOAT_EQ(add(0.0f, 0.0f), 0.0f);
}

TEST(AdditionTest, AddsEqualNumbers) {
    EXPECT_FLOAT_EQ(add(10.0f, 10.0f), 20.0f);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
