#include "unity.h"

void test_button_press(void) {
    TEST_ASSERT_TRUE(button_press());
}

void test_button_release(void) {
    TEST_ASSERT_FALSE(button_release());
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_button_press);
    RUN_TEST(test_button_release);
    return UNITY_END();
}