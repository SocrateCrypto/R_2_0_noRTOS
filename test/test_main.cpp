#include "unity.h"

void setUp(void) {}

void tearDown(void) {}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_buttons);
    RUN_TEST(test_potentiometer);
    RUN_TEST(test_state_machine);
    return UNITY_END();
}