/*
#include <Arduino.h>
#include <unity.h>

void setUp(void) {
  // Set up before each test
}

void tearDown(void) {
  // Clean up after each test
}

void test_hello_world(void) {
  TEST_ASSERT_TRUE(true);
}

void setup() {
  // 1. Wait for hardware to settle
  delay(2000); 
  
  // 2. Wait explicitly for the TinyUSB CDC serial connection
  // Without this, the RP2040 will print the results before PIO is listening
  while (!Serial) {
    delay(10);
  }

  UNITY_BEGIN();
  RUN_TEST(test_hello_world);
  UNITY_END();
}

void loop() {
  // Empty
}
*/