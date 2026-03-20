#include <cstdio>
#include <cstdint>
#include <vector>

#include "my_bot_hw/stm32_comm.hpp"

using namespace my_bot_hw;

// Test case structure
struct TestCase
{
  const char * name;
  std::vector<uint8_t> data;
  uint8_t expected_crc;
};

int main()
{
  printf("=== CRC8 Function Unit Tests ===\n\n");

  // Test vectors based on vendor protocol
  // Frame format: [0x5A][LEN][0x01][CMD][DATA...][0x00][CRC8]
  // CRC is calculated over all bytes except the CRC byte itself

  std::vector<TestCase> tests = {
    // Test 1: Empty data
    {
      "Empty data",
      {},
      0x00
    },

    // Test 2: Single byte 0x00
    {
      "Single byte 0x00",
      {0x00},
      0x00
    },

    // Test 3: Single byte 0x5A (frame header)
    {
      "Single byte 0x5A",
      {0x5A},
      0xA5
    },

    // Test 4: Single byte 0x01
    {
      "Single byte 0x01",
      {0x01},
      0x5E
    },

    // Test 5: Simple CMD 0x11 request frame (request odometry)
    // Frame: [0x5A][0x07][0x01][0x11][0x00][0x00][0x00]
    // CRC should be calculated over [0x5A][0x07][0x01][0x11][0x00][0x00] (6 bytes)
    {
      "CMD 0x11 request odometry (first 6 bytes)",
      {0x5A, 0x07, 0x01, 0x11, 0x00, 0x00},
      0xDE
    },

    // Test 6: Frame with all zeros except header
    {
      "Frame: [0x5A][0x00][0x00][0x00][0x00][0x00]",
      {0x5A, 0x00, 0x00, 0x00, 0x00, 0x00},
      0xE1
    },

    // Test 7: Frame header + length
    {
      "Frame: [0x5A][0x07]",
      {0x5A, 0x07},
      0x13
    },

    // Test 8: Frame with incrementing bytes
    {
      "Incrementing bytes: [0x00...0x04]",
      {0x00, 0x01, 0x02, 0x03, 0x04},
      0xF4
    },

    // Test 9: Frame with all 0xFF
    {
      "All 0xFF: 4 bytes",
      {0xFF, 0xFF, 0xFF, 0xFF},
      0x8D
    },

    // Test 10: Typical velocity command frame structure (without actual data)
    // [0x5A][0x0E][0x01][0x01][0x00][0x00][0x00][0x00][0x00][0x00][0x00][0x00][0x00]
    // CRC calculated over bytes 0-12
    {
      "Velocity command frame structure (13 bytes)",
      {0x5A, 0x0E, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      0x9B
    }
  };

  int passed = 0;
  int failed = 0;

  for (const auto & test : tests) {
    uint8_t computed_crc = crc8_calculate(test.data);
    bool is_pass = (computed_crc == test.expected_crc);

    if (is_pass) {
      passed++;
      printf("[PASS] %s\n", test.name);
      printf("       Input: ");
      for (const auto & byte : test.data) {
        printf("0x%02X ", byte);
      }
      printf("\n       Expected: 0x%02X, Got: 0x%02X\n\n", test.expected_crc, computed_crc);
    } else {
      failed++;
      printf("[FAIL] %s\n", test.name);
      printf("       Input: ");
      for (const auto & byte : test.data) {
        printf("0x%02X ", byte);
      }
      printf("\n       Expected: 0x%02X, Got: 0x%02X\n\n", test.expected_crc, computed_crc);
    }
  }

  printf("=== Test Summary ===\n");
  printf("Passed: %d/%zu\n", passed, tests.size());
  printf("Failed: %d/%zu\n", failed, tests.size());

  // Frame packing tests
  printf("\n=== Frame Packing Tests ===\n\n");

  int frame_tests_passed = 0;
  int frame_tests_failed = 0;

  // Test 1: CMD 0x11 (request odometry)
  {
    auto frame = build_cmd_request_odom();
    std::vector<uint8_t> expected = {0x5A, 0x07, 0x01, 0x11, 0x00, 0x00, 0xDE};
    bool match = (frame == expected);
    if (match) {
      frame_tests_passed++;
      printf("[PASS] CMD 0x11 (request odom): ");
    } else {
      frame_tests_failed++;
      printf("[FAIL] CMD 0x11 (request odom): ");
    }
    for (const auto & b : frame) {
      printf("%02X ", b);
    }
    printf("\n");
  }

  // Test 2: CMD 0x01 (velocity v=0.0 m/s, w=0.0 rad/s)
  {
    auto frame = build_cmd_velocity(0.0f, 0.0f, 0.0f);
    std::vector<uint8_t> expected = {0x5A, 0x0C, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    // Last byte is CRC, compute it
    expected.push_back(crc8_calculate(expected.data(), expected.size()));
    bool match = (frame == expected);
    if (match) {
      frame_tests_passed++;
      printf("[PASS] CMD 0x01 (v=0.0 w=0.0):  ");
    } else {
      frame_tests_failed++;
      printf("[FAIL] CMD 0x01 (v=0.0 w=0.0):  ");
    }
    for (const auto & b : frame) {
      printf("%02X ", b);
    }
    printf("\n");
  }

  // Test 3: CMD 0x01 (velocity v=0.3 m/s, w=0.0 rad/s)
  {
    auto frame = build_cmd_velocity(0.3f, 0.0f, 0.0f);
    // 0.3 * 1000 = 300 = 0x012C (big-endian: 0x01 0x2C)
    std::vector<uint8_t> expected = {0x5A, 0x0C, 0x01, 0x01, 0x01, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00};
    expected.push_back(crc8_calculate(expected.data(), expected.size()));
    bool match = (frame == expected);
    if (match) {
      frame_tests_passed++;
      printf("[PASS] CMD 0x01 (v=0.3 w=0.0):  ");
    } else {
      frame_tests_failed++;
      printf("[FAIL] CMD 0x01 (v=0.3 w=0.0):  ");
    }
    for (const auto & b : frame) {
      printf("%02X ", b);
    }
    printf("\n");
  }

  // Test 4: CMD 0x13 (request IMU)
  {
    auto frame = build_cmd_request_imu();
    // Should match structure of CMD 0x11 but with different CMD
    std::vector<uint8_t> expected = {0x5A, 0x07, 0x01, 0x13, 0x00, 0x00};
    expected.push_back(crc8_calculate(expected.data(), expected.size()));
    bool match = (frame == expected);
    if (match) {
      frame_tests_passed++;
      printf("[PASS] CMD 0x13 (request IMU):  ");
    } else {
      frame_tests_failed++;
      printf("[FAIL] CMD 0x13 (request IMU):  ");
    }
    for (const auto & b : frame) {
      printf("%02X ", b);
    }
    printf("\n");
  }

  printf("\n=== Frame Test Summary ===\n");
  printf("Passed: %d/4\n", frame_tests_passed);
  printf("Failed: %d/4\n", frame_tests_failed);

  return ((failed == 0) && (frame_tests_failed == 0)) ? 0 : 1;
}
