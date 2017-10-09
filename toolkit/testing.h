// Simple unit testing infrastructure.

#ifndef __TOOLKIT_TESTING_H__
#define __TOOLKIT_TESTING_H__

namespace testing {

  // Run all unit tests.
  void RunAll();

  // Internal function called by TEST_FUNCTION.
  void __RegisterTest(void (*fn)(), const char *name,
                      const char *filename, int line_number);

}

// Define a new test function.
#define TEST_FUNCTION(name) \
  void __RunTest_##name(); \
  static struct __RegisterTest_##name { \
    __RegisterTest_##name() { \
      testing::__RegisterTest(__RunTest_##name, #name, __FILE__, __LINE__); \
    } \
  } __register_test_##name; \
  void __RunTest_##name()

#endif
