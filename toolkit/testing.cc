
#include "testing.h"
#include <stdio.h>
#include "myvector"

struct TestFunction {
  void (*fn)();
  const char *name, *filename;
  int line_number;
};

// This should be a pointer to a vector, not the vector itself, because
// otherwise this might be constructed *after* some calls to __RegisterTest
// which are also driven from global initializers.
static std::vector<TestFunction> *test_functions;

void testing::__RegisterTest(void (*fn)(), const char *name,
                             const char *filename, int line_number) {
  if (!test_functions) {
    test_functions = new std::vector<TestFunction>;
  }
  TestFunction t;
  t.fn = fn;
  t.name = name;
  t.filename = filename;
  t.line_number = line_number;
  test_functions->push_back(t);
}

void testing::RunAll() {
  for (int i = 0; i < test_functions->size(); i++) {
    printf("********** Test %s (%s:%d)\n", test_functions->at(i).name,
           test_functions->at(i).filename, test_functions->at(i).line_number);
    test_functions->at(i).fn();
  }
  printf("%d tests passed!\n", (int) test_functions->size());
}

#ifdef __TOOLKIT_DEFINE_TESTING_MAIN__
int main(int argc, char **argv) {
  testing::RunAll();
  return 0;
}
#endif
