#define WRJ_DEBUG
#include "utils.h"
#include "timer.h"

using wrj::Timer;

static int test_cnt = 0;
static int test_passed = 0;

#define EXPECT_EQ_BASE(equality, expect, actual, format) \
do { \
  ++test_cnt; \
  if (equality) { \
    ++test_passed; \
  } else { \
    fprintf(stderr, "%s:%d expect: " format " actual: " format "\n", __FILE__, __LINE__, expect, actual); \
  } \
while (0)

#define EXPECT_EQ_INT(expect, actual) EXPECT_EQ_BASE((expect) == (actual), expect, actual, "%d");
#define EXPECT_EQ_BOOL(expect, actual) EXPECT_EQ_BASE((expect) == (actual), expect, actual, "%d");

void test_assert() {
  WRJ_ASSERT(1);  // succeed 
  //WRJ_ASSERT(0);  // error
}

int main()
{
	test_assert();
	std::cout << "All work successfully." << std::endl;
	return 0;
}