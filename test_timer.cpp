#include "timer.h"
#include <iostream>
using wrj::Timer;

void test_time() {
  wrj::Timer t("sleep");
  t.Start();
	Sleep(1000);
	t.Stop();
	t.Start();
	Sleep(500);
  t.StopAndReport();


	t.Reset();
	t.Start();
	Sleep(500);
	t.StopAndReport();
}


int main()
{
	test_time();
	std::cout << "All work successfully." << std::endl;
	return 0;
}