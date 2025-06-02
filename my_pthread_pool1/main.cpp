#include "mythread.hpp"
#include <Windows.h>


int main()
{
	sui::thread_pool a(4);
	int n = a.task_registration([]()->void {
		int s = 10;
		while (s--)
		{
			std::cout << s << std::endl;
		}
		});
	std::cout << "n:" << n << std::endl;
	int zzz = 5;
	while (zzz--)
	{
		a.work(n);
	}

	Sleep(10000);
	std::cout << "pause success" << std::endl;
	int tt = 0;
	std::cin >> tt;
	return 0;
}