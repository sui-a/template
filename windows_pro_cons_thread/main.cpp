#include "my_pthread.hpp"
#include <thread>
#include <Windows.h>

void my_fun1(sui::my_block_queue<int>& mbq)
{
	int a = 0;
	while (1)
	{
		mbq.data_add(a);
		a++;
		Sleep(2000);
	}
}

void my_fun2(sui::my_block_queue<int>& mbq)
{
	while (1)
	{
		int a;
		bool zzz = mbq.data_front(&a);
		if (zzz)
		{
			std::cout << "获得数据 " << a << std::endl;
		}
		Sleep(2000);
	}
}

int main()
{
	//创建两个线程，然后进入死循环
	sui::my_block_queue<int> a;
	                         //使用包装器，让参数可以正确右值引用
	std::thread add(my_fun1, std::ref(a));
	std::thread front(my_fun2, std::ref(a));


	while(1)
	{ }
	return 0;
}