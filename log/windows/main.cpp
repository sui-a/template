#pragma once
#include "log.hpp"
#include <Windows.h>
#include "pthread.hpp"
#include "pthreadpool.hpp"

void fun(void)
{
	int z = 10;
	while (z--)
	{
		std::cout << z << std::endl;
		Sleep(1000);
	}
}
int main()
{
	sui::my_pthreadpool* a = sui::my_pthreadpool::Get_Pthread_Pool();
	a->add_task(fun);
	a->add_task(fun);
	a->start();
	Sleep(1000);
	a->wait_and_join();
	std::cout << "进行休眠" << std::endl;
	Sleep(20000);
	
	return 0;
}