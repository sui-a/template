#include <iostream>
#include "blockqueue.hpp"
#include <Windows.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <atomic>


//提供原子性变量
std::atomic<bool> stop_flag(false);

bool ff = false;

sui::block_queue<int> bq(10);

// 线程函数
void shenchan_thread(std::mutex& mtx, std::condition_variable& cv)
{
	static int a = 0;

	while (1) 
	{
		std::unique_lock<std::mutex> lock(mtx);
		cv.wait(lock, [] { return stop_flag.load();});

		if (!ff) {
			std::cout << "xiaofei: 退出" << std::endl;

			break;
		}
		std::cout << "生产" << std::endl;

		stop_flag = false;

		//低于一半进入
		if (!bq.half())
		{
			while (!bq.is_full())
			{
				bq.add(a % 10);
				a++;
			}
		}
	}

}

void xiaofei_thread(std::mutex& mtx, std::condition_variable& cv)
{
	while (1) 
	{
		std::unique_lock<std::mutex> lock(mtx);
		cv.wait(lock, [] { return stop_flag.load();});

		if (!ff) {
			std::cout << "xiaofei: 退出" << std::endl;
			break;
		}
		std::cout << "消费" << std::endl;

		stop_flag = false;
		// 空则退出if
		if (!bq.empty())
		{
			int data = 0;
			bool ss = bq.extract_and_delete(&data);
			if (!ss)
			{
				std::cerr << "提取错误" << std::endl;
			}
			else
			{
				std::cout << "消费值： " << data << std::endl;
			}
		}
	}
}

int main()
{
	//锁
	std::mutex mtx;

	//条件变量
	std::condition_variable cv;

	

	std::thread shenchan(shenchan_thread, std::ref(mtx), std::ref(cv));
	Sleep(100);
	std::thread xiaofei(xiaofei_thread, std::ref(mtx), std::ref(cv));
	Sleep(100);


	int a = 0;

	ff = true;
	
	while (1)
	{
		bool tt = stop_flag.load();
		if(!tt)
		{
			stop_flag = true;
			cv.notify_one();
			std::cout << "a: " << a << std::endl;
			//使用这个会使线程失去顺序
			//cv.notify_all();
			a++;
			if (a >= 40)
			{
				break;
			}
			Sleep(2000);
			std::cout << "-------------------------" << std::endl;
		}
	}

	//清空线程

	{
		std::lock_guard<std::mutex> lock(mtx);
		stop_flag = true;
		ff = false;
	}
	cv.notify_all();


	std::cout << "准备结束" << std::endl;
	shenchan.join();
	xiaofei.join();
	Sleep(1000);


	return 0;
}