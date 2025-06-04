#include <iostream>
#include "blockqueue.hpp"
#include <Windows.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <atomic>


//�ṩԭ���Ա���
std::atomic<bool> stop_flag(false);

bool ff = false;

sui::block_queue<int> bq(10);

// �̺߳���
void shenchan_thread(std::mutex& mtx, std::condition_variable& cv)
{
	static int a = 0;

	while (1) 
	{
		std::unique_lock<std::mutex> lock(mtx);
		cv.wait(lock, [] { return stop_flag.load();});

		if (!ff) {
			std::cout << "xiaofei: �˳�" << std::endl;

			break;
		}
		std::cout << "����" << std::endl;

		stop_flag = false;

		//����һ�����
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
			std::cout << "xiaofei: �˳�" << std::endl;
			break;
		}
		std::cout << "����" << std::endl;

		stop_flag = false;
		// �����˳�if
		if (!bq.empty())
		{
			int data = 0;
			bool ss = bq.extract_and_delete(&data);
			if (!ss)
			{
				std::cerr << "��ȡ����" << std::endl;
			}
			else
			{
				std::cout << "����ֵ�� " << data << std::endl;
			}
		}
	}
}

int main()
{
	//��
	std::mutex mtx;

	//��������
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
			//ʹ�������ʹ�߳�ʧȥ˳��
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

	//����߳�

	{
		std::lock_guard<std::mutex> lock(mtx);
		stop_flag = true;
		ff = false;
	}
	cv.notify_all();


	std::cout << "׼������" << std::endl;
	shenchan.join();
	xiaofei.join();
	Sleep(1000);


	return 0;
}