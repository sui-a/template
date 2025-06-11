#pragma once
#include "log.hpp"
#include <vector>
#include <iostream>
#include "pthread.hpp"
#include <queue>
#include <functional>
#include <Windows.h>

namespace sui
{
	class my_pthreadpool
	{
	public:
		static my_pthreadpool* Get_Pthread_Pool(int a = 5)
		{
			std::unique_lock<std::mutex> lock(root_mutex);
			if (root_inc == nullptr)
			{
				root_inc = new my_pthreadpool(5);
			}
			return root_inc;
		}

		void start()
		{
			for (int i = 0; i < _pth_count; i++)
			{
				_thread[i].start();
			}
			_if_running = true;
		}

		//循环遍历获取任务
		void HandlerTask(const int& i)
		{
			while (1)
			{
				std::function<void()> my_task;
				bool if_begin = false;
				//上锁获取任务
				{
					std::unique_lock<std::mutex> lock(_mutex);
					while (_test.empty() && _if_running)
					{
						_sleep_thread++;
						cv.wait(lock, [this] { return this->start_thread; });
						start_thread = false;
						_sleep_thread--;
					}

					//判断线程池是否还要继续运行
					if (!_if_running)
					{
						LOG(sui::My_level::info) << "第" << i << "号线程，准备停止";
						//显示停止进入
						//无论是否有等待的线程，始终保持true坚持唤醒
						start_thread = true;
						cv.notify_one();
						break;
					}

					if (_sleep_thread > 0 && !_test.empty())
					{
						//如果还有任务并且还有空闲线程 则继续唤醒
						start_thread = true;
						cv.notify_one();

					}
					my_task = _test.front();
					_test.pop();
				}
				
				//启动任务
				LOG(sui::My_level::info) << "第" << i << "号线程，开始执行任务";
				my_task();
			}
		}

		void add_task(std::function<void()> fun)
		{
			//上锁并且唤醒一次线程
			std::unique_lock<std::mutex> lock(_mutex);
			_test.push(fun);
			start_thread = true;
			cv.notify_one();
		}

		//强制停止，谨慎使用，会导致各种问题
		void stop()
		{
			for (int i = 0; i < _pth_count; i++)
			{
				_thread[i].stop();
			}
			LOG(sui::My_level::info) << "强制停止线程池成功";
			std::unique_lock<std::mutex> lock(_mutex);
			_if_running = false;
			start_thread = false;
		}

		//等待并回收
		void wait_and_join()
		{
			wait();
			join();
		}

		~my_pthreadpool()
		{
			if (_if_running)
			{
				//先等待
				wait();
				//回收
				join();
			}
			else
			{
				//使用单个线程默认释放操作
			}
			LOG(sui::My_level::info) << "析构成功";

		}

	private:
		//隐藏构造函数，不让用户随意创建
		my_pthreadpool(int n)
			:_pth_count(n),
			_if_running(false),
			_sleep_thread(0),
			start_thread(false)
		{
			//创建等待线程
			for (int i = 0; i < n; i++)
			{
				_thread.push_back(my_thread([this, i]() {
					this->HandlerTask(i);
					}, false));
			}
		}

		//停止线程池，但不回收线程
		void wait()
		{
			LOG(sui::My_level::info) << "开始让线程停止";

			//上锁调整成非运行状态，然后激活等待线程
			std::unique_lock<std::mutex> lock(_mutex);
			_if_running = false;
			start_thread = true;
			cv.notify_all();
		}

		//回收线程池
		void join()
		{
			LOG(sui::My_level::info) << "开始回收线程";

			for (int i = 0; i < _pth_count; i++)
			{
				_thread[i].wait();
				LOG(sui::My_level::info) << i << "号线程回收成功";
			}
			LOG(sui::My_level::info) << "回收成功";
			std::unique_lock<std::mutex> lock(_mutex);
			start_thread = false;
		}


		//线程数量
		int _pth_count;
		//线程存储列表
		std::vector<my_thread> _thread;
		//任务队列
		std::queue <std::function<void()>> _test;
		//是否运行
		bool _if_running;
		//锁
		std::mutex _mutex;
		//条件变量
		std::condition_variable cv;
		//等待线程数量
		int _sleep_thread;
		//告诉线程是否开始
		bool start_thread;

		//设置给用户使用的接口
		static my_pthreadpool* root_inc;
		//用户操作时使用的锁
		static std::mutex root_mutex;
	};
	my_pthreadpool* my_pthreadpool::root_inc = nullptr;
	std::mutex my_pthreadpool::root_mutex;
}