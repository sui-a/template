#pragma once
#include <Windows.h>
#include <iostream>
#include <functional>
#include <vector>

namespace sui
{
	class my_thread
	{
	public:
		my_thread(std::function<void()> fun, bool while_separate)
			:_hThread(nullptr),
			_whether_separate(while_separate),
			_whether_start(false),
			_fun(fun),
			_id(-1)
		{

		}

		~my_thread()
		{
			if (_hThread != nullptr && _whether_start)
			{
				wait();
			}
		}

		//线程启动函数
		static DWORD WINAPI ThreadFunc(LPVOID lpParam)
		{
			try
			{
				my_thread* thread = static_cast<my_thread*>(lpParam);
				if (thread) {
					thread->_fun(); // 调用传入的函数
				}
			}
			catch (const std::exception& e)
			{
				std::cerr << "Exception in thread: " << e.what() << std::endl;
				std::cerr << "mythread " << std::endl;
			}
			catch (...) {
				std::cerr << "Unknown exception in thread." << std::endl;
				std::cerr << "mythread " << std::endl;
			}
			return 0;
		}

		//启动线程
		void start()
		{
			if (!_whether_start)
			{
				unsigned long id = -1;
				HANDLE hThread = CreateThread(
					NULL,                   // 安全属性
					0,                      // 堆栈大小
					ThreadFunc,             // 线程函数
					this,                   // 线程函数的参数
					0,                      // 创建标志
					&id                    // 返回线程ID
				);

				if (hThread == NULL)
				{
					std::cerr << "Failed to create thread." << std::endl;
				}
				//是否分离 不分离将数据存储在类中
				if (_whether_separate) {
					CloseHandle(hThread); // 分离线程
					_hThread = nullptr;
					_whether_start = false;
					_id = -1;
				}
				else {
					_hThread = hThread;
					_whether_start = true;
					_id = id;
				}
			}
		}

		//等待并回收
		void wait()
		{
			if (_whether_start)
			{
				// 等待线程结束
				WaitForSingleObject(_hThread, INFINITE);

				// 关闭线程句柄
				CloseHandle(_hThread);

				_hThread = nullptr;
				_whether_start = false;

				_id = -1;
			}
		}

		//非阻塞等待
		bool wait_0()
		{
			if (_whether_start)
			{
				// 等待线程结束
				DWORD result = WaitForSingleObject(_hThread, 0);

				if (result == WAIT_OBJECT_0)
				{
					//成功
					// 关闭线程句柄
					CloseHandle(_hThread);
					_hThread = nullptr;
					_whether_start = false;

					_id = -1;
					return true;
				}
				else if (result == WAIT_TIMEOUT)
				{
					//失败
					return false;
				}
				else
				{
					//等待失败
					std::cerr << "wait_0 failed." << std::endl;
					return false;
				}
			}
		}
		
	private:
		//线程是否开始
		bool _whether_start;
		//是否分离
		bool _whether_separate;
		//任务
		std::function<void()> _fun;
		//id
		unsigned long _id;
		//线程句柄
		HANDLE _hThread;
	};
	class thread_pool
	{
	public:
		thread_pool(int count)
			:_idleness_count(count),
			_count_thread(count)
		{
			_work_fun.push_back(thread_pool_init_fun);

			int current = 0;
			while (current < count)
			{
				int thread_id = current;
				_work_number.push_back(0);
				_thread_vector_work.push_back(false);
				_thread_vector.push_back(my_thread([this, thread_id](void)->void
					{
						std::cout << "线程" << thread_id << " begin: " << std::endl;
						std::cout << "运行序号" << this->_work_number[thread_id] << std::endl;
						this->_work_fun[this->_work_number[thread_id]]();
					},
					false));
				current++;
			}
			/*
			current = 0;
			while(current < count)
			{
				std::cout << "检查 " << current << std::endl;
				_thread_vector[1].start();
				_thread_vector[1].wait();
				current++;
			}*/
		}

		bool work(int num)
		{
			//递归回收
			if (_idleness_count <= _idleness_count / 2)
			{
				thread_recycle();
			}

			if (_idleness_count > 0)
			{
				int couo = 0;
				while (couo < _count_thread)
				{
					if (_thread_vector_work[couo] == false)
					{
						//找到未使用的线程
						std::cout << "有可用线程: " << couo << std::endl;
						_work_number[couo] = num;
						_thread_vector[couo].start();
						_thread_vector_work[couo] = true;
						_idleness_count--;
						return true;
					}
					couo++;
				}
			}
			else
			{
				std::cout << "线程剩余量不足" << std::endl;
				return false;
			}
		}

		//回收
		void thread_recycle()
		{
			int couo = 0;
			while (couo < _count_thread)
			{
				if (_thread_vector_work[couo])
				{
					//线程处于使用种，尝试回收
					bool recycle = _thread_vector[couo].wait_0();
					if (recycle)
					{
						//成功回收
						_thread_vector_work[couo] = false;
						_idleness_count++;
					}
				}
				couo++;
			}
		}

		//任务注册
		int task_registration(std::function<void()> fun)
		{
			_work_fun.push_back(fun);
			return _work_fun.size() - 1;
		}

		static void thread_pool_init_fun()
		{
			std::cout << "pool " << std::endl;
		}

		~thread_pool()
		{
			//递归回收释放
			int couo = 0;
			while (couo < _count_thread)
			{
				if (_thread_vector_work[couo])
				{
					//线程处于使用种，尝试回收
					_thread_vector[couo].wait();
					_thread_vector_work[couo] = false;
					_idleness_count++;
				}
				couo++;
			}
		}

	private:
		//线程存储
		std::vector<my_thread> _thread_vector;
		//是否在工作
		std::vector<bool> _thread_vector_work;
		//对应线程执行哪个操作
		std::vector<int> _work_number;
		//任务存储
		std::vector<std::function<void()>> _work_fun;
		//总共的数量
		int _count_thread;
		//空闲线程数量
		int _idleness_count;
	};
};