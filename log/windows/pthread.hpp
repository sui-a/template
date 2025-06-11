#pragma once
#include<functional>
#include <Windows.h>
#include <iostream>
#include "log.hpp"

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
				LOG(sui::My_level::info) << "线程开始";
				my_thread* thread = static_cast<my_thread*>(lpParam);
				if (thread) {
					thread->_fun(); // 调用传入的函数
				}
			}
			catch (const std::exception& e)
			{
				LOG(sui::My_level::error) << "Exception in thread: " << e.what();
			}
			catch (...) 
			{
				LOG(sui::My_level::error) << "Unknown exception in thread.";
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
					LOG(My_level::error) << "Failed to create thread.";
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
					//等待错误
					LOG(My_level::error) << "wait_0 failed.";
					return false;
				}
			}
		}

		//强制中断
		void stop()
		{
			if (!TerminateThread(_hThread, 0)) 
			{
				LOG(My_level::error) << "Failed to terminate thread.";
			}
			CloseHandle(_hThread);
		}

		unsigned long get_id()
		{
			return _id;
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
}