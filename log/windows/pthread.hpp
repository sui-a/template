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

		//�߳���������
		static DWORD WINAPI ThreadFunc(LPVOID lpParam)
		{
			try
			{
				LOG(sui::My_level::info) << "�߳̿�ʼ";
				my_thread* thread = static_cast<my_thread*>(lpParam);
				if (thread) {
					thread->_fun(); // ���ô���ĺ���
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

		//�����߳�
		void start()
		{
			if (!_whether_start)
			{
				unsigned long id = -1;
				HANDLE hThread = CreateThread(
					NULL,                   // ��ȫ����
					0,                      // ��ջ��С
					ThreadFunc,             // �̺߳���
					this,                   // �̺߳����Ĳ���
					0,                      // ������־
					&id                    // �����߳�ID
				);

				if (hThread == NULL)
				{
					LOG(My_level::error) << "Failed to create thread.";
				}
				//�Ƿ���� �����뽫���ݴ洢������
				if (_whether_separate) {
					CloseHandle(hThread); // �����߳�
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

		//�ȴ�������
		void wait()
		{
			if (_whether_start)
			{
				// �ȴ��߳̽���
				WaitForSingleObject(_hThread, INFINITE);

				// �ر��߳̾��
				CloseHandle(_hThread);

				_hThread = nullptr;
				_whether_start = false;

				_id = -1;
			}
		}

		//�������ȴ�
		bool wait_0()
		{
			if (_whether_start)
			{
				// �ȴ��߳̽���
				DWORD result = WaitForSingleObject(_hThread, 0);

				if (result == WAIT_OBJECT_0)
				{
					//�ɹ�
					// �ر��߳̾��
					CloseHandle(_hThread);
					_hThread = nullptr;
					_whether_start = false;

					_id = -1;
					return true;
				}
				else if (result == WAIT_TIMEOUT)
				{
					//ʧ��
					return false;
				}
				else
				{
					//�ȴ�����
					LOG(My_level::error) << "wait_0 failed.";
					return false;
				}
			}
		}

		//ǿ���ж�
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
		//�߳��Ƿ�ʼ
		bool _whether_start;
		//�Ƿ����
		bool _whether_separate;
		//����
		std::function<void()> _fun;
		//id
		unsigned long _id;
		//�߳̾��
		HANDLE _hThread;
	};
}