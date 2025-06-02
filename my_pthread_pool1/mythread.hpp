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

		//�߳���������
		static DWORD WINAPI ThreadFunc(LPVOID lpParam)
		{
			try
			{
				my_thread* thread = static_cast<my_thread*>(lpParam);
				if (thread) {
					thread->_fun(); // ���ô���ĺ���
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
					std::cerr << "Failed to create thread." << std::endl;
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
					//�ȴ�ʧ��
					std::cerr << "wait_0 failed." << std::endl;
					return false;
				}
			}
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
						std::cout << "�߳�" << thread_id << " begin: " << std::endl;
						std::cout << "�������" << this->_work_number[thread_id] << std::endl;
						this->_work_fun[this->_work_number[thread_id]]();
					},
					false));
				current++;
			}
			/*
			current = 0;
			while(current < count)
			{
				std::cout << "��� " << current << std::endl;
				_thread_vector[1].start();
				_thread_vector[1].wait();
				current++;
			}*/
		}

		bool work(int num)
		{
			//�ݹ����
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
						//�ҵ�δʹ�õ��߳�
						std::cout << "�п����߳�: " << couo << std::endl;
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
				std::cout << "�߳�ʣ��������" << std::endl;
				return false;
			}
		}

		//����
		void thread_recycle()
		{
			int couo = 0;
			while (couo < _count_thread)
			{
				if (_thread_vector_work[couo])
				{
					//�̴߳���ʹ���֣����Ի���
					bool recycle = _thread_vector[couo].wait_0();
					if (recycle)
					{
						//�ɹ�����
						_thread_vector_work[couo] = false;
						_idleness_count++;
					}
				}
				couo++;
			}
		}

		//����ע��
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
			//�ݹ�����ͷ�
			int couo = 0;
			while (couo < _count_thread)
			{
				if (_thread_vector_work[couo])
				{
					//�̴߳���ʹ���֣����Ի���
					_thread_vector[couo].wait();
					_thread_vector_work[couo] = false;
					_idleness_count++;
				}
				couo++;
			}
		}

	private:
		//�̴߳洢
		std::vector<my_thread> _thread_vector;
		//�Ƿ��ڹ���
		std::vector<bool> _thread_vector_work;
		//��Ӧ�߳�ִ���ĸ�����
		std::vector<int> _work_number;
		//����洢
		std::vector<std::function<void()>> _work_fun;
		//�ܹ�������
		int _count_thread;
		//�����߳�����
		int _idleness_count;
	};
};