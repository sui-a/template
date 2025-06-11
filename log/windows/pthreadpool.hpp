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

		//ѭ��������ȡ����
		void HandlerTask(const int& i)
		{
			while (1)
			{
				std::function<void()> my_task;
				bool if_begin = false;
				//������ȡ����
				{
					std::unique_lock<std::mutex> lock(_mutex);
					while (_test.empty() && _if_running)
					{
						_sleep_thread++;
						cv.wait(lock, [this] { return this->start_thread; });
						start_thread = false;
						_sleep_thread--;
					}

					//�ж��̳߳��Ƿ�Ҫ��������
					if (!_if_running)
					{
						LOG(sui::My_level::info) << "��" << i << "���̣߳�׼��ֹͣ";
						//��ʾֹͣ����
						//�����Ƿ��еȴ����̣߳�ʼ�ձ���true��ֻ���
						start_thread = true;
						cv.notify_one();
						break;
					}

					if (_sleep_thread > 0 && !_test.empty())
					{
						//������������һ��п����߳� ���������
						start_thread = true;
						cv.notify_one();

					}
					my_task = _test.front();
					_test.pop();
				}
				
				//��������
				LOG(sui::My_level::info) << "��" << i << "���̣߳���ʼִ������";
				my_task();
			}
		}

		void add_task(std::function<void()> fun)
		{
			//�������һ���һ���߳�
			std::unique_lock<std::mutex> lock(_mutex);
			_test.push(fun);
			start_thread = true;
			cv.notify_one();
		}

		//ǿ��ֹͣ������ʹ�ã��ᵼ�¸�������
		void stop()
		{
			for (int i = 0; i < _pth_count; i++)
			{
				_thread[i].stop();
			}
			LOG(sui::My_level::info) << "ǿ��ֹͣ�̳߳سɹ�";
			std::unique_lock<std::mutex> lock(_mutex);
			_if_running = false;
			start_thread = false;
		}

		//�ȴ�������
		void wait_and_join()
		{
			wait();
			join();
		}

		~my_pthreadpool()
		{
			if (_if_running)
			{
				//�ȵȴ�
				wait();
				//����
				join();
			}
			else
			{
				//ʹ�õ����߳�Ĭ���ͷŲ���
			}
			LOG(sui::My_level::info) << "�����ɹ�";

		}

	private:
		//���ع��캯���������û����ⴴ��
		my_pthreadpool(int n)
			:_pth_count(n),
			_if_running(false),
			_sleep_thread(0),
			start_thread(false)
		{
			//�����ȴ��߳�
			for (int i = 0; i < n; i++)
			{
				_thread.push_back(my_thread([this, i]() {
					this->HandlerTask(i);
					}, false));
			}
		}

		//ֹͣ�̳߳أ����������߳�
		void wait()
		{
			LOG(sui::My_level::info) << "��ʼ���߳�ֹͣ";

			//���������ɷ�����״̬��Ȼ�󼤻�ȴ��߳�
			std::unique_lock<std::mutex> lock(_mutex);
			_if_running = false;
			start_thread = true;
			cv.notify_all();
		}

		//�����̳߳�
		void join()
		{
			LOG(sui::My_level::info) << "��ʼ�����߳�";

			for (int i = 0; i < _pth_count; i++)
			{
				_thread[i].wait();
				LOG(sui::My_level::info) << i << "���̻߳��ճɹ�";
			}
			LOG(sui::My_level::info) << "���ճɹ�";
			std::unique_lock<std::mutex> lock(_mutex);
			start_thread = false;
		}


		//�߳�����
		int _pth_count;
		//�̴߳洢�б�
		std::vector<my_thread> _thread;
		//�������
		std::queue <std::function<void()>> _test;
		//�Ƿ�����
		bool _if_running;
		//��
		std::mutex _mutex;
		//��������
		std::condition_variable cv;
		//�ȴ��߳�����
		int _sleep_thread;
		//�����߳��Ƿ�ʼ
		bool start_thread;

		//���ø��û�ʹ�õĽӿ�
		static my_pthreadpool* root_inc;
		//�û�����ʱʹ�õ���
		static std::mutex root_mutex;
	};
	my_pthreadpool* my_pthreadpool::root_inc = nullptr;
	std::mutex my_pthreadpool::root_mutex;
}