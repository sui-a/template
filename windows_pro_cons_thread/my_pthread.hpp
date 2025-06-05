/*
�������������ģ�ͱ������н��������
�����ͻ�����������add��front��ʵ�֣�
�������ߺ������߽����߳�����

*�����������Ե���������׼��һ������Ϳ����������Ƿ�����������
*���ַ�װ��ʽ�������Խ��ж�����������
*���������������в���Ҫ��ע�̵߳ĵ��ó���������������������������̵߳Ĺ���
*/

#pragma once 
#include <queue>
#include <mutex>
#include <iostream>

#define MY_BLOCK_QUEUE_NUM 5

namespace sui
{

	//�����������ѹ������
	template<class T>
	class my_block_queue
	{
	public:
		my_block_queue(int&& size)
			:_max_count(size),
			_wait_count(0)
		{ }

		my_block_queue(int size = MY_BLOCK_QUEUE_NUM)
			:_max_count(size),
			_wait_count(0)
		{ }

		bool is_full()
		{
			//�������ֵ����true
			return _max_count == _data.size();
		}

		bool empty()
		{
			return _data.empty();
		}

		bool data_add(T data)
		{
			{
				//����
				std::unique_lock<std::mutex> lock(_mutex);

				//���Ļ���һֱѭ�� ��ֹα����  δ���������������ִ��
				while (is_full())
				{

					//���ӵȴ����߳�
					_wait_count++;
					//���뻷�������ж�        ���Ĳ��÷���
					_cv.wait(lock, [this]() {return !this->is_full();});
					_wait_count--;
				}

				_data.push(data);

			}

			//����ȴ��̴߳���0����ֱ�Ӽ���
			if(_wait_count > 0)
			{
				_cv.notify_one();
			}

			//���������Զ��������������
			
			return true;
		}

		bool data_front(T* data)
		{
			{
				//��������
				std::unique_lock<std::mutex> lock(_mutex);


				//�ǿ���һֱѭ�� ���ȴ�  �ǿղ������������ִ��
				while (empty())
				{

					_wait_count++;
					//�ǿ������
					_cv.wait(lock, [this]() {return !this->empty();});
					_wait_count--;
				}

				*data = _data.front();
				_data.pop();
			}

			if (_wait_count > 0)
			{
				_cv.notify_one();
			}

			

			return true;
		}

		//����Ҫ�����ٵ�ʱ�򼤻����еȴ����ȵ��̣߳���Ϊ���������У�û��ɲ����������
		~my_block_queue()
		{ }


	private:
		std::queue<T> _data;
		//���洢��
		int _max_count;

		//��
		std::mutex _mutex;
		//��������
		std::condition_variable _cv;
		
		//�ȴ�����
		int _wait_count;

	};
}