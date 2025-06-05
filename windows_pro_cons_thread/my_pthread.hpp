/*
这里的生产消费模型本质是中介控制器，
将锁和环境变量交给add和front上实现，
将生产者和消费者交给线程来做

*条件变量可以当作阻塞标准，一旦激活就可以无视其是否启动或销毁
*这种封装方式天生可以进行多生产多消费
*这里的阻塞管理队列不需要关注线程的调用持续，纯粹把条件变量当作阻塞线程的工具
*/

#pragma once 
#include <queue>
#include <mutex>
#include <iostream>

#define MY_BLOCK_QUEUE_NUM 5

namespace sui
{

	//阻塞生产消费管理队列
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
			//等于最大值返回true
			return _max_count == _data.size();
		}

		bool empty()
		{
			return _data.empty();
		}

		bool data_add(T data)
		{
			{
				//上锁
				std::unique_lock<std::mutex> lock(_mutex);

				//满的话就一直循环 防止伪激活  未满并抢到锁则继续执行
				while (is_full())
				{

					//增加等待的线程
					_wait_count++;
					//进入环境变量判断        满的不让返回
					_cv.wait(lock, [this]() {return !this->is_full();});
					_wait_count--;
				}

				_data.push(data);

			}

			//如果等待线程大于0，则直接激活
			if(_wait_count > 0)
			{
				_cv.notify_one();
			}

			//放入类中自动解锁，无需管理
			
			return true;
		}

		bool data_front(T* data)
		{
			{
				//进行上锁
				std::unique_lock<std::mutex> lock(_mutex);


				//是空则一直循环 并等待  非空并持有锁则持续执行
				while (empty())
				{

					_wait_count++;
					//非空则进入
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

		//不需要在销毁的时候激活所有等待调度的线程，因为是阻塞队列，没完成不会进行析构
		~my_block_queue()
		{ }


	private:
		std::queue<T> _data;
		//最大存储量
		int _max_count;

		//锁
		std::mutex _mutex;
		//条件变量
		std::condition_variable _cv;
		
		//等待数量
		int _wait_count;

	};
}