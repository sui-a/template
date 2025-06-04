#pragma once 
#include <queue>
namespace sui
{
	template<class T>
	class block_queue
	{
	public:
		block_queue(int&& a)
			:_size(a)
		{
		}


		~block_queue()
		{
		}

		//过半返回true 
		bool half()
		{
			return (_data.size() > _size / 2);
		}

		//充满返回true
		bool is_full()
		{
			return (_data.size() == _size);
		}

		//添加成功返回true
		bool add(T dat)
		{
			if (is_full())
				return false;

			_data.push(dat);

			return true;
		}

		//获取并删除，空返回false
		bool extract_and_delete(T* dat)
		{
			//是空返回
			if (empty())
				return false;

			*dat = _data.front();
			_data.pop();
			return true;
		}

		bool empty()
		{
			return _data.empty();
		}

	private:
		std::queue<T> _data;
		int _size;
	};
}