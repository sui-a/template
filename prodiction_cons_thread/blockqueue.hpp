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

		//���뷵��true 
		bool half()
		{
			return (_data.size() > _size / 2);
		}

		//��������true
		bool is_full()
		{
			return (_data.size() == _size);
		}

		//��ӳɹ�����true
		bool add(T dat)
		{
			if (is_full())
				return false;

			_data.push(dat);

			return true;
		}

		//��ȡ��ɾ�����շ���false
		bool extract_and_delete(T* dat)
		{
			//�ǿշ���
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