#include"my_sock.hpp"


int main()
{
	sui::my_sock_server server;
	LOG(sui::My_level::info) << "服务端开启";
	while (1)
	{
		std::string my_data;
		int a = server.receive(my_data);
		if (!a)
		{
			//成功接收数据
			LOG(sui::My_level::info) << "接收到数据:" << my_data;
			std::string my_data2 = "收到数据";
			server.to_send(my_data2);
		}
	}
	return 0;
}