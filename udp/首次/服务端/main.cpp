#include"my_sock.hpp"


int main()
{
	sui::my_sock_server server;
	LOG(sui::My_level::info) << "����˿���";
	while (1)
	{
		std::string my_data;
		int a = server.receive(my_data);
		if (!a)
		{
			//�ɹ���������
			LOG(sui::My_level::info) << "���յ�����:" << my_data;
			std::string my_data2 = "�յ�����";
			server.to_send(my_data2);
		}
	}
	return 0;
}