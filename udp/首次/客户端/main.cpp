#include "my_client.hpp"

int main()
{
	std::string dat = "hello world";
	sui::my_client cli;
	while (1)
	{
		int a = cli.send(dat);
		if (a)
		{
			LOG(sui::My_level::error) << "����ʧ��";
		}
		LOG(sui::My_level::info) << "���ͳɹ�";
		std::string re_dat;
		cli.receive(re_dat);
		LOG(sui::My_level::info) << "���ճɹ��������ǣ�" << re_dat;
		Sleep(2000);
	}
	return 0;
}