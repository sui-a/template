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
			LOG(sui::My_level::error) << "发送失败";
		}
		LOG(sui::My_level::info) << "发送成功";
		std::string re_dat;
		cli.receive(re_dat);
		LOG(sui::My_level::info) << "接收成功，数据是：" << re_dat;
		Sleep(2000);
	}
	return 0;
}