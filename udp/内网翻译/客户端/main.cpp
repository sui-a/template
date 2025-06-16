#include "my_client.hpp"

int main()
{
	std::string dat;
	sui::my_client cli;
	while (1)
	{
		std::cout << "请输入单词:@ ";
		std::getline(std::cin, dat);
		int a = cli.send(dat);
		if (a)
		{
			LOG(sui::My_level::error) << "发送失败";
		}
		std::string re_dat;
		int ret = cli.receive(re_dat);
		if(ret == 0)
		{
			std::cout << "获得数据：" << re_dat << std::endl;
		}
	}
	return 0;
}