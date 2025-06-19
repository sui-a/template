#include "tcp_ser.hpp"

int main()
{
	sui::my_tcp_server my_server(8888);
	my_server.init();
	my_server.start();
	return 0;
}