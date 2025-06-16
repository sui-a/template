#include "server.hpp"
#include <direct.h>
#include "dictioinary.hpp"
int main()
{
	sui::my_server::get_server()->init();
	sui::my_server::get_server()->start();
	return 0;
}