#include "my_sys.hpp"

int main()
{
	my_shell shell;
	while (true)
	{
		shell.my_shell_head();
		shell.my_shell_input();
	}
	return 0;
}