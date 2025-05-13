#include <fcntl.h>
#include <sys/stat.h>
#include <iostream>
#include <unistd.h>
#include <string>

#define fifo "fifo"

int main()
{
    std::cout << "link ing~" << std::endl;
    int fd = 0;
    while (true)
    {
        fd = open(fifo, O_RDWR);
        if (fd == -1)
        {
            std::cerr << "open error" << std::endl;
        }
        else { break; }
    }

    std::cout << "link success" << std::endl;
    std::string cli_cin;
    while (true)
    {
        std::cout << "please:@   ";
        std::getline(std::cin, cli_cin);
        if (cli_cin.empty())
        {
            continue;
        }
        int n = write(fd, cli_cin.c_str(), cli_cin.size());
        if (fd == -1)
        {
            std::cerr << "write error " << std::endl;
            break;
        }
        cli_cin.clear();
    }
    close(fd);
    return 0;
}