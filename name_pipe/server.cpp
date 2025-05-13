#include <fcntl.h>
#include <sys/stat.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <cstring>

#define fifo "fifo"

void signal_handler(int signum)
{
    std::cout << "shut fifo" << std::endl;
    unlink(fifo);
    exit(signum);
}

int main()
{
    signal(SIGINT, signal_handler);

    umask(0);
    int n = mkfifo(fifo, 0666);
    if (n == -1)
    {
        if (errno != EEXIST)
        {
            std::cerr << "mkfifo error: " << n << std::endl;
        }
    }

    std::cout << "fifo success" << std::endl;
    int fd = open(fifo, O_RDONLY);
    if (fd == -1)
    {
        std::cerr << "open error" << std::endl;
    }
    else
    {
        std::cout << "open success " << std::endl;
        char buff[1024];
        while (true)
        {
            buff[0] = '\0';
            int n = read(fd, buff, sizeof(buff) - 1);
            if (n < 0)
            {
                std::cerr << "read error" << std::endl;
                break;
            }
            else if (n == 0)
            {
                continue;
            }
            else
            {
                std::cout << "child say: " << buff << std::endl;
            }
        }
    }

    //shut mkfifo file
    if (unlink(fifo) == -1)
    {
        std::cerr << "unlink error" << std::endl;
        return 1;
    }
    std::cout << "shut success" << std::endl;
    return 0;
}