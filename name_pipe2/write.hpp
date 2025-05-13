#pragma once

#include <iostream>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>
#include <string>

namespace sui
{
    class my_write
    {
    public:
        my_write(const std::string& name)
            :fifo(name)
        {
            fd = open(fifo.c_str(), O_RDWR);
            if (fd == -1)
            {
                std::cerr << "open error" << std::endl;
            }
        }

        ~my_write()
        {
            close(fd);
        }

        bool work(const std::string& dat)
        {
            int n = write(fd, dat.c_str(), dat.size());
            if (n == -1)
            {
                std::cerr << "write error " << std::endl;
                return false;
            }
            return true;
        }


    private:
        int fd;
        std::string fifo;
    };
}