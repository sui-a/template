#pragma once

#include <iostream>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

namespace sui
{
    class my_read
    {
    public:
        my_read(const std::string& name)
            :fifo(name)
        {


            //jin xing init
            umask(0);
            int n = mkfifo(fifo.c_str(), 0666);

            if (n == -1)
            {
                if (errno != EEXIST)
                    std::cout << "mkfife error " << std::endl;
            }

            link();
            std::cout << "link success" << std::endl;
        }
        ~my_read()
        {

            close(fd);
            stop();
        }

        void link()
        {
            while (true)
            {
                fd = open(fifo.c_str(), O_RDONLY);
                if (fd == -1)
                {
                    std::cerr << "open error" << std::endl;
                    stop();
                    exit(0);
                }
                else if (fd == 0)
                {
                    std::cout << "The write side is closed, wait  " << std::endl;
                    continue;
                }
                else {
                    break;
                }
            }
        }

        std::string work()
        {
            std::string ret_cli;
            char buff[1024] = "\0";
            while (true)
            {
                int n = read(fd, buff, sizeof(buff) - 1);
                if (n < 0)
                {
                    std::cerr << "read error" << std::endl;
                    exit(0);
                }
                else if (n > 0)
                {
                    buff[n] = '\0';
                    ret_cli = buff;
                    break;
                }
                else if (n == 0)
                {
                    link();
                }
            }
            return ret_cli;
        }

        void stop()
        {
            unlink(fifo.c_str());
        }

    private:
        int fd;
        std::string fifo;
    };
}