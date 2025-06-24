#pragma once

#include <iostream>
#include <string>
#include <cstdio>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "Log.hpp"
#include "Common.hpp"

using namespace LogModule;

const std::string dev = "/dev/null";

// 将服务进行守护进程化的服务
void Daemon(int nochdir, int noclose)
{
    // 1. 忽略IO，子进程退出等相关的信号
    signal(SIGPIPE, SIG_IGN);
    signal(SIGCHLD, SIG_IGN); // SIG_DFL

    // 2. 父进程直接结束
    if (fork() > 0)
        exit(0);

    // 3. 只能是子进程,孤儿了,父进程就是1
    setsid(); // 成为一个独立的会话

    if (nochdir == 0) // 更改进程的工作路径？？？为什么？？
        chdir("/");

    // 4. 依旧可能显示器，键盘，stdin，stdout，stderr关联的.
    //  守护进程，不从键盘输入，也不需要向显示器打印
    //  打开/dev/null, 重定向标准输入，标准输出，标准错误到/dev/null
    if (noclose == 0)
    {
        int fd = ::open(dev.c_str(), O_RDWR);
        if (fd < 0)
        {
            LOG(LogLevel::FATAL) << "open " << dev << " errno";
            exit(OPEN_ERR);
        }
        else
        {
            dup2(fd, 0);
            dup2(fd, 1);
            dup2(fd, 2);
            close(fd);
        }
    }
}
