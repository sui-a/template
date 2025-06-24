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

// ����������ػ����̻��ķ���
void Daemon(int nochdir, int noclose)
{
    // 1. ����IO���ӽ����˳�����ص��ź�
    signal(SIGPIPE, SIG_IGN);
    signal(SIGCHLD, SIG_IGN); // SIG_DFL

    // 2. ������ֱ�ӽ���
    if (fork() > 0)
        exit(0);

    // 3. ֻ�����ӽ���,�¶���,�����̾���1
    setsid(); // ��Ϊһ�������ĻỰ

    if (nochdir == 0) // ���Ľ��̵Ĺ���·��������Ϊʲô����
        chdir("/");

    // 4. ���ɿ�����ʾ�������̣�stdin��stdout��stderr������.
    //  �ػ����̣����Ӽ������룬Ҳ����Ҫ����ʾ����ӡ
    //  ��/dev/null, �ض����׼���룬��׼�������׼����/dev/null
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
