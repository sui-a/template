#pragma once

#include <iostream>
#include <cstdio>
#include <string>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>

const int gdefaultid = -1;
const int gsize = 4096;
const std::string pathname = ".";
const int projid = 0x66;
const int gmode = 0666;
#define CREATER "creater"
#define USER "user"

#define ERR_EXIT(m)         \
    do                      \
    {                       \
        perror(m);          \
        exit(EXIT_FAILURE); \
    } while (0)

class Shm
{
private:
    //使用shmget系统调用创建新的共享内存区块
    void CreateHelper(int flg)
    {
        printf("key: 0x%x\n", _key);
        _shmid = shmget(_key, _size, flg);
        if (_shmid < 0)
        {
            ERR_EXIT("shmget");
        }
        printf("shmid: %d\n", _shmid);
    }
    void Create()
    {
        //创建全新的，已经创建则报错 gmode为权限
        CreateHelper(IPC_CREAT | IPC_EXCL | gmode);
    }
    //进行连接 将本地虚拟内存开辟一样大小的区块与共享内存进行映射
    void Attach()
    {
        //系统调用，返回映射完成的虚拟地址
        _start_mem = shmat(_shmid, nullptr, 0);
        if ((long long)_start_mem < 0)
        {
            ERR_EXIT("shmat");
        }
        printf("attach success\n");
    }
    //解除映射 《不影响销毁》
    void Detach()
    {
        //解除映射系统调用 即从地址空间中分离
        int n = shmdt(_start_mem);
        if (n == 0)
        {
            printf("detach success\n");
        }
    }

    void Get()
    {
        //共享内存存在则直接返回，不存在则创建
        CreateHelper(IPC_CREAT);
    }

    //进行销毁，在析构函数中使用
    void Destroy()
    {
        //先解除映射
        Detach();
        //判断是否是创建者，创建者则销毁内存块
        if (_usertype == CREATER)
        {
            //shmctl为控制内存的系统调用，IPC_RMID为销毁命令
            int n = shmctl(_shmid, IPC_RMID, nullptr);
            if (n > 0)
            {
                printf("shmctl delete shm: %d success!\n", _shmid);
            }
            else
            {
                ERR_EXIT("shmctl");
            }
        }
    }

public:
    //构造函数
    Shm(const std::string& pathname, int projid, const std::string& usertype)
        : _shmid(gdefaultid),
        _size(gsize),
        _start_mem(nullptr),
        _usertype(usertype)
    {
        _key = ftok(pathname.c_str(), projid);
        if (_key < 0)
        {
            ERR_EXIT("ftok");
        }
        //判断使用类型来决定怎么创建
        if (_usertype == CREATER)
            Create();
        else if (_usertype == USER)
            Get();
        else
        {
        }
        //进行映射
        Attach();
    }
    //获取当前地址
    void* VirtualAddr()
    {
        printf("VirtualAddr: %p\n", _start_mem);
        return _start_mem;
    }
    //返回大小
    int Size()
    {
        return _size;
    }
    //获取状态
    void Attr()
    {
        struct shmid_ds ds;
        int n = shmctl(_shmid, IPC_STAT, &ds);
        printf("shm_segsz: %ld\n", ds.shm_segsz);
        printf("key: 0x%x\n", ds.shm_perm.__key);
    }

    ~Shm()
    {
        //只有creater类型销毁是因为进程结束自动去掉连接，和Detach效果一样
        std::cout << _usertype << std::endl;
        if (_usertype == CREATER)
            Destroy();
    }

private:
    //id
    int _shmid;
    //唯一键值
    key_t _key;
    //共享内存大小
    int _size;
    //虚拟内存地址
    void* _start_mem;
    //启动方式
    std::string _usertype;
};