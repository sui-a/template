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
    //ʹ��shmgetϵͳ���ô����µĹ����ڴ�����
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
        //����ȫ�µģ��Ѿ������򱨴� gmodeΪȨ��
        CreateHelper(IPC_CREAT | IPC_EXCL | gmode);
    }
    //�������� �����������ڴ濪��һ����С�������빲���ڴ����ӳ��
    void Attach()
    {
        //ϵͳ���ã�����ӳ����ɵ������ַ
        _start_mem = shmat(_shmid, nullptr, 0);
        if ((long long)_start_mem < 0)
        {
            ERR_EXIT("shmat");
        }
        printf("attach success\n");
    }
    //���ӳ�� ����Ӱ�����١�
    void Detach()
    {
        //���ӳ��ϵͳ���� ���ӵ�ַ�ռ��з���
        int n = shmdt(_start_mem);
        if (n == 0)
        {
            printf("detach success\n");
        }
    }

    void Get()
    {
        //�����ڴ������ֱ�ӷ��أ��������򴴽�
        CreateHelper(IPC_CREAT);
    }

    //�������٣�������������ʹ��
    void Destroy()
    {
        //�Ƚ��ӳ��
        Detach();
        //�ж��Ƿ��Ǵ����ߣ��������������ڴ��
        if (_usertype == CREATER)
        {
            //shmctlΪ�����ڴ��ϵͳ���ã�IPC_RMIDΪ��������
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
    //���캯��
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
        //�ж�ʹ��������������ô����
        if (_usertype == CREATER)
            Create();
        else if (_usertype == USER)
            Get();
        else
        {
        }
        //����ӳ��
        Attach();
    }
    //��ȡ��ǰ��ַ
    void* VirtualAddr()
    {
        printf("VirtualAddr: %p\n", _start_mem);
        return _start_mem;
    }
    //���ش�С
    int Size()
    {
        return _size;
    }
    //��ȡ״̬
    void Attr()
    {
        struct shmid_ds ds;
        int n = shmctl(_shmid, IPC_STAT, &ds);
        printf("shm_segsz: %ld\n", ds.shm_segsz);
        printf("key: 0x%x\n", ds.shm_perm.__key);
    }

    ~Shm()
    {
        //ֻ��creater������������Ϊ���̽����Զ�ȥ�����ӣ���DetachЧ��һ��
        std::cout << _usertype << std::endl;
        if (_usertype == CREATER)
            Destroy();
    }

private:
    //id
    int _shmid;
    //Ψһ��ֵ
    key_t _key;
    //�����ڴ��С
    int _size;
    //�����ڴ��ַ
    void* _start_mem;
    //������ʽ
    std::string _usertype;
};