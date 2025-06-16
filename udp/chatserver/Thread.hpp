#ifndef _THREAD_H_
#define _THREAD_H_

#include <iostream>
#include <string>
#include <pthread.h>
#include <cstdio>
#include <cstring>
#include <functional>

namespace ThreadModlue
{
    static uint32_t number = 1; // bug

    class Thread
    {
        using func_t = std::function<void()>; // ��ʱ����д����ȫ����
    private:
        void EnableDetach()
        {
            _isdetach = true;
        }
        void EnableRunning()
        {
            _isrunning = true;
        }
        static void* Routine(void* args) // �������ڵĳ�Ա������Ĭ�ϰ���thisָ�룡
        {
            Thread* self = static_cast<Thread*>(args);
            self->EnableRunning();
            if (self->_isdetach)
                self->Detach();
            pthread_setname_np(self->_tid, self->_name.c_str());
            self->_func(); // �ص�����

            return nullptr;
        }
        // bug
    public:
        Thread(func_t func)
            : _tid(0),
            _isdetach(false),
            _isrunning(false),
            res(nullptr),
            _func(func)
        {
            _name = "thread-" + std::to_string(number++);
        }
        void Detach()
        {
            if (_isdetach)
                return;
            if (_isrunning)
                pthread_detach(_tid);
            EnableDetach();
        }

        bool Start()
        {
            if (_isrunning)
                return false;
            int n = pthread_create(&_tid, nullptr, Routine, this);
            if (n != 0)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        bool Stop()
        {
            if (_isrunning)
            {
                int n = pthread_cancel(_tid);
                if (n != 0)
                {
                    return false;
                }
                else
                {
                    _isrunning = false;
                    return true;
                }
            }
            return false;
        }
        void Join()
        {
            if (_isdetach)
            {
                return;
            }
            int n = pthread_join(_tid, &res);
            if (n != 0)
            {
            }
            else
            {
            }
        }
        pthread_t Id()
        {
            return _tid;
        }
        ~Thread()
        {
        }

    private:
        pthread_t _tid;
        std::string _name;
        bool _isdetach;
        bool _isrunning;
        void* res;
        func_t _func;
    };
}

#endif