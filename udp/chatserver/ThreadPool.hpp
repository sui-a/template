#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include "Log.hpp"
#include "Thread.hpp"
#include "Cond.hpp"
#include "Mutex.hpp"

// .hpp header only

namespace ThreadPoolModule
{
    using namespace ThreadModlue;
    using namespace LogModule;
    using namespace CondModule;
    using namespace MutexModule;

    static const int gnum = 5;
    template <typename T>
    class ThreadPool
    {
    private:
        void WakeUpAllThread()
        {
            LockGuard lockguard(_mutex);
            if (_sleepernum)
                _cond.Broadcast();
            LOG(LogLevel::INFO) << "�������е������߳�";
        }

        void WakeUpOne()
        {
            _cond.Signal();
            LOG(LogLevel::INFO) << "����һ�������߳�";
        }

        ThreadPool(int num = gnum) : _num(num), _isrunning(false), _sleepernum(0)
        {
            for (int i = 0; i < num; i++)
            {
                _threads.emplace_back(
                    [this]()
                    {
                        HandlerTask();
                    });
            }
        }
        void Start()
        {
            if (_isrunning)
                return;
            _isrunning = true;
            for (auto& thread : _threads)
            {
                thread.Start();
                // LOG(LogLevel::INFO) << "start new thread success: " << thread.Name();
            }
        }

        ThreadPool(const ThreadPool<T>&) = delete;
        ThreadPool<T>& operator=(const ThreadPool<T>&) = delete;

    public:
        static ThreadPool<T>* GetInstance()
        {
            if (inc == nullptr)
            {
                LockGuard lockguard(_lock);

                LOG(LogLevel::DEBUG) << "��ȡ����....";
                if (inc == nullptr)
                {
                    LOG(LogLevel::DEBUG) << "�״�ʹ�õ���, ����֮....";
                    inc = new ThreadPool<T>();
                    inc->Start();
                }
            }

            return inc;
        }
        void Stop()
        {
            if (!_isrunning)
                return;
            _isrunning = false;

            // �������е��߲�
            WakeUpAllThread();
        }
        void Join()
        {
            for (auto& thread : _threads)
            {
                thread.Join();
            }
        }
        void HandlerTask()
        {
            char name[128];
            pthread_getname_np(pthread_self(), name, sizeof(name));
            while (true)
            {
                T t;
                {
                    LockGuard lockguard(_mutex);
                    // 1. a.����Ϊ�� b. �̳߳�û���˳�
                    while (_taskq.empty() && _isrunning)
                    {
                        _sleepernum++;
                        _cond.Wait(_mutex);
                        _sleepernum--;
                    }
                    // 2. �ڲ����̱߳�����
                    if (!_isrunning && _taskq.empty())
                    {
                        LOG(LogLevel::INFO) << name << " �˳���, �̳߳��˳�&&�������Ϊ��";
                        break;
                    }

                    // һ��������
                    t = _taskq.front(); // ��q�л�ȡ���������Ѿ����߳�˽�е��ˣ�����
                    _taskq.pop();
                }
                t(); // ����������/Ҫ���ٽ����ڲ�������1 0
            }
        }
        bool Enqueue(const T& in)
        {
            if (_isrunning)
            {
                LockGuard lockguard(_mutex);
                _taskq.push(in);
                if (_threads.size() == _sleepernum)
                    WakeUpOne();
                return true;
            }
            return false;
        }
        ~ThreadPool()
        {
        }

    private:
        std::vector<Thread> _threads;
        int _num; // �̳߳��У��̵߳ĸ���
        std::queue<T> _taskq;
        Cond _cond;
        Mutex _mutex;

        bool _isrunning;
        int _sleepernum;

        // bug??
        static ThreadPool<T>* inc; // ����ָ��
        static Mutex _lock;
    };

    template <typename T>
    ThreadPool<T>* ThreadPool<T>::inc = nullptr;

    template <typename T>
    Mutex ThreadPool<T>::_lock;

}