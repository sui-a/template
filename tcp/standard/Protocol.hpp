#pragma once
#include "Socket.hpp"
#include <iostream>
#include <string>
#include <memory>
#include <jsoncpp/json/json.h>
#include <functional>

// ʵ��һ���Զ��������汾�ļ�����

using namespace SocketModule;

// content_len jsonstring
// 50\r\n{"x": 10, "y" : 20, "oper" : '+'}\r\n
// 50
// {"x": 10, "y" : 20, "oper" : '+'}
class Request
{
public:
    Request()
    {
    }
    Request(int x, int y, char oper) : _x(x), _y(y), _oper(oper)
    {
    }
    std::string Serialize()
    {
        // _x = 10 _y = 20, _oper = '+'
        // "10" "20" '+' : �ÿո���Ϊ�ָ���
        Json::Value root;
        root["x"] = _x;
        root["y"] = _y;
        root["oper"] = _oper; // ?

        Json::FastWriter writer;
        std::string s = writer.write(root);
        return s;
    }

    // {"x": 10, "y" : 20, "oper" : '+'}
    bool Deserialize(std::string& in)
    {
        // "10" "20" '+' -> �Կո���Ϊ�ָ��� -> 10 20 '+'
        Json::Value root;
        Json::Reader reader;
        bool ok = reader.parse(in, root);
        if (ok)
        {
            _x = root["x"].asInt();
            _y = root["y"].asInt();
            _oper = root["oper"].asInt(); //?
        }
        return ok;
    }
    ~Request() {}
    int X() { return _x; }
    int Y() { return _y; }
    char Oper() { return _oper; }
private:
    int _x;
    int _y;
    char _oper; // + - * / % -> _x _oper _y -> 10 + 20
};

// server -> client
class Response
{
public:
    Response() {}
    Response(int result, int code) : _result(result), _code(code)
    {
    }
    std::string Serialize()
    {
        Json::Value root;
        root["result"] = _result;
        root["code"] = _code;

        Json::FastWriter writer;
        return writer.write(root);
    }
    bool Deserialize(std::string& in)
    {
        Json::Value root;
        Json::Reader reader;
        bool ok = reader.parse(in, root);
        if (ok)
        {
            _result = root["result"].asInt();
            _code = root["code"].asInt();
        }
        return ok;
    }
    ~Response() {}
    void SetResult(int res)
    {
        _result = res;
    }
    void SetCode(int code)
    {
        _code = code;
    }
private:
    int _result; // ���������޷��������Ӧ���Ǽ������������쳣ֵ
    int _code;   // 0:sucess, 1,2,3,4->��ͬ�������쳣�����, Լ����������
};

const std::string sep = "\r\n";

using func_t = std::function<Response(Request& req)>;

// Э��(����TCP��)��Ҫ����������⣺
// 1. request��response����������л��ͷ����л�����
// 2. ���뱣֤����ȡ��ʱ�򣬶�������������(TCP, UDP���ÿ���)
class Protocol
{
public:
    Protocol(func_t func) :_func(func)
    {
    }
    std::string Encode(const std::string& jsonstr)
    {
        // 50\r\n{"x": 10, "y" : 20, "oper" : '+'}\r\n
        std::string len = std::to_string(jsonstr.size());
        return len + sep + jsonstr + sep; // Ӧ�ò��װ��ͷ
    }

    // 50\r\n{"x": 10, "y" : 20, "oper" : '+'}\r\n
    // 5
    // 50
    // 50\r
    // 50\r\n
    // 50\r\n{"x": 10, "
    // 50\r\n{"x": 10, "y" : 20, "oper" : '+'}\r\n
    // 50\r\n{"x": 10, "y" : 20, "oper" : '+'}\r\n50\r\n{"x": 10, "y" : 20, "ope
    //.....
    // packge������&
    // 1. �жϱ���������
    // 2. �����������һ������������ȡ���� �����Ƴ��������㴦����һ��
    bool Decode(std::string& buffer, std::string* package)
    {
        ssize_t pos = buffer.find(sep);
        if (pos == std::string::npos)
            return false; // �õ��÷��������ں��ж�ȡ����
        std::string package_len_str = buffer.substr(0, pos);
        int package_len_int = std::stoi(package_len_str);
        // buffer һ���г��ȣ�����һ���������ı�����
        int target_len = package_len_str.size() + package_len_int + 2 * sep.size();
        if (buffer.size() < target_len)
            return false;

        // bufferһ��������һ�������ı��ģ�
        *package = buffer.substr(pos + sep.size(), package_len_int);
        buffer.erase(0, target_len);
        return true;
    }
    void GetRequest(std::shared_ptr<Socket>& sock, InetAddr& client)
    {
        // ��ȡ
        std::string buffer_queue;
        while (true)
        {
            int n = sock->Recv(&buffer_queue);
            if (n > 0)
            {
                std::string json_package;
                // 1. �������ģ���ȡ������json������������������÷�����������ȡ
                bool ret = Decode(buffer_queue, &json_package);
                if (!ret)
                    continue;
                // һ���õ���һ�������ı���
                // {"x": 10, "y" : 20, "oper" : '+'} 
                // 2. ����json���������л�
                Request req;
                bool ok = req.Deserialize(json_package);
                if (!ok)
                    continue;
                // 3. һ���õ���һ���ڲ������Ѿ��������˵�req��.
                // ͨ��req->resp, ������Ҫ��ɼ��㹦�����ҵ��
                Response resp = _func(req);

                // 4. ���л�
                std::string json_str = resp.Serialize();

                // 5. ����Զ��峤��
                std::string send_str = Encode(json_str); // Я�����ȵ�Ӧ������"len\r\n{result:XXX, code:XX}\r\n"

                // 6. ֱ�ӷ���
                sock->Send(send_str);
            }
            else if (n == 0)
            {
                LOG(LogLevel::INFO) << "client:" << client.StringAddr() << "Quit!";
                break;
            }
            else
            {
                LOG(LogLevel::WARNING) << "client:" << client.StringAddr() << ", recv error";
                break;
            }
        }
    }
    ~Protocol()
    {
    }

private:
    // Request _req;
    // Response _resp;
    func_t _func;
};