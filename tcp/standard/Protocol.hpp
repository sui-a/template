#pragma once
#include "Socket.hpp"
#include <iostream>
#include <string>
#include <memory>
#include <jsoncpp/json/json.h>
#include <functional>

// 实现一个自定义的网络版本的计算器

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
        // "10" "20" '+' : 用空格作为分隔符
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
        // "10" "20" '+' -> 以空格作为分隔符 -> 10 20 '+'
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
    int _result; // 运算结果，无法区分清楚应答是计算结果，还是异常值
    int _code;   // 0:sucess, 1,2,3,4->不同的运算异常的情况, 约定！！！！
};

const std::string sep = "\r\n";

using func_t = std::function<Response(Request& req)>;

// 协议(基于TCP的)需要解决两个问题：
// 1. request和response必须得有序列化和反序列化功能
// 2. 必须保证，读取的时候，读到完整的请求(TCP, UDP不用考虑)
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
        return len + sep + jsonstr + sep; // 应用层封装报头
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
    // packge故意是&
    // 1. 判断报文完整性
    // 2. 如果包含至少一个完整请求，提取他， 并从移除它，方便处理下一个
    bool Decode(std::string& buffer, std::string* package)
    {
        ssize_t pos = buffer.find(sep);
        if (pos == std::string::npos)
            return false; // 让调用方继续从内核中读取数据
        std::string package_len_str = buffer.substr(0, pos);
        int package_len_int = std::stoi(package_len_str);
        // buffer 一定有长度，但是一定有完整的报文吗？
        int target_len = package_len_str.size() + package_len_int + 2 * sep.size();
        if (buffer.size() < target_len)
            return false;

        // buffer一定至少有一个完整的报文！
        *package = buffer.substr(pos + sep.size(), package_len_int);
        buffer.erase(0, target_len);
        return true;
    }
    void GetRequest(std::shared_ptr<Socket>& sock, InetAddr& client)
    {
        // 读取
        std::string buffer_queue;
        while (true)
        {
            int n = sock->Recv(&buffer_queue);
            if (n > 0)
            {
                std::string json_package;
                // 1. 解析报文，提取完整的json请求，如果不完整，就让服务器继续读取
                bool ret = Decode(buffer_queue, &json_package);
                if (!ret)
                    continue;
                // 一定拿到了一个完整的报文
                // {"x": 10, "y" : 20, "oper" : '+'} 
                // 2. 请求json串，反序列化
                Request req;
                bool ok = req.Deserialize(json_package);
                if (!ok)
                    continue;
                // 3. 一定得到了一个内部属性已经被设置了的req了.
                // 通过req->resp, 不就是要完成计算功能嘛！！业务
                Response resp = _func(req);

                // 4. 序列化
                std::string json_str = resp.Serialize();

                // 5. 添加自定义长度
                std::string send_str = Encode(json_str); // 携带长度的应答报文了"len\r\n{result:XXX, code:XX}\r\n"

                // 6. 直接发送
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