#ifndef _http_hpp_
#define _http_hpp_

#include <string>
#include <vector>
#include <map>


namespace sui
{
    class httprequeue
    {
    public:
        httprequeue() = default;

        httprequeue(std::string& http_input);

        ~httprequeue() = default;

    public:
        //请求方法
        std::string _req_meth;
        //http版本
        std::string _http_version;
        //uri
        std::string _uri_path;
        //存储头
        std::vector<std::string> _headers;
        //存储请求头字段
        std::map<std::string, std::string> _http_req_head;
    };

    //输出请求
    std::ostream& operator<<(std::ostream& out, sui::httprequeue& httpout);


    //回应类
    class httpreply
    {
    public:
        //初始化
        httpreply() = default;

        ~httpreply() = default;

        //序列化
        std::string serialization();

    public:
        //http版本
        std::string _http_version;
        //状态码
        std::string _state_number;
        //描述
        std::string _state_describe;

        //响应头定义
        //响应体类型
        std::string _Content_Type;
        //重定向目标uri
        std::string _Location;
        //缓存策略
        std::string _Cache_Control;
        //设置客户端cookie
        std::string _Set_Cookie;

        //响应体
        std::string _reply_body;
    };
}

#endif