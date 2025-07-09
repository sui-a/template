#include "http.hpp"
#include "Strdeal.hpp"
#include <iostream>

namespace sui
{

    //使用字符串进行初始化
    httprequeue::httprequeue(std::string& http_input)
    {
        sui::http_stc_deal::req_to_class(http_input, this);
    }

    //输出请求
    std::ostream& operator<<(std::ostream& out, sui::httprequeue& httpout)
    {
        out << httpout._req_meth << " " << httpout._uri_path << " " << httpout._http_version << "\r\n";
        
        for(int i = 0; i < httpout._headers.size(); i++)
        {
            out << httpout._headers[i] << ": " <<httpout._http_req_head[httpout._headers[i]] << "\r\n";
        }
        return out;
    }


    //序列化
    std::string httpreply::serialization()
    {
        std::string http_reply;
        http_reply += _http_version + " " + _state_number + " " + _state_describe + "\r\n";

        if(!_Content_Type.empty())
        {
            //如果不为空则添加上去
            http_reply += "Content-Type: " + _Content_Type + "\r\n";
        }

        http_reply += "Content-Length: " + std::to_string(_reply_body.size()) + "\r\n";


        if(!_Location.empty())
        {
            //如果不为空则添加上去
            http_reply += "Location: " + _Location + "\r\n";
        }

        if(!_Cache_Control.empty())
        {
            //如果不为空则添加上去
            _Cache_Control += "Cache-Control: " + _Cache_Control + "\r\n";
        }

        if(!_Set_Cookie.empty())
        {
            //如果不为空则添加上去
            http_reply += "Set-Cookie: " + _Set_Cookie + "\r\n";
        }

        http_reply += "\r\n";

        if(!_reply_body.empty())
        {
            http_reply += _reply_body;
        }

        return http_reply;
    }




}