#ifndef _Strdeal_hpp_
#define _Strdeal_hpp_
#include <string>
#include <filesystem>
#include <fstream>
#include "http.hpp"
#include <iostream>


namespace sui
{
    //构造http请求字符处理类
    class http_stc_deal
    {
    public:
        //模拟流操作进行提取字符
        static bool str_divide(std::string& total, std::string& out, std::string divide)
        {
            //查错
            if(divide.empty())
                return false;


            //进行查找
            size_t pos = total.find(divide);
            if (pos != std::string::npos) 
            {
                //得到非最大值pos，即成功查找到字符串  //自动清空原有的数据
                out = total.substr(0, pos);
                
                total.erase(0, pos + divide.size());
                return true;
            }
            return false;
        }


        //解析请求  注：必须以\r\n结尾，否则解析失败  //反序列化
        static bool req_to_class(std::string& data, sui::httprequeue* outp)
        {
            //使用流操作对数据进行逐行处理
            std::string line;

            //对第一行，进行分类
            str_divide(data, line, "\r\n");

            //开始拿取数据

            {
                //拿取请求方法
                std::string line2;

                if(!sui::http_stc_deal::str_divide(line, line2, " "))
                {
                    //提取失败返回
                    return false;
                }
                outp->_req_meth = line2;

                if(!sui::http_stc_deal::str_divide(line, line2, " "))
                {
                    return false;
                }
                outp->_uri_path = line2;

                if(line.empty())
                {
                    //为空返回
                    return false;
                }
                outp->_http_version = line;
            }

            //开始拿取请求头字段
            while(!data.empty())
            {
                if(!sui::http_stc_deal::str_divide(data, line, "\r\n"))
                {
                    //有错误
                    return false;
                }
                std::string line2;
                if(!sui::http_stc_deal::str_divide(line, line2, ": "))
                {
                    return false;
                }
                //正常解析
                outp->_headers.push_back(line2);
                outp->_http_req_head[line2] = line;
            }
            return true;
        }

        //从文件中提取网页
        static bool file_read_html(std::string& path, std::string& name, std::string& out)
        {
            //拼接路文件
            std::string filename;
            filename = path + (path.back() == '/' ? "" : "/") + name;

            //检查文件是否存在
            if(!std::filesystem::exists(filename))
            {
                std::cout << "文件不存在， 文件名为 ： " << filename << std::endl; 
                //文件不存在 返回失败 交给外部进行处理
                return false;
            }

            //以只读的形式打开文件
            std::ifstream file(filename);

            //检查文件是否打开
            if(!file.is_open())
            {
                //文件打开失败 交给外部处理
                return false;
            }

            out = std::string((std::istreambuf_iterator<char>(file)), 
                            std::istreambuf_iterator<char>());

            return true;
        }
    };

}

#endif