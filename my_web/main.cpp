#include <iostream>
#include "my_log.hpp"
#include "Socket.hpp"
#include "http.hpp"
#include "Strdeal.hpp"


int main()
{
    
    sui::myTcpSocket server;
    server.mySocketServerInit(8080);
    while(true)
    {
        LOG(sui::LogLevel::INFO) << "开始等待客户端";
        InetAddr client_addr;
        auto client_fd = server.socket_accept(&client_addr);
        LOG(sui::LogLevel::INFO) << "已经连接到客户端" << "[" << client_addr.StringAddr() << "] ";
        while(true)
        {
            //连接成功，进行交互
            std::string dat;
            //存储请求字段
            std::string get_http_req;
            while(1)
            {
                int ret_cli = client_fd->my_recv(dat);
                if(ret_cli == 0)
                {
                    LOG(sui::LogLevel::INFO) << "客户端已经关闭";
                    break;
                }
                else if(ret_cli < 0)
                {
                    LOG(sui::LogLevel::ERROR) << "读写发生错误: " << strerror(errno);
                    break;
                }
                
                //为正常读取
                size_t pos = dat.find("\r\n\r\n");  // 查找连续的"\n\n"
                if (pos != std::string::npos) 
                {
                    //能够找到一个完整的http请求
                    get_http_req = dat.substr(0, pos + 2);

                    dat.erase(0, pos + 4);
                    break;
                }
            }

            //请求字段存储位置
            sui::httprequeue ht_re;
            //回应字段请求位置
            sui::httpreply ht_reply;

            std::cout << ht_re << std::endl;
            
            ht_reply._http_version = "HTTP/1.1";
            ht_reply._Cache_Control = "no-cache";
            LOG(sui::LogLevel::INFO) << get_http_req;
            if(get_http_req.empty())
            {
                client_fd->Close();
                break;
            }
            //对http请求进行解析
            if(!sui::http_stc_deal::req_to_class(get_http_req, &ht_re))
            {
                //转换失败
                LOG(sui::LogLevel::ERROR) << "http请求解析失败";
                ht_reply._state_number = "404"; 
                ht_reply._state_describe = "FAILE TO REQUEUE";
                ht_reply._Content_Type = "text/html; charset=utf-8";

                std::string path = "./html";
                std::string name = "found_not.html";

                if(sui::http_stc_deal::file_read_html(path, name, ht_reply._reply_body))
                {
                    LOG(sui::LogLevel::INFO) << "读取成功";
                    ht_reply._state_number = "404"; 
                    ht_reply._state_describe = "FAILE NOT";
                    ht_reply._Content_Type = "text/html; charset=utf-8";
                }
            }
            else
            {
                if(ht_re._uri_path == "/")
                {
                    std::string path = "./html";
                    std::string name = "homepage.html";

                    if(sui::http_stc_deal::file_read_html(path, name, ht_reply._reply_body))
                    {
                        LOG(sui::LogLevel::INFO) << "读取成功";
                        ht_reply._state_number = "200"; 
                        ht_reply._state_describe = "OK";
                        ht_reply._Content_Type = "text/html; charset=utf-8";
                    }
                    else
                    {
                        LOG(sui::LogLevel::ERROR) << "读取失败";
                        ht_reply._state_number = "404"; 
                        ht_reply._state_describe = "FAILE NOT";
                        ht_reply._Content_Type = "text/html; charset=utf-8";
                    }
                }
                else if(ht_re._uri_path == "/gallery")
                {
                    std::string path = "./html";
                    std::string name = "gallery.html";

                    if(sui::http_stc_deal::file_read_html(path, name, ht_reply._reply_body))
                    {
                        LOG(sui::LogLevel::INFO) << "读取成功";
                        ht_reply._state_number = "200"; 
                        ht_reply._state_describe = "OK";
                        ht_reply._Content_Type = "text/html; charset=utf-8";
                    }
                    else
                    {
                        LOG(sui::LogLevel::ERROR) << "读取失败";
                        ht_reply._state_number = "404"; 
                        ht_reply._state_describe = "FAILE NOT";
                        ht_reply._Content_Type = "text/html; charset=utf-8";
                    }
                }
                else if(ht_re._uri_path == "/image3.jpg")
                {
                    std::string path = "./html";
                    std::string name = "image3.jpg";

                    if(sui::http_stc_deal::file_read_html(path, name, ht_reply._reply_body))
                    {
                        LOG(sui::LogLevel::INFO) << "读取成功";
                        ht_reply._state_number = "200"; 
                        ht_reply._state_describe = "OK";
                        ht_reply._Content_Type = "text/html; charset=utf-8";
                    }
                    else
                    {
                        LOG(sui::LogLevel::ERROR) << "读取失败";
                        ht_reply._state_number = "404"; 
                        ht_reply._state_describe = "FAILE NOT";
                        ht_reply._Content_Type = "text/html; charset=utf-8";
                    }
                }
                else
                {
                    LOG(sui::LogLevel::DEBUG) << "请求空路径";
                    ht_reply._state_number = "404"; 
                    ht_reply._state_describe = "FAILE NOT";
                    ht_reply._Content_Type = "text/html; charset=utf-8";

                    std::string path = "./html";
                    std::string name = "found_not.html";

                    if(sui::http_stc_deal::file_read_html(path, name, ht_reply._reply_body))
                    {
                        LOG(sui::LogLevel::INFO) << "读取成功";
                        ht_reply._state_number = "404"; 
                        ht_reply._state_describe = "FAILE NOT";
                        ht_reply._Content_Type = "text/html; charset=utf-8";
                    }
                }
            }

            client_fd->my_send(ht_reply.serialization());
            client_fd->Close();
            LOG(sui::LogLevel::INFO) << "成功回应";
            break;
        }
    }
    return 0;
}



// int main()
// {
//     std::string http_re_str = "GET / HTTP/1.1\r\nHost: 127.0.0.1:8080\r\nUser-Agent: Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:130.0) Gecko/20100101 Firefox/130.0\r\nAccept: text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image/png,image/svg+xml,*/*;q=0.8\r\nAccept-Language: zh-CN,zh;q=0.8,zh-TW;q=0.7,zh-HK;q=0.5,en-US;q=0.3,en;q=0.2\r\nAccept-Encoding: gzip, deflate, br, zstd\r\nConnection: keep-alive\r\nUpgrade-Insecure-Requests: 1\r\nSec-Fetch-Dest: document\r\nSec-Fetch-Mode: navigate\r\nSec-Fetch-Site: none\r\nSec-Fetch-User: ?1\r\nPriority: u=0, i\r\n";
//     sui::httpreply aaa;
//     aaa._http_version = "HTTP/1.1";
//     aaa._state_number = "200";
//     aaa._state_describe = "ok";

//     aaa._Content_Type = "text/html; charset=utf-8";
//     aaa._Cache_Control = "no-cache";

    

    
//     std::string path = "./html";
//     std::string name = "hutao_web.html";

//     //从文件中提取网页信息
//     std::string sss;

//     if(sui::http_stc_deal::file_read_html(path, name, aaa._reply_body))
//     {
//         std::cout << "读取成功" << std::endl << std::endl;
//     }
//     else
//     {
//         std::cout << "读取失败" << std::endl;
//     }

//     std::cout << aaa.serialization();

//     return 0;
// }