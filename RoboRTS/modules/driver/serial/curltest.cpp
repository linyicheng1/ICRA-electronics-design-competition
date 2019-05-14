//
// Created by hzr on 18-8-18.
//

#include <curl/curl.h>
#include <curl/easy.h>
#include <curl/curlbuild.h>
#include <sstream>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/detail/ptree_implementation.hpp>
#include <boost/foreach.hpp>
#include <boost/progress.hpp>
#include <boost/date_time.hpp>
#include <string>
#include <ctime>
#include <vector>
#include "curltest.h"
#ifdef _WINDOWS
#include <windows.h>
#else
#include <unistd.h>
#define Sleep(x) usleep((x)*1000)
#endif


using namespace std;
using namespace boost::property_tree;
using namespace boost::gregorian;
using namespace boost;



static size_t OnWriteData(void* buffer, size_t size, size_t nmemb, void* userdata)
{
    string* str = dynamic_cast<string*>((string *)userdata);
    if( NULL == str || NULL == buffer )
    {
        return -1;
    }

    char* pData = (char*)buffer;
    str->append(pData, size * nmemb);
    return nmemb;
}

void get(string &out)
{
    // 基于当前系统的当前日期/时间
    time_t now = time(0);
    char* dt = ctime(&now);
    cout << dt << "-------------------------------------" << endl;

    /*HTTP GET json data*/
    // string out;

    void* curl = curl_easy_init();
    // 设置URL
    //  curl_easy_setopt(curl,CURLOPT_NOBODY,true);
    curl_easy_setopt(curl, CURLOPT_URL, "https://api.bmob.cn/1/classes/Order?where={\"status\":1}&order=createdAt");
    // 设置接收数据的处理函数和存放变量
    // curl_easy_setopt(curl,CURLOPT_READFUNCTION,readData);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, OnWriteData);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &out);

    //设置请求头
    struct  curl_slist *header_list = NULL;
    header_list = curl_slist_append(header_list,"X-Bmob-Application-Id: 139af1dbfb4de4b1917866c6a77a0b53");
    header_list = curl_slist_append(header_list,"X-Bmob-REST-API-Key: 9e89b3fc99d966fca704fd7c3b2a8ceb");


    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, header_list);

    // 执行HTTP GET操作
    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
    }

    if(res!=0)
    {
        curl_slist_free_all(header_list);

    }


    curl_easy_cleanup(curl);
}

void getsorder(string &out)
{
    // 基于当前系统的当前日期/时间
    time_t now = time(0);
    char* dt = ctime(&now);
    cout << dt << "-------------------------------------" << endl;

    /*HTTP GET json data*/
    // string out;

    void* curl = curl_easy_init();
    // 设置URL
    //  curl_easy_setopt(curl,CURLOPT_NOBODY,true);
    curl_easy_setopt(curl, CURLOPT_URL, "https://api.bmob.cn/1/classes/Order?order=-createdAt");
    // 设置接收数据的处理函数和存放变量
    // curl_easy_setopt(curl,CURLOPT_READFUNCTION,readData);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, OnWriteData);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &out);

    //设置请求头
    struct  curl_slist *header_list = NULL;
    header_list = curl_slist_append(header_list,"X-Bmob-Application-Id: 139af1dbfb4de4b1917866c6a77a0b53");
    header_list = curl_slist_append(header_list,"X-Bmob-REST-API-Key: 9e89b3fc99d966fca704fd7c3b2a8ceb");


    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, header_list);

    // 执行HTTP GET操作
    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
    }

    if(res!=0)
    {
        curl_slist_free_all(header_list);

    }


    curl_easy_cleanup(curl);
}

void post(int foodID,int goal)
{
    CURLcode res; //easy_handle定义的一些错误码
    CURL *curl;
    curl =curl_easy_init();

    char szJsonData[1024];
    memset(szJsonData, 0, sizeof(szJsonData));

    std::stringstream ss;
    ss<<foodID;
    string detail;
    string title;
    switch (foodID)
    {
        case 1:
            title="卤蛋";
            break;
        case 2:
            title="火腿肠";
            break;
        case 3:
            title="矿泉水";
            break;
        case 4:
            title="可乐";
            break;
        case 5:
            title="卫生纸";
            break;
        case 6:
            title="口香糖";
            break;
        default:
            break;

    }
    switch (goal)
    {
        case 1:
            detail="1号";
            break;
        case 2:
            detail="2号";
            break;
        case 3:
            detail="3号";
            break;
        default:
            break;

    }


    std::string strJson = "{";
    strJson += "\"title\" : \"";
    strJson += title;
    strJson += "\",\"status\" : 0,";
    strJson += "\"quantity\" : 2,";
    strJson += "\"FoodID\" :";
    strJson += ss.str();
    strJson += ",\"DetailAddress\":\"";
    strJson +=detail;
    strJson += "\"}";
    strcpy(szJsonData, strJson.c_str());


    string response;

    curl_easy_setopt(curl, CURLOPT_POST, 1); // post req
    curl_easy_setopt(curl, CURLOPT_URL,"https://api.bmob.cn/1/classes/Order");           //指定url
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, szJsonData);           //指定post内容
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, false); // if want to use https
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);
    struct  curl_slist *header_list = NULL;
    header_list = curl_slist_append(header_list,"X-Bmob-Application-Id: 139af1dbfb4de4b1917866c6a77a0b53");
    header_list = curl_slist_append(header_list,"X-Bmob-REST-API-Key: 9e89b3fc99d966fca704fd7c3b2a8ceb");
    header_list = curl_slist_append(header_list,"Content-Type: application/json");


    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, header_list);                       //设置协议头

    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, OnWriteData);          //绑定相应
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);        //绑定响应内容的地址
    res = curl_easy_perform(curl);                          //执行请求
    if (res != CURLE_OK)
        cerr << "curl_easy_perform() failed: " + string(curl_easy_strerror(res)) << endl;
    else
        cout << response << endl;
}

void put(string &urlid,int status)
{

    CURLcode res; //easy_handle定义的一些错误码
    CURL *curl;
    curl =curl_easy_init();

    char szJsonData[1024];
    char szaddressJson[1024];
    memset(szJsonData, 0, sizeof(szJsonData));

    std::stringstream ss;
    ss<<status;

    std::string strJson = "{";
    strJson += "\"status\" : ";
    strJson += ss.str();
    strJson += "}";
    strcpy(szJsonData, strJson.c_str());


    string response;
    string urlbody="https://api.bmob.cn/1/classes/Order/";
    string url=urlbody+urlid;
    cout<<url<<endl;

    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
    curl_easy_setopt(curl, CURLOPT_URL,url.c_str());           //指定url
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, szJsonData);           //指定post内容
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, false); // if want to use https
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);
    struct  curl_slist *header_list = NULL;
    header_list = curl_slist_append(header_list,"X-Bmob-Application-Id: 139af1dbfb4de4b1917866c6a77a0b53");
    header_list = curl_slist_append(header_list,"X-Bmob-REST-API-Key: 9e89b3fc99d966fca704fd7c3b2a8ceb");
    header_list = curl_slist_append(header_list,"Content-Type: application/json");


    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, header_list);                       //设置协议头
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, OnWriteData);          //绑定相应
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);        //绑定响应内容的地址
    res = curl_easy_perform(curl);                          //执行请求
    if (res != CURLE_OK)
        cerr << "curl_easy_perform() failed: " + string(curl_easy_strerror(res)) << endl;
    else
        cout << response << endl;
}

int checkStatus(string &id,string &goal,int &orderNum)//将空闲模式改为赶路模式
{
    string out;
    get( out);
    if(!out.empty())
    {
        //  cout << out << endl;
    }else{
        cout << "resp empty" << endl;
        return 0;
    }

    /*Parse json data*/
    ptree pt,p1,p2;                       //define property_tree object
    // vector<string> vecStr;
    std::stringstream ss(out);
    cout<<ss.str();
    try {
        read_json(ss, pt);          //parse json
    } catch (ptree_error & e) {
        cout<<e.what()<<endl;
        return 2;
    }
    p1 = pt.get_child("results");
    ptree::iterator it = p1.begin();
    if( it != p1.end())
        p2 = it->second;
    else
        return 0;
    int status=p2.get<int>("status");

    if(status==1)//修改成2
    {
        id=p2.get<string>("objectId");
        goal=p2.get<string>("DetailAddress");
        orderNum=p2.get<int>("orderID");
        put(id,2);
        return 1;

    }
    else
    {
        return 0;
    }
}

void getorder(ORDER order[],int &count)
{
    count=0;
    string out;
    getsorder( out);
    if(!out.empty())
    {
        //  cout << out << endl;
    }else{
        cout << "resp empty" << endl;
        count=0;
        return ;
    }

    /*Parse json data*/
    ptree pt,p1,p2;                       //define property_tree object
    // vector<string> vecStr;
    std::stringstream ss(out);
    cout<<ss.str();
    try {
        read_json(ss, pt);          //parse json
    } catch (ptree_error & e) {
        cout<<e.what()<<endl;
        count=0;
        return;
    }
    p1 = pt.get_child("results");
    for (ptree::iterator it = p1.begin(); it != p1.end()&&count<10; ++it)
    {
        p2 = it->second; //first为空
        order[count].foodID=p2.get<int>("FoodID");
        order[count].status=p2.get<int>("status");
        string goal=p2.get<string>("DetailAddress");
        order[count].goal=goal[0]-'0';
        order[count].ordernum=p2.get<int>("orderID");

        count++;

    }
    return;
}//获取订单信息

void arrived(std::string &id)
{
    put(id,3);
}