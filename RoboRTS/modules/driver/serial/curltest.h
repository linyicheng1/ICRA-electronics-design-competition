//
// Created by hzr on 18-8-18.
//

#ifndef RRTS_CURLTEST_H
#define RRTS_CURLTEST_H

#include <string>
typedef struct
{
    int goal;//目的地
    int foodID;//食物的ID
    int status;//订单的状态
    int ordernum;
}ORDER;
void arrived(std::string &id);
int checkStatus(std::string &id,std::string &goal,int &orderNum);//将空闲模式改为赶路模式
void post(int foodID,int goal);
void getorder(ORDER order[],int &count);


#endif //RRTS_CURLTEST_H