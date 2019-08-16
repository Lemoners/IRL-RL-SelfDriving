#ifndef _ENV_H_
#define _ENV_H_
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<unistd.h>
#include<arpa/inet.h>
#include<sys/socket.h>

struct Protocol{
    char message[30];
};

struct UsrInfo{
    float sensor[19];
    float wheel[4];
    float pos[4];
    float car[5];
    float seg[4][4];
    float current_dist;
    float total_dist;
    float rpm;
};

struct SvrInfo{
    float steer, accelerate, brake, time;
};

class Env{
    public:
        Env();
        ~Env();
        int restart();
        Protocol send();
        void send(float*, float*, float*, float*, float**, int, int, int, int, int, int, float, float, float);
        SvrInfo get();
        Protocol package(char[30]);
    private:
        int sock;
        struct sockaddr_in serv_addr;
        char message[30];
        int str_len;
        int bug;
};

#endif // _ENV_H_
