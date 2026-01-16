#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <zmq.hpp>
#include <string>
#include <iostream>
#include <thread>
#include "nlohmann/json.hpp"
#include <vector>

void RobotSwitch(zmq::socket_t &pub,bool value)
{
    nlohmann::ordered_json cmd_switch;
    cmd_switch["Switch"]=value;
    pub.send(zmq::buffer("Switch " + cmd_switch.dump()));
}

nlohmann::ordered_json GetRobotState(zmq::socket_t &sub)
{
    nlohmann::ordered_json state_json;
    zmq::message_t msg;
    sub.recv(msg);
    std::string state(static_cast<char*>(msg.data()), msg.size());
    auto pos = state.find(' ');
    std::string topic = state.substr(0, pos);
    if ("State"==topic)
    {
        state_json=nlohmann::json::parse(state.substr(pos + 1));
    }
    return state_json;
}

void ClearHistoricalData(zmq::socket_t &sub)
{
    zmq::message_t msg;
    while (sub.recv(msg, zmq::recv_flags::dontwait)) {
        usleep(200);
    }
}

void SetRobotJoint(zmq::socket_t &pub,std::vector<double> &joint0,std::vector<double> &joint1,double time)
{
    nlohmann::ordered_json data;
    data["Robot0"]["time"]=time;
    data["Robot0"]["joint"]=joint0;
    data["Robot1"]["time"]=time;
    data["Robot1"]["joint"]=joint1;
    pub.send(zmq::buffer("Joint " + data.dump()));
}

void SetRobotJoint(zmq::socket_t &pub,std::vector<double> &joint,double time)
{
    nlohmann::ordered_json data;
    data["Robot0"]["time"]=time;
    data["Robot0"]["joint"]=joint;
    pub.send(zmq::buffer("Joint " + data.dump()));
}

void SetRobotCartesian(zmq::socket_t &pub,std::vector<double> &cartesian,double time)
{
    nlohmann::ordered_json data;
    data["Robot0"]["time"]=time;
    data["Robot0"]["cartesian"]=cartesian;
    pub.send(zmq::buffer("Cartesian " + data.dump()));
}

int main(int argc, char *argv[])
{	
    zmq::context_t context(1);
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    zmq::socket_t subscriber(context, zmq::socket_type::sub);
    publisher.set(zmq::sockopt::sndhwm, 0);  // 0 表示无限小队列，但行为是：不能缓存
    publisher.set(zmq::sockopt::immediate, 1);  // SUB 未连接时直接丢弃
    publisher.bind("tcp://*:8001");
    subscriber.connect("tcp://192.168.0.99:8000");
    subscriber.set(zmq::sockopt::subscribe, "");
    printf("RobotSwitch\n");
    sleep(1);
    RobotSwitch(publisher,true);
    sleep(1);
    printf("GetRobotState\n");
    auto state=GetRobotState(subscriber);
    printf("=====\n");
    // std::cout << state.dump(4) << std::endl;
    std::vector<double> roobt0_joint=state["Robot0"]["Joint"].get<std::vector<double>>();
    //std::vector<double> roobt1_joint=state["Robot1"]["Joint"].get<std::vector<double>>();
    std::vector<double> roobt0_cartesian=state["Robot0"]["Cartesian"].get<std::vector<double>>();
    //std::vector<double> roobt1_cartesian=state["Robot1"]["Cartesian"].get<std::vector<double>>();

    std::vector<double> targt0_joint=roobt0_joint;
    double time=0;
    double dt=0.01;
    double A=1;double f=0.1;
    printf("loop\n");
    ClearHistoricalData(subscriber);
    while (1)
    {
        auto state=GetRobotState(subscriber);//阻塞等待接收，兼有定时功能,定时周期为数据发布周期
        
        targt0_joint[6]=A*cos(3.14*2*f*time)-A+roobt0_joint[6];
        
        SetRobotJoint(publisher,targt0_joint,time);
        printf("%f,%f\n",targt0_joint[6],state["Robot0"]["Joint"].get<std::vector<double>>()[6]);
        time+=dt;
    }
    printf("RobotSwitch\n");
    RobotSwitch(publisher,false);

	return 0;
}