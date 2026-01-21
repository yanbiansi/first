#include "HYYRobotInterface.h"
#include <zmq.hpp>
#include <string>
#include <iostream>
#include <thread>
#include "nlohmann/json.hpp"
#include <vector>
#include <atomic>
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief 插件入库函数,用于实现插件初始化(被控制系初始函数调用),该函数要求非阻塞，如阻塞需要开线程运行
 */
extern void PluginMain();
#ifdef __cplusplus
}
#endif

#define CYCLIE 10

static zmq::context_t context(1);
static zmq::socket_t publisher(context, zmq::socket_type::pub);
static zmq::socket_t subscriber(context, zmq::socket_type::sub);
std::thread* pub_th=nullptr;
std::thread* sub_th=nullptr;

static std::atomic<bool> handshake_ok{false};

static void publisher_loop()
{
    HYYRobotBase::RTimer timer;
    HYYRobotBase::initUserTimer(&timer,0,CYCLIE);//10ms
    nlohmann::ordered_json data;
    printf("start publisher_loop\n");
    while (true)
    {
        HYYRobotBase::userTimer(&timer);
        // 握手时候不要发状态，否则高频率的state会让客户端读不到握手消息
        if (!handshake_ok.load())
        {
            continue;
        }
        int rn=HYYRobotBase::robot_getNUM();
        for (int i=0;i<rn;i++)
        {
            data[std::string("Robot")+std::to_string(i)]["MoveState"]=HYYRobotBase::get_robot_move_state(i);
            data[std::string("Robot")+std::to_string(i)]["PowerState"]=HYYRobotBase::GetRobotPowerState(i);
            int dof=HYYRobotBase::robot_getDOF(i);
            std::vector<double> joint(dof);
            HYYRobotBase::GetCurrentJoint(joint.data(), i);
            data[std::string("Robot")+std::to_string(i)]["Joint"]=joint;
            std::vector<double> Cartesian(6);
            HYYRobotBase::GetCurrentCartesian(NULL,NULL,(HYYRobotBase::robpose*)Cartesian.data(), i);
            data[std::string("Robot")+std::to_string(i)]["Cartesian"]=Cartesian;
        }
        // State  │ data的json  
        publisher.send(zmq::buffer("State " + data.dump()));
    }
}

static void subscriber_loop()
{
    nlohmann::ordered_json data;
    printf("start subscriber_loop\n");
    while(true)
    {
        zmq::message_t msg;
        subscriber.recv(msg);   // 阻塞接收，没有消息时就停在这里
        std::string cmd(static_cast<char*>(msg.data()), msg.size()); // 转换消息为字符串
        auto pos = cmd.find(' ');  // 查找第一个空格的位置
        std::string topic = cmd.substr(0, pos); // 提取空格前的部分作为topic
        nlohmann::ordered_json cmd_json = nlohmann::json::parse(cmd.substr(pos + 1));

        int rn=HYYRobotBase::robot_getNUM();
        if ("Switch"==topic)
        {
            if (cmd_json.contains("Switch")) // JSON 对象的第一层里，是否存在Switch
            {
                HYYRobotBase::ClearRobotError();
                if (cmd_json["Switch"].get<bool>())
                {
                    for (int i=0;i<rn;i++)
                    {
                        HYYRobotBase::RobotStopRecover(i);
                        HYYRobotBase::ServoEnd(i);
                        HYYRobotBase::RobotPoweroff(i);
                        HYYRobotBase::RobotPower(i);
                        // HYYRobotBase::ServoStart(0.0001,0.1, i);
                    }
                }
                else
                {
                    for (int i=0;i<rn;i++)
                    {
                        // HYYRobotBase::ServoEnd(i);
                        HYYRobotBase::RobotPoweroff(i);
                    } 
                }
            } 
        }else if (topic == "Hello")
        {
            printf("Hello received\n");
            data[std::string("HelloAck")] = true;
            publisher.send(zmq::buffer("HelloAck " + data.dump()));
        }else if (topic == "Ready")
        {
            printf("Ready received, handshake complete ✔\n");
            handshake_ok.store(true);
        }else if (topic == "stop")
        {
            printf("client bye\n");
            handshake_ok.store(false);
        }else if (topic == "moveA")
        {
            std::string joint_name = cmd_json["joint_name"].get<std::string>();
            std::string speed_name = cmd_json["speed_name"].get<std::string>();
            HYYRobotBase::robjoint p;
            HYYRobotBase::getrobjoint(joint_name.c_str(), &p);
            HYYRobotBase::speed v;
            HYYRobotBase::getspeed(speed_name.c_str(),&v);
            HYYRobotBase::moveA(&p,&v,NULL,NULL,NULL);
        }else if (topic == "moveL")
        {
            std::string pose_name = cmd_json["pose_name"].get<std::string>();
            std::string speed_name = cmd_json["speed_name"].get<std::string>();
            HYYRobotBase::robpose d;
            HYYRobotBase::getrobpose(pose_name.c_str(), &d);
            HYYRobotBase::speed v;
            HYYRobotBase::getspeed(speed_name.c_str(),&v);
            HYYRobotBase::moveL(&d,&v,NULL,NULL,NULL);
        }
    }
}

void PluginMain()
{
    publisher.set(zmq::sockopt::sndhwm, 1);  // 0 表示无限队列 1 队列只有1
    publisher.set(zmq::sockopt::conflate, 1);   
    publisher.set(zmq::sockopt::immediate, 1);  // SUB 未连接时直接丢弃
    publisher.bind("tcp://*:8000");
    pub_th=new std::thread(publisher_loop);
    pub_th->detach();
    subscriber.connect("tcp://192.168.0.35:8001");
    subscriber.set(zmq::sockopt::subscribe, "");
    subscriber.set(zmq::sockopt::rcvhwm, 1);
    subscriber.set(zmq::sockopt::conflate, 1); 
    sub_th=new std::thread(subscriber_loop);
    sub_th->detach();

    

}
