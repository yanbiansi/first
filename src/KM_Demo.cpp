#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include "HYYRobotInterface.h"
#include "user/BscanServer.h"

using namespace HYYRobotBase;

int main(int argc, char *argv[])
{	
	//------------------------initialize----------------------------------
	int err=0;
	HYYRobotBase::command_arg arg;
	err=HYYRobotBase::commandLineParser(argc, argv,&arg);
	if (0!=err)
	{
		return -1;
	}
	err=HYYRobotBase::system_initialize(&arg);
	if (0!=err)
	{
		return err;
	}
	//-----------------------user designation codes---------------

    // 1.状态相关接口
    // 1.1获取机器人运动状态-输入-robot_index 机器人索引-输出-int 0：正常，<0:异常，7：机器人被强制停止，其他：机器人在运行
	int rob_state = get_robot_move_state(0);
    printf("rob_state:%d\n",rob_state);
    // 1.2机器人使能状态-输入-robot_index 机器人索引-输出-int 1:所有轴均使能 0:至少存在一个未使能轴
    int rob_power = GetRobotPowerState(0);
    printf("rob_power:%d\n",rob_power);
    // 1.3获取机器人自由度-输入-机器人索引号-输出-int 返回机器人自由度
    int dof = HYYRobotBase::robot_getDOF(0);
    printf("dof:%d\n",dof);
    // 1.4获取机器人当前关节角
    std::vector<double> joint(dof);
    GetCurrentJoint(joint.data(), 0);
    printf("%f,%f,%f,%f,%f,%f,%f\n", 
       joint[0], joint[1], joint[2], joint[3], 
       joint[4], joint[5], joint[6]);
    // 1.5获取机器人当前目标关节角(上一个周期的期望关节角)
    std::vector<double> target_joint(dof);
    GetCurrentLastTargetJoint(target_joint.data(), 0);
    printf("%f,%f,%f,%f,%f,%f,%f\n", 
       target_joint[0], target_joint[1], target_joint[2], target_joint[3], 
       target_joint[4], target_joint[5], target_joint[6]);
    // 1.6获取机器人当前位姿
    std::vector<double> Cartesian(6);
    GetCurrentCartesian(NULL, NULL, (HYYRobotBase::robpose*)Cartesian.data(), 0);
    // 1.7获取机器人当前目标位姿(上一个周期的期望位姿)
    std::vector<double> target_Cartesian(6);
    GetCurrentLastTargetCartesian(NULL, NULL, (HYYRobotBase::robpose*)target_Cartesian.data(), 0);
    // 1.8清除所有机器人运动和驱动错误
    ClearRobotError();
    // 1.9机器人下电 * @param robot_index 机器人索引 * @return int 0:成功;其他：失败
    int rob_poweroff = RobotPoweroff(0);
    printf("rob_poweroff:%d\n",rob_poweroff); 
    // 1.10 @brief 机器人上电 * @param robot_index 机器人索引 * @return int 0:成功;其他：失败
    int rob_poweron = RobotPower(0);
    printf("rob_poweron:%d\n",rob_poweron);


    // 2.运动相关接口
    // 2.1获取关节空间示教点
	IMPORTJOINT(R0_L_HOME);// 导入示教器采集的关节角度
    IMPORTJOINT(R0_HOME);
    IMPORTJOINT(R0_P1);
    IMPORTJOINT(R0_P2);
    IMPORTJOINT(R0_P3);
    IMPORTJOINT(R0_P4);
    // 另一种获取示教点方式
    robjoint R0_P1_copy;
	getrobjoint("R0_P1", &R0_P1_copy);

    // 2.2获取笛卡尔空间示教点
    IMPORTPOSE(R0_D1);// 导入示教器采集的位姿
    IMPORTPOSE(R0_D2);
    IMPORTPOSE(R0_D3);
    IMPORTPOSE(R0_D4);
    IMPORTSPEED(R0_speed20);
    // 另一种获取示教点方式
    robpose R0_D3_copy;
	getrobpose("R0_D3", &R0_D3_copy);
    // 另一种获取示教速度方式
    speed R0_speed20_copy;
	getspeed("R0_speed20",&R0_speed20_copy);
    // 获取转弯区
    IMPORTZONE(z1);
    // 2.3求逆运动学
    const char* robot_name=get_name_robot_device(get_deviceName(0,NULL),0);//获取索要操作的机器人名称
    double dou_joint[10];// 定义所有关节位置数组，非vector
    GetGroupPosition(robot_name,dou_joint);// 获取所有关节位置
    R7_KINE rkine;// 运动学数据
    init_R7_KINE2(&rkine,dou_joint,&dof, NULL, NULL);// 初始化运动学数据(R7_KINE)
    double xyz[3];
    double rpy[3];
    for (int i = 0; i < 1; i++)
    {
        for (int j = 0; j < 3; j++) 
        {
            xyz[j] = R0_D1->xyz[j];
            rpy[j] = R0_D1->kps[j];
        }
        set_R7_KINE_joint(&rkine, dou_joint);// 设置当前位置(用于选解,选取距离当前关节最近的解)
        set_R7_KINE_pose(&rkine, xyz, rpy);// 设置待求目标
        int ret=Kine_Inverse(robot_name, &rkine);//逆运动学
        printf("Kine_Inverse:%d\n",ret);
        double joint_ik[10];
        get_R7_KINE_joint(&rkine, joint_ik);//对应rkine.joint
        printf("1:%f,2:%f,3:%f,4:%f,5:%f,6:%f,7:%f\n",
        joint_ik[0],joint_ik[1],joint_ik[2],joint_ik[3],joint_ik[4],joint_ik[5],joint_ik[6]);
    }

    // 2.4 运动指令，指定目标关节位置与运行速度，配置NULL3即可
    printf("MoveA关节空间运动到MoveL起点\n");
    MoveA(R0_P1,R0_speed20,NULL3);
    printf("MoveL终点\n");
    MoveL(R0_D3,R0_speed20,NULL3); 
    printf("MoveA关节空间运动到终点\n");
    MoveA(R0_HOME,R0_speed20,NULL3);
    // 另一种运动形式
    printf("moveA关节空间运动到moveL起点\n");
    moveA(&R0_P1_copy,&R0_speed20_copy,NULL,NULL,NULL);
    printf("moveL终点\n");
    moveL(&R0_D3_copy,&R0_speed20_copy,NULL,NULL,NULL); 
    printf("moveA关节空间运动到终点\n");
    MoveA(R0_HOME,R0_speed20,NULL3);

    
    printf("挂起\n");
	//------------------------wait----------------------------------
	pause();
	return 0;
}
