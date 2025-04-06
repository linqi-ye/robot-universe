#include <iostream>
#include <string>
#include <vector>
#include <sys/types.h>   
#include <sys/socket.h>   
#include <netinet/in.h>   
#include <arpa/inet.h>   
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "jsk_rviz_plugins/OverlayText.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <math.h>


#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>

#define pi 3.141592653
#if 0
#pragma pack(1)
struct Send_Package_Socket_Show
{
    float angle_actual[4][3];
    float end_pos_nb_actual[4][3];
    short gait_state;
    float cpu_rate[10];
    int ooda_mode;
    int power_mode;
};
#pragma pack()
#else
#pragma pack(1)
struct Send_Package_Socket_Show
{
    int32_t time_stemp;
    unsigned char gait_mode;
    unsigned char sdk_mode;
    unsigned char leg_state[4];
    unsigned char imu_state;
    unsigned char battery_state[4];
    float att[3];
    float att_rate[3];
    float pos_n[3];
    float spd_b[3];
    float angle_actual[4][3];
    float end_pos_nb[4][3];
    float end_pos_b[4][3];
    float cpu_rate[10];
    int ooda_mode;
    int power_mode;
};
#pragma pack()
#endif


struct joint_st
{
    std::vector<std::string> joint_name;
    std::vector<float> joint_pos;
}joint;

void state_init(sensor_msgs::JointState &joint_state)
{
    joint.joint_name = {
    "LF_1_Link_joint", "LF_2_Link_joint", "LF_3_Link_joint", // left front
    "RF_1_Link_joint", "RF_2_Link_joint", "RF_3_Link_joint", // right front cezhan datui xiaotui
    "LH_1_Link_joint", "LH_2_Link_joint", "LH_3_Link_joint", // left back
    "RH_1_Link_joint", "RH_2_Link_joint", "RH_3_Link_joint", // right back
    };
    joint.joint_pos = {
        0,0,0,
        0,0,0,
        0,0,0,
        0,0,0
    };
    joint_state.header.stamp = ros::Time::now();
     
    joint_state.name.resize(12);
    joint_state.position.resize(12);
 
    for(size_t i=0;i<=11;i++)
    {
        joint_state.name[i] = joint.joint_name[i];
    }
    
    for(int i=0;i<=11;i++)
    {
        joint_state.position[i]=joint.joint_pos[i];
    }
}

void Perror(const char *s)
{
    perror(s);
    exit(EXIT_FAILURE);
}

static void setnonblocking(int sockfd) {
    int flag = fcntl(sockfd, F_GETFL, 0);
    if (flag < 0) {
        Perror("fcntl F_GETFL fail");
        return;
    }
    if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) {
        Perror("fcntl F_SETFL fail");
    }
}

std::string ooda_mode_str(int mode)
{
    std::string mode_str="";
    int _mode = mode % 100;
    
    // 1 oa
    if(mode / 100 == 0)
    {
        mode_str = "";
        mode_str += "OODA Mode: OA  Gait Mode: ";
    }
    if(mode / 100 == 1)
    {
        mode_str = "";
        mode_str += "OODA Mode: OODA Gait Mode: ";
    }
    if(mode / 100 == 2)
    {
        mode_str = "";
        mode_str += "OODA Mode: ODA Gait Mode: ";
    }

    switch (_mode)
    {
    case 0:
        mode_str += "START";
        break;
    case 1:
        mode_str += "READY";
        break;
    case 2:
        mode_str += "GO";
        break;
    case 3:
        mode_str += "BALANCE";
        break;
    case 4:
        mode_str += "BALANCE_UP";
        break;
    case 5:
        mode_str += "BALANCE_DOWN";
        break;
    case 6:
        mode_str += "BALANCE_RPY";
        break;
    case 13:
        mode_str += "MOTOR_DISABLE";
        break;
    case 55:
        mode_str += "GO_PROTECT";
        break;
    case 66:
        mode_str += "GO_SELF";
        break;
    case 12:
        mode_str += "TO_RUN";
        break;
    case 15:
        mode_str += "TO_BALANCE";
        break;
    case 16:
        mode_str += "JUMPING";
        break;
    case 17:
        mode_str += "JUMPING_UP";
        break;

    default:
        break;
    }
    
    return mode_str;
}

std::string power_mode_tostr(int mode)
{
    std::string num_str=std::to_string(mode);
    std::string str="Power_Mode:" + num_str;
    switch (mode)
    {
    case 0:
        return str+=" MAXN_30W_8CORE";
    case 2:
        return str+=" MODE_15W_4CORE";
    case 3:
        return str+=" MODE_30W_ALL";
    case 4:
        return str+=" MODE_30W_6CORE";
    case 5:
        return str+=" MODE_30W_4CORE";
    case 6:
        return str+=" MODE_30W_2CORE";
    default:
        return "No such mode to set";
    }
}

int main(int argc,char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"panda_visual");
    ros::NodeHandle nh;

    ros::Publisher joint_pub=nh.advertise<sensor_msgs::JointState>("joint_states",1);
    ros::Publisher cpu_pro_pub1 = nh.advertise<std_msgs::Float32>("/stamp/cpu_sample1", 10);
    ros::Publisher cpu_pro_pub2 = nh.advertise<std_msgs::Float32>("/stamp/cpu_sample2", 10);
    ros::Publisher cpu_pro_pub3 = nh.advertise<std_msgs::Float32>("/stamp/cpu_sample3", 10);
    ros::Publisher cpu_pro_pub4 = nh.advertise<std_msgs::Float32>("/stamp/cpu_sample4", 10);
    ros::Publisher cpu_pro_pub5 = nh.advertise<std_msgs::Float32>("/stamp/cpu_sample5", 10);
    ros::Publisher cpu_pro_pub6 = nh.advertise<std_msgs::Float32>("/stamp/cpu_sample6", 10);
    ros::Publisher cpu_pro_pub7 = nh.advertise<std_msgs::Float32>("/stamp/cpu_sample7", 10);
    ros::Publisher cpu_pro_pub8 = nh.advertise<std_msgs::Float32>("/stamp/cpu_sample8", 10);
    ros::Publisher cpu_pro_all_pub = nh.advertise<std_msgs::Float32>("/stamp/cpu_all", 10);
    ros::Publisher ooda_mode_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/stamp/ooda_mode", 10);
    ros::Publisher power_mode_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/stamp/power_mode", 10);
 
    //robot
    ros::Publisher leg_end_pub = nh.advertise<std_msgs::Float32MultiArray>("/panda5/leg_end_bn", 10);
 
    static int cnt = 0;
    float sys_dt = 0;
    int flag = 0;
    float timer[5]={0};
    int i=0;

    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }
   setnonblocking(sock_fd);//非阻塞
   struct sockaddr_in addr_serv;
   int len;
   memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
   addr_serv.sin_family = AF_INET;//使用IPV4地址
   addr_serv.sin_port = htons(8000);//端口
   addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  //自动获取IP地址

   ROS_INFO("-----addr_serv:%s----------\n", inet_ntoa(addr_serv.sin_addr));
 
   len = sizeof(addr_serv);

   if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
   {
     perror("bind error:");
     exit(1);
   }

    int recv_num=0,send_num=0;
    char send_buf[1024]={0},recv_buf[1024]={0};
    struct sockaddr_in addr_client;

    sensor_msgs::JointState joint_state;
    std_msgs::Float32 cpu_pro_data1;
    std_msgs::Float32 cpu_pro_data2;
    std_msgs::Float32 cpu_pro_data3;
    std_msgs::Float32 cpu_pro_data4;
    std_msgs::Float32 cpu_pro_data5;
    std_msgs::Float32 cpu_pro_data6;
    std_msgs::Float32 cpu_pro_data7;
    std_msgs::Float32 cpu_pro_data8;
    std_msgs::Float32 cpu_all;
    jsk_rviz_plugins::OverlayText ooda_mode_text;
    jsk_rviz_plugins::OverlayText power_mode_text;

    cpu_pro_data1.data=0;
    cpu_pro_data2.data=0;
    cpu_pro_data2.data=0;
    cpu_pro_data4.data=0;
    cpu_pro_data5.data=0;
    cpu_pro_data6.data=0;
    cpu_pro_data7.data=0;
    cpu_pro_data8.data=0;
    cpu_all.data=0;

    state_init(joint_state);

    joint_pub.publish(joint_state);
    cpu_pro_pub1.publish(cpu_pro_data1);
    cpu_pro_pub2.publish(cpu_pro_data2);
    cpu_pro_pub3.publish(cpu_pro_data3);
    cpu_pro_pub4.publish(cpu_pro_data4);
    cpu_pro_pub5.publish(cpu_pro_data5);
    cpu_pro_pub6.publish(cpu_pro_data6);
    cpu_pro_pub7.publish(cpu_pro_data7);
    cpu_pro_pub8.publish(cpu_pro_data8);
    cpu_pro_all_pub.publish(cpu_all);

	ros::Rate loop_rate(1000);
    ROS_INFO("Panda5 Loop Start!");
	while (ros::ok())
    {
  //读取客户端
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        //ROS_INFO("recv_num=%d",recv_num);
        if(recv_num <= 0)
        {
            //perror("OCU recvfrom error:");
            //exit(1);
        }
        else{
            //解码
            Send_Package_Socket_Show _recv_msg;
            // ROS_INFO("[0]=%f %f %f",_recv_msg.angle_actual[0][0],_recv_msg.angle_actual[0][1],_recv_msg.angle_actual[0][2]);
            // ROS_INFO("[1]=%f %f %f",_recv_msg.angle_actual[1][0],_recv_msg.angle_actual[1][1],_recv_msg.angle_actual[1][2]);
            // ROS_INFO("[2]=%f %f %f",_recv_msg.angle_actual[2][0],_recv_msg.angle_actual[2][1],_recv_msg.angle_actual[2][2]);
            // ROS_INFO("[3]=%f %f %f",_recv_msg.angle_actual[3][0],_recv_msg.angle_actual[3][1],_recv_msg.angle_actual[3][2]);

            // ROS_INFO("[0]=%f %f %f",_recv_msg.end_pos_nb[0][0],_recv_msg.end_pos_nb[0][1],_recv_msg.end_pos_nb[0][2]);
            // ROS_INFO("[1]=%f %f %f",_recv_msg.end_pos_nb[1][0],_recv_msg.end_pos_nb[1][1],_recv_msg.end_pos_nb[1][2]);
            // ROS_INFO("[2]=%f %f %f",_recv_msg.end_pos_nb[2][0],_recv_msg.end_pos_nb[2][1],_recv_msg.end_pos_nb[2][2]);
            // ROS_INFO("[3]=%f %f %f",_recv_msg.end_pos_nb[3][0],_recv_msg.end_pos_nb[3][1],_recv_msg.end_pos_nb[3][2]);
            std_msgs::Float32MultiArray leg_end;
            for(int i=0;i<4;i++){
                leg_end.data.push_back(_recv_msg.end_pos_nb[i][0]);
                leg_end.data.push_back(_recv_msg.end_pos_nb[i][1]);
                leg_end.data.push_back(_recv_msg.end_pos_nb[i][2]);
            }
            leg_end_pub.publish(leg_end);

            memcpy(&_recv_msg,recv_buf,sizeof(_recv_msg));

            joint_state.header.stamp = ros::Time::now();
          
            joint_state.position[0]=_recv_msg.angle_actual[0][0];
            joint_state.position[1]=_recv_msg.angle_actual[0][1];
            joint_state.position[2]=_recv_msg.angle_actual[0][2];
            
            joint_state.position[3]=_recv_msg.angle_actual[1][0];
            joint_state.position[4]=_recv_msg.angle_actual[1][1];
            joint_state.position[5]=_recv_msg.angle_actual[1][2];

            joint_state.position[6]=_recv_msg.angle_actual[2][0];
            joint_state.position[7]=_recv_msg.angle_actual[2][1];
            joint_state.position[8]=_recv_msg.angle_actual[2][2];
            
            joint_state.position[9]=_recv_msg.angle_actual[3][0];
            joint_state.position[10]=_recv_msg.angle_actual[3][1];
            joint_state.position[11]=_recv_msg.angle_actual[3][2];

            joint_pub.publish(joint_state);
            // ROS_INFO("cpu:%f %f %f %f %f %f %f %f\n",_recv_msg.cpu_rate[0],_recv_msg.cpu_rate[1],
            // _recv_msg.cpu_rate[2],_recv_msg.cpu_rate[3],_recv_msg.cpu_rate[4],_recv_msg.cpu_rate[5],
            // _recv_msg.cpu_rate[6],_recv_msg.cpu_rate[7]);

            cpu_pro_data1.data=_recv_msg.cpu_rate[0];
            cpu_pro_data2.data=_recv_msg.cpu_rate[1];
            cpu_pro_data2.data=_recv_msg.cpu_rate[2];
            cpu_pro_data4.data=_recv_msg.cpu_rate[3];
            cpu_pro_data5.data=_recv_msg.cpu_rate[4];
            cpu_pro_data6.data=_recv_msg.cpu_rate[5];
            cpu_pro_data7.data=_recv_msg.cpu_rate[6];
            cpu_pro_data8.data=_recv_msg.cpu_rate[7];
            cpu_all.data=cpu_pro_data1.data+cpu_pro_data2.data+cpu_pro_data3.data+cpu_pro_data4.data
                         +cpu_pro_data5.data+cpu_pro_data6.data+cpu_pro_data7.data+cpu_pro_data8.data;
            // cpu_all.data=cpu_all.data;
            
            cpu_pro_pub1.publish(cpu_pro_data1);
            cpu_pro_pub2.publish(cpu_pro_data2);
            cpu_pro_pub3.publish(cpu_pro_data3);
            cpu_pro_pub4.publish(cpu_pro_data4);
            cpu_pro_pub5.publish(cpu_pro_data5);
            cpu_pro_pub6.publish(cpu_pro_data6);
            cpu_pro_pub7.publish(cpu_pro_data7);
            cpu_pro_pub8.publish(cpu_pro_data8);
            cpu_pro_all_pub.publish(cpu_all);

            ooda_mode_text.text = ooda_mode_str(_recv_msg.ooda_mode);
            ooda_mode_pub.publish(ooda_mode_text);
            
            power_mode_text.text = power_mode_tostr(_recv_msg.power_mode);
            power_mode_pub.publish(power_mode_text);
        }

		ros::spinOnce();               // check for incoming messages
		loop_rate.sleep();
    }
    close(sock_fd);  
    return 0;  
}