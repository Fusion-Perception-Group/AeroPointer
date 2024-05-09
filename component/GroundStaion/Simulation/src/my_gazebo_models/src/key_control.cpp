#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

// 定义按键代码
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64

#define INCREMENT 0.02

int kfd = 0; // 文件描述符，用于读取键盘输入
struct termios cooked, raw; // 终端属性结构体，用于设置终端模式

// 退出程序的信号处理函数
void quit(int sig) {
    tcsetattr(kfd, TCSANOW, &cooked); // 恢复终端模式
    ros::shutdown(); // 关闭 ROS
    exit(0); // 退出程序
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "key_control"); // 初始化 ROS 节点
    ros::NodeHandle nh; // 创建节点句柄
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); // 创建服务客户端

    signal(SIGINT, quit); // 注册 SIGINT 信号处理函数

    // 获取终端属性并设置为原始模式
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    // 设置终端为非阻塞模式
    int flags = fcntl(kfd, F_GETFL, 0);
    fcntl(kfd, F_SETFL, flags | O_NONBLOCK);

    puts("Reading from keyboard"); // 打印提示信息
    puts("---------------------------");
    puts("Use 'WASD' keys to control the stick orientation");

    double azimuth = 0, polar = M_PI / 2; // 初始化方位角和极角
    double initial_x = 0.0, initial_y = 0.0, initial_z = 1.6; // 初始位置

    fd_set readfds; // 文件描述符集合，用于 select 函数
    char c; // 存储读取的字符

    while (ros::ok()) { // 循环执行
        // 清空文件描述符集合
        FD_ZERO(&readfds);
        FD_SET(kfd, &readfds);

        // 设置超时时间
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        // 等待键盘输入或超时
        int result = select(kfd + 1, &readfds, NULL, NULL, &tv);

        if (result == -1) {
            perror("select()"); // 错误处理
            continue;
        }

        if (result && FD_ISSET(kfd, &readfds)) { // 有键盘输入
            // 非阻塞读取所有等待的字符
            while (read(kfd, &c, 1) > 0) {
                switch (c) { // 根据按键更新方位角和极角
                    case KEYCODE_W:
                        polar = std::min(M_PI, polar + INCREMENT );
                        break;
                    case KEYCODE_S:
                        polar = std::max(0.0, polar - INCREMENT );
                        break;
                    case KEYCODE_A:
                        azimuth += INCREMENT ;
                        break;
                    case KEYCODE_D:
                        azimuth -= INCREMENT ;
                        break;
                }
            }
        }

        // 根据方位角和极角计算四元数
        tf2::Quaternion q;
        q.setRPY(0, polar, azimuth);

        // 设置模型状态
        gazebo_msgs::SetModelState set_model_state;
        set_model_state.request.model_state.model_name = "stick_model";
        set_model_state.request.model_state.pose.position.x = initial_x;
        set_model_state.request.model_state.pose.position.y = initial_y;
        set_model_state.request.model_state.pose.position.z = initial_z;
        set_model_state.request.model_state.pose.orientation.x = q.x();
        set_model_state.request.model_state.pose.orientation.y = q.y();
        set_model_state.request.model_state.pose.orientation.z = q.z();
        set_model_state.request.model_state.pose.orientation.w = q.w();

        // 调用服务设置模型状态
        if (!client.call(set_model_state)) {
            ROS_ERROR("Failed to call service set_model_state");
        }
    }

    return 0;
}

// 我这个代码是来控制一个gazebo里的棒子的方向的，但我在gazebo调试时发现，当我输入了很多WASD这样的键盘指令后，该程序不能十分及时的响应，表现在话题不能快速反应增加，我怀疑是计算量太大？或者是延迟太高了，因此我想把这个程序改进一下，牺牲一些东西换取模型快速响应。尽量使用和现在这种差不多的代码，不要写类。