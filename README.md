# AeroPointer

a Near field unmanned aerial vehicle interaction system

AEROPOINTER aims to control the movement of multi rotor unmanned aerial vehicles (UAVs) through human arm pointing, finger posture, and other actions, in order to directly specify the flight target, flight mode, and other functions of the UAV, bringing a new interaction method of "using the UAV as an extension of the operator's arm". Based on this, this project can be extended and developed to achieve fixed distance and direction automatic following, accompanying visual assistance, and other functions. It has great application value in certain fields and industries.

![LOGO](./img/LOGO.svg)

将你的代码转换为 STM32 上的程序，主要涉及到以下几个方面：

1. **串口通信**：使用 UART 进行串口通信。
2. **获取 UWB 数据**：从 UWB 模块获取定位数据。
3. **MAVLink 通信**：使用 MAVLink 协议发送定位数据。
4. **时间管理**：获取时间戳和延迟。

假设你使用的是 STM32 HAL 库来开发，并且已经配置好了 UART 和其他外设，下面是一个大致的框架，展示如何在 STM32 上实现类似的功能。

### 初始化代码

```c
#include "main.h"
#include "usart.h"
#include "tim.h"
#include "mavlink.h"
#include "UWBmodule.h"

UART_HandleTypeDef huart1;  // 用于 UWB 模块的 UART
UART_HandleTypeDef huart2;  // 用于 MAVLink 的 UART

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_TIM2_Init(void);

// 时间戳函数
uint32_t get_timestamp_ms(void) {
    return HAL_GetTick();
}

// MAVLink 通信函数
void send_mavlink_vision_position_estimate(uint32_t timestamp, float x, float y, float z) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;

    mavlink_msg_vision_position_estimate_pack(
        1, 200, &msg, timestamp, x, y, z, 0, 0, 0
    );

    len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();

    printf("UWB初始化完成\n");
    printf("mavlink初始化完成\n");

    while (1) {
        Pose pose;
        if (UWBmodule_getPose(&huart1, &pose) == 0) {
            uint32_t timestamp = get_timestamp_ms();
            printf("Valid position detected: x=%.2f, y=%.2f, z=%.2f\n", pose.x, pose.y, pose.z);
            send_mavlink_vision_position_estimate(timestamp, pose.x, pose.y, pose.z);
        } else {
            printf("Error or no data from UWB module\n");
        }
        HAL_Delay(100);  // 每 100 毫秒读取和发送一次数据
    }
}
```

