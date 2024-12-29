# DeepMotor 项目

## 项目简介

这是一个基于 STM32 的机械臂的运控板。已有的功能如下：

- 2个按键外部中断
- 2个放大输出，可以接继电器，喇叭之类的
- 1路PWM控制蜂鸣器输出
- 3路PWM舵机控制
- 电脑下发UART报文到控制器，控制器转成CAN报文
- CAN报文，转换成UART报文，上传到上位机
- 主循环只跑任务调度函数，有100us, 1ms, 10ms，100ms, 1s等多种任务，新的任务放置到对应的位置上即可
- SPI采集编码器数据
- AD采样的驱动
- 模拟EEPROM的驱动
- 100us AD采样中断，AD是由PWM进行触发的
- 3路电流采样
- FOC控制驱动

## 对应硬件

### CubeMX资源

![image-20241229182714919](https://deep-diary.oss-cn-hangzhou.aliyuncs.com/blog/image-20241229182714919.png)

### 实物硬件

![046dccb65fa378507e576eb68338937](https://deep-diary.oss-cn-hangzhou.aliyuncs.com/blog/046dccb65fa378507e576eb68338937.jpg)

## 开发环境

- IDE: Keil MDK-ARM V5.38.0.0
- STM32CubeMX
- 固件库版本：[待填写]
- 目标芯片：[待填写]

## 目录结构

略

## 注意事项

由CubeMax生成代码后，便于可能会无法通过，可能报错缺少'stm32g4xx_ll_rcc.c'文件，这个时候，可以尝试把Core 文件夹下面的`stm32g4xx_ll_rcc.c` 拷贝到`Driver`中去

## 社交账号

![c45ed1452fb67f4d38c3ef26da32a41](https://deep-diary.oss-cn-hangzhou.aliyuncs.com/blog/c45ed1452fb67f4d38c3ef26da32a41.jpg)
