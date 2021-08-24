# Motor 真香的RM电机库

![](https://img.shields.io/badge/%E5%BD%93%E5%89%8D%E7%89%88%E6%9C%AC-V1.0-blue.svg) ![](https://img.shields.io/badge/C++-11-blue.svg) ![](https://img.shields.io/badge/%E7%BD%91%E7%AE%A1%E6%B5%8B%E8%AF%95-2%20%E9%80%9A%E8%BF%87%2C%201%20%E7%AD%89%E5%BE%85-yellow.svg)

## 测试列表
- [x] Motor.h ![](https://img.shields.io/badge/%E7%BD%91%E7%AE%A1%E6%B5%8B%E8%AF%95-%E9%80%9A%E8%BF%87-brightgreen.svg)
- [ ] PID.h ![](https://img.shields.io/badge/%E7%BD%91%E7%AE%A1%E6%B5%8B%E8%AF%95-%E7%AD%89%E5%BE%85-yellow.svg) 
- [x] MotorControl.h ![](https://img.shields.io/badge/%E7%BD%91%E7%AE%A1%E6%B5%8B%E8%AF%95-%E9%80%9A%E8%BF%87-brightgreen.svg)

## 目录
- [介绍](#介绍)
- [使用方法](#使用方法)
    - [Motor.h 的使用](#motorh-的使用)
        - [电机初始化](#电机初始化)
        - [通过CAN更新电机数据](#通过can更新电机数据)
        - [通过CAN控制电机](#通过can控制电机)
    - [PID.h 的使用](PID.md)
        - [积分项使用细则](#积分项使用细则)
        - [微分项使用细则](#微分项使用细则)
        - [模糊PD控制](#模糊PD控制)
    - [MotorControl.h 的使用](#motorcontrolh-的使用)
        - [设置电机控制目标](#设置电机控制目标)
        - [执行电机控制](#执行电机控制)

## 介绍

* 强大。几乎包含所有RoboMaster电机，自动判断电机协议格式。  
* 自由。既可单独调用`Motor`、`PID`，也可以用聚合模块`MotorControl`。  
* 先进。大量使用C++11特性，大量使用模板，保证除了作者谁也不敢改。

由 [永不编译@BigeYoung](https://www.scut-robotlab.cn/git/BigeYoung) 重构代码，程序专业、规范 ~~虽然开始有21个errors~~ 。  
经过 [客观独立第三方@M3chD09](https://www.scut-robotlab.cn/git/M3chD09) 修复并测试通过，~~可以放心使用~~ 勉强可以使用。

## 使用方法

### Motor.h 的使用

目前支持的电机有：  
* Motor_820R ： 可以与[820R电调](https://www.robomaster.com/zh-CN/products/components/detail/136)搭配的电机（M3510，M2310，M2006）  
* Motor_GM3510 ： [GM3510电机](https://www.robomaster.com/zh-CN/products/components/detail/127)  
* Motor_6623 ： [6623电机](https://www.robomaster.com/zh-CN/products/components/detail/131)  
* Motor_C610 ： 可以与[C610电调](https://www.robomaster.com/zh-CN/products/components/general/M2006)搭配的电机（M2006）  
* Motor_C620 ： 可以与[C620电调](https://www.robomaster.com/zh-CN/products/components/general/M3508)搭配的电机（M3508）  
* Motor_GM6020 ： [GM6020电机](https://www.robomaster.com/zh-CN/products/components/general/GM6020)  

#### 电机初始化

构造函数参数为电机ID。  
特别地，6623电机可以使用`Motor_6623::Type`作为电机ID，其中`Type`可以是`Yaw, Pitch, Roll, Resv, Ex1, Ex2, Ex3, Ex4`中的一个。  

```C++
#include "Motor.h"

Motor_820R pitch_motor(1); //直接初始化
Motor_6623 yaw_motor(Motor_6623::Yaw); //6623电机的初始化
Motor_C620 chassis_motor[] = { Motor_C620(1), Motor_C620(2), Motor_C620(3), Motor_C620(4) };
```

#### 通过CAN更新电机数据

可以在CAN接收中断中调用该函数。

```C++
uint8_t motor_update(CAN_RxHeaderTypeDef *header, uint8_t can_rx_data[])
{
  for (auto& m : chassis_motor)
  {
      if (m.CheckID(header->StdId))
      {
          m.update(can_rx_data);
          return m.ID;
      }
  }
  return 0;
}
```

不同电机有不同的`get*()`函数接口，可以读到角度、速度、力矩、温度等数值。

#### 通过CAN控制电机

对`Motor`的公有成员`Out`赋值，然后调用`MotorMsgSend`函数，就可以将`Out`数据用CAN发送。

```C++
//多个电机
chassis_motor[0].Out = 100;
chassis_motor[1].Out = 100;
chassis_motor[2].Out = 100;
chassis_motor[3].Out = 100;
MotorMsgSend(&hcan1, chassis_motor);
//单个电机
pitch_motor.Out = 100;
MotorMsgSend(&hcan1, pitch_motor);
```
### PID.h 的使用

![](https://img.shields.io/badge/%E7%BD%91%E7%AE%A1%E6%B5%8B%E8%AF%95-%E7%AD%89%E5%BE%85-yellow.svg)  

> 本节教程所标注的引用皆来自[《先进PID控制 MATLAB仿真(第4版)》](https://www.scut-robotlab.cn/nextcloud/remote.php/webdav/%E5%AD%A6%E4%B9%A0%E8%B5%84%E6%96%99/2016_%E5%85%88%E8%BF%9BPID%E6%8E%A7%E5%88%B6MATLAB%E4%BB%BF%E7%9C%9F%20(4th)_%E5%B8%A6%E4%B9%A6%E7%AD%BE.pdf)。
本节所述内容未经过网管测试。  

**请按照以下顺序选择控制器算法。**

#### 1. 开环能解决吗？

PID控制器某些情况下反而降低性能。例如，一个能够自锁的丝杆，直接开环控制即可。

#### 2. 执行机构需要的是控制量的增量吗？

- 是
 
    当执行机构需要的是控制量的增量，例如驱动步进电机时，应采用**增量式PID**[P25]。
    ```C++
    Output += PID.Adjust();
    ```

- 否
 
    其他情况，请选择**位置式PID**[P12]。
    ```C++
    Output = PID.Adjust();
    ```

#### 3. 你有以下烦恼吗？
 
 - 受控系统**存在静态误差**，请参考 [积分项](#积分项使用细则)。

 - 受控系统**惯性太大**，请参考 [微分项](#微分项使用细则)。

 - 向**正/反方向控制**，性能有明显差异，请参考 [模糊PD控制](#模糊PD控制)。

#### 积分项使用细则

积分项只是为了消除静差，`error`比较大的时候，应该交给`PTerm`解决。

如果从`error`还有很大的时候就开始积分，最后`ITerm`过大，会导致**系统振荡**。

因此，我们推荐使用**积分分离**和**变速积分**两个方法。

> 注：本部分所述 `error` 均为 `abs(error)`。

1. **积分分离**[P26]

    当`error > I_SeparThresh`时，积分项不再增大，同时`ITerm`设置为0。
    
    默认情况下，`I_SeparThresh`是一个很大的数字，即不启用积分分离。设置时，请保证`I_SeparThresh`为正数。
    
2. **变速积分**[P34]

    在 `error <= B` 区间内，为普通积分效果（积分速度100%）；

    在 `B < error <= A+B` 区间内，为变速积分效果（积分速度0-100%变化）；
     
    在 `A+B < error` 区间内，不继续积分（积分速度0%）。

    程序中的`A`和`B`数字以`VarSpeed_I_A`和`VarSpeed_I_B`表示。默认情况下，`VarSpeed_I_B`是一个很大的数字，即不启用变速积分。设置时，请保证两个参数均为正数。

#### 微分项使用细则

微分项可以提高系统动态特性，但也容易引入高频干扰。如果你加入微分项后，系统振荡现象没有改善，请考虑观察`error`和`d_err`（微分）是否存在高频噪声。

> 注意：由于`error`和`d_err`为局部变量，在J-Scope中无法观察。如果`PID`是全局变量，可以观察其成员变量`LowPass_error.value`和`LowPass_d_err.value`。因为默认情况下，`LowPassFilter.Factor=0`，即不启用低通滤波器，此时`LowPassFilter.value`就是原始数据。

1. 如果`Target`不变，`error`和`d_err`仍存在高频噪声
    
    请启用低通滤波器。设置对应的`LowPassFilter.Factor`范围在0~1之间即可启用，设置为0即关闭。数字越大，滤波效果越明显，但控制系统效应越严重。
    
    建议只设置微分项的低通滤波器`LowPass_d_err`，因为它对控制系统迟滞影响较小。该算法称为**不完全微分**[P44]。

2. 如果振荡是`Target`变化频繁导致的
    
    请启用**微分先行**[P48]。设置`D_of_Current=true`即可。该算法会采用`Target`的微分量代替`error`的微分量。


如果以上方法都没能解决在目标值附近**高频**振荡的问题，请设置死区（Deadzone），注意该数字必须为正数[P51]。

#### 模糊PD控制

一般不用用到，需要时请自己看书[P295]。

### MotorControl.h 的使用

`MotorControl`将`Motor`和`PID`放在一起，`Motor`采用多态，`PID`采用模板。  
电机控制封装为电机控制类，可以选择的控制方式包括：
* 角度环 `MotorAngleCtrl`
* 速度环 `MotorSpeedCtrl`
* 串级控制 `MotorCascadeCtrl`  

模板参数为控制环，可选择的环包括
* 开环 `OpenLoop`
* PID `PID`
* 模糊PD `FuzzyPD`  

函数参数为电机的指针。

电机控制初始化范例：

```C++
#include "MotorControl.h"

// 串级控制<角度环控制方式, 速度环控制方式> 电机控制器变量名(电机指针)
MotorCascadeCtrl<FuzzyPD, PID> pitch_ctrl(&pitch_motor);

// 多个电机使用相同控制方式
using ChassisCtrl = MotorCascadeCtrl<PID, PID>;
ChassisCtrl chassis_ctrl[] = { 
    ChassisCtrl(&chassis_motor[0]),
    ChassisCtrl(&chassis_motor[1]),
    ChassisCtrl(&chassis_motor[2]),
    ChassisCtrl(&chassis_motor[3])
};
```
[using声明类型别名](https://zh.cppreference.com/w/cpp/language/type_alias)为C++11推荐用法，类似于`typedef`。

在`MotorContrl`类中，`PID`为公有成员，可以直接设置参数。

```C++
//设置单个电机控制类
pitch_ctrl[0].AnglePID.Table = {
    { -200, -100,   0   },
    { -100, 0,      100 },
    { 0,    100,    200 }
};
pitch_ctrl[0].SpeedPID.SetPIDParam(3.0f, 0, 0);

//多个电机控制类使用相同PID参数
for (auto& m : chassis_ctrl)
{
    m.AnglePID.SetPIDParam(1.0f, 0.1f, 0);
    m.SpeedPID.SetPIDParam(1.0f, 0,    0);
}
```

#### 设置电机控制目标

尽管你可以直接对公有成员`PID.Target`赋值，但如果电机采用`CascadeCtrl`（串级控制），速度环的`Target`是**没有效果**的。建议采用`setTarget()`设置控制目标。

* 对于 **角度环** `MotorAngleCtrl`，`setTarget()`相当于给`AnglePID.Target`赋值；
* 对于 **速度环** `MotorSpeedCtrl`，`setTarget()`相当于给`SpeedPID.Target`赋值；
* 对于 **串级控制** `MotorCascadeCtrl`，`setTarget()`相当于给`AnglePID.Target`赋值。

```C++
pitch_ctrl.setTarget(1.0f);
```

#### 执行电机控制

调用`MotorCtrl::Adjust()`函数，会自动结算`PID`，并给`Motor.Out`赋值。
```C++
/*** 单个电机 ***/
picth_ctrl.Adjust();
// MotorCtrl.Adjust()函数会给Motor.Out赋值。
// 如需添加其它控制环，例如前馈控制，可以手动修改Out
pitch_motor.Out += 0.5 * pitch_motor.getAngle();

/*** 多个电机 ***/
for(auto &c: chassis_ctrl)
    c.Adjust();
```
最后，别忘了[通过CAN将数据发送出去](#通过can控制电机)噢！
