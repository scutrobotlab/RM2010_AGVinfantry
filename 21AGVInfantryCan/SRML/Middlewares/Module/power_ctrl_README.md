## power_ctrl

#### 一、使用方法
**1.初始化**  

阅读构造函数，根据构造函数定义功率控制类  
**步兵和英雄**都有超级电容和电流采样板，所以使能电容充电，使能功率环
`	PowerCtrl_ClassDef PowerCtrl( 8, REAL_POWER_LOOP, __ENABLE, 10);	`  
**哨兵**一般没有超级电容和电流采样板，只用裁判系统供电，所以不使能电容充电，使能剩余能量环。
`	PowerCtrl_ClassDef PowerCtrl( 2, REMAIN_ENERGY_LOOP, __DISABLE, 10);	` 

加载外部控制器  
`Load_capChargeController(CapChargeController)；`  
`Load_motorLimitController(MotorLimitController)；`   

**2.使用** 

根据需要设置裁判系统功率，电机功率，剩余能量的目标值  
`Set_PE_Target(80, 120, 30);`  

运行主控制函数,传入当前RF功率、电机功率、剩余能量以及电机速度环的输出值  
`Control(referee_power, motor_power, remain_energy, motor_out_raw);`   

读取电机的限幅比例，并用自写限幅函数进行限幅  
`Get_limScale();`  

读取电容允许的充电功率,并进行充电输出  
`Get_capChargePower()；`  

**注意**  
1.配合chassis.cpp模块使用时  
1)由于chassis需要设置不同的速度挡位，所以你得先知道多大的功率有多大的速度，然后将不同的功率对应的挡位初始化到chassis中。  
2)在变换功率时设置相应的合理速度挡位，才能更好地进行功率控制（四个麦轮经功控之后运动不变形）。举个例子，我当前使用当前功率为80W(对应步兵1.9m/s)，那我应该设置的是低速档或者中速档，而不是高速档。

#### 二、内部结构与实现
**算法流程图**

![image-20200304162627556](C:\Users\kn\AppData\Roaming\Typora\typora-user-images\image-20200304162627556.png)

**有电流采样板？（步兵 / 英雄）**

正常来说，打比赛的车（步兵、英雄）都有我们自己做的电流采样板，可以对底盘的总功率进行采样，采样频率可以达到500Hz左右，没有滤波的实时功率，已经可以把底盘功率控制得比较稳定（速度、功率波动不大）。

**没有电流采样板？（哨兵）**

一般哨兵只搭载陀螺仪主控，是不会有电流采样板的，这时只能用裁判系统提供的剩余能量进行闭环。这时只能把功率控制在30W（哨兵），没有其他选择。



#### 三、FAQ

**底盘系统框图**

**想要进行功率分配？**
在外部实现。实现可以如下

```c++
float allocation[4] = {0.25, 0.3, 0.4, 0.05};/* 分配比例，使用分配算法实时更改，必须是归一化的 */
float motor_out[4];
uint8_t motot_num = 4;

void ProcessMotorData(){
    float lim_scale = PowerCtrl.Get_limScale();/* 得到功控计算的限幅比例 */
    float motor_out_sum = 0.0f;
    
    for(uint8_t i ; i < motot_num; i++)
    	motor_out[i] *= lim_scale;
    
    /* 功率分配 */
    for(uint8_t i ; i < motot_num; i++)
    	motor_out_sum += motor_out[i];
    for(uint8_t i ; i < motot_num; i++)
    	motot_num[i] = motor_out_sum * allocation[i];
}
```




