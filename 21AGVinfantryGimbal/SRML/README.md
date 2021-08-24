## SRML: SCUT Robotlab Middlewares Layer Library
<Font><font size="5">__机器人实验室嵌入式软件中间件层库__</Font>

![](https://img.shields.io/badge/当前版本(构建中)-V0.4-blue.svg)

### Summary 简介 

SRML旨在将已有的代码解耦，模块化后封装成一个服务于组织内所有软件项目的中间件层，形成实验室内部软件开发标准接口。今后机器人的电控软件业务逻辑（跟比赛项目中机器人的具体功能流程逻辑相关的代码）的开发过程中统一基于这个接口，以解决实验室代码混乱的问题。同时机器人软件开发者需要维护、升级已有的模块，而无需为了同一个功能重复造轮子，提高软件开发效率和质量。如果你认为自己对某个模块有更好的代码方案，或者发现了现有模块不再满足当前使用需求，请仔细阅读完原有代码后向实验室其他队员收集需求意见然后再进行重构。重构时请务必谨慎考虑模块接口的设置！
- __功能强大。__
  - 本库包含实验室所有稳定的底层代码，通信协议，设备驱动，功能模块以及终端调试系统，可提供完整的嵌入式控制系统开发工具链。
- __灵活方便。__
  - 内核可裁剪（通过预编译宏决定参与编译的模块），使用时只需要包含一个头文件即可。
- __维护方便。__
  - 本仓库将严格按照代码规范编写，各个功能模块由模块负责人专人维护，建立清晰的依赖关系。
  - 所有代码需要严格测试后才能发布。
  - 按照Git使用规范管理本仓库。

详情请参考 Confluence：[通用软件系统搭建分析](https://www.scut-robotlab.cn/confluence/pages/viewpage.action?pageId=21987475)

### How to Deploy 配置方法
- __获取 SRML 库__
  - 参见：[添加子模块的远程仓库](#添加子模块的远程仓库)
- __添加 SRML 库到崭新的 Stm32 工程中__
  - __单个IDE工程__
    - 包含 SRML.h
    - 将本库放置于工程根目录中
    - 编译
  - __多个IDE工程（开发中）__
    - 包含 SRML.h
  - __MDK工程__
    - 包含 SRML.h
    - 在根目录中添加名为`srml_config.h`的头文件,并在项目包含目录中添加该文件所在的目录
    - 在`srml_config.h`中定义各模块是否参与编译（内容参考[附件1](#附件1)）
    - 按照文件目录结构将库文件添加到工程中
    - 编译

### How to Develop 维护方法
- __Git仓库管理__
  - 添加子模块的远程仓库

  ```bash
  $ git submodule add https://www.scut-robotlab.cn/git/Embedded/SRML.git
  ```

  - clone已有的工程时拉取SRML的代码
  ```bash
  $ git submodule init
  $ git submodule update
  ```

  - 更新 SRML 库

  ```bash
  $ cd SRML/
  $ git pull origin master
  ```

  - 推送更改到 SRML 库
  
  ```bash
  $ cd SRML/
  $ git push origin dev
  ```

### 附件1

```c
#ifndef __SRML_CONFIG_H__
#define __SRML_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Drivers ----------------------------------------------------*/
#define USE_SRML_REFEREE                  0
#define USE_SRML_DJI_MOTOR                1
#define USE_SRML_OTHER_MOTOR              0
#define USE_SRML_DR16                     1
#define USE_SRML_BMX055                   1
#define USE_SRML_MPU6050                  1
#define USE_SRML_W25Qx                    1
#define USE_SRML_W25Qx                    1
#define USE_SRML_FATFS                    1 
  
#define USE_SRML_I2C                      1
#define USE_SRML_SPI                      1
#define USE_SRML_CAN                      1
#define USE_SRML_UART                     1
#define USE_SRML_TIMER                    1

/* Middlewares -----------------------------------------------*/
#define USE_SRML_PID                      1
#define USE_SRML_FILTER                   1
#define USE_SRML_MYMAT                    0
#define USE_SRML_KALMAN                   0
#define USE_SRML_TRACK_GENERATOR          0

#define USE_SRML_SERIAL_LINE_IP           1
  
#define USE_SRML_MYASSERT                 0
#define USE_SRML_LIST                     1
#define USE_SRML_SYSLOG                   1
#define USE_SRML_SYSANALYSIS         	    1

#define USE_SRML_GIMBAL                   0
#define USE_SRML_CHASSIS                  1
#define USE_SRML_POW_CTRL                 1


#ifdef __cplusplus
}
#endif

#endif
```

