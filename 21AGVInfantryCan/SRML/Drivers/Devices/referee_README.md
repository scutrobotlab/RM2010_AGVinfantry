## 一、模块功能 
**1.裁判系统数据接收与外部读取**  
裁判系统会把机器人的状态信息等数据通过串口发送给主控  
**2.机器人车间通信**  
每个机器人都可以通过发送一定格式的数据包给裁判系统，裁判系统转发给对应的机器人  
**3.操作手界面数据**  
客户端可以显示3个float数据和六个红/绿小灯  
**4.操作手界面UI**  
每个机器人都可以在相应客户端画出直线、矩形这些基本图形

## 二、使用方法  
首先，Set_refereeUart()设置referee所用串口  
下面三个功能API参数请阅读函数注释的note    

**裁判系统数据接收与外部读取**  
1. 用串口底层接收裁判系统数据，然后把数据丢进消息队列里面。dataQueueSendByU4()  
2. 单独开一个任务,本工程为refereeUnpack_task(),在任务里面调用unPackDataFromRF()  
3. 在RefereeHandle()加入需要接收的包的cmd_id 具体详见referee.h->RefereeSystemID_t  
4.外部读取直接读取相应的数据结构体即可  
	
**机器人车间通信**  
0. 车间通信暂时只能发送1个data，裁判系统是允许113个字节的数据的，后期有必要再改进  
1. 在CV_ToOtherRobot()里面设置target_id与data数据,target_id可以通过robot_client_ID查询接收方机器人ID  
2. 在发送端机器人工程里面，调用CV_ToOtherRobot()，添加需要发送的data数据  
3. 外部重写RobotInteractiveHandle()，内部判断接收ID是否为本地ID，是则进行数据处理   

**操作手界面数据**  
1. 在CV_ToServer()里面设置data1~3与小灯的颜色  
2. 注意一定要让机器人的裁判系统连入服务器，不然无法使用  

**操作手界面UI**  
1.设置绘画的图层Set_DrawingLayer()  
2.对应的drawin()，clean()    
绘画API示例    
Referee.line_drawing(MODIFY_PICTURE, 400,500,1000,900,GREEN, test);  
Referee.rectangle_drawing(ADD_PICTURE, 700,300,200,900,GREEN, test);  
Referee.circle_drawing(ADD_PICTURE, 800,500,200, DARKGREEN, test);  
Referee.oval_drawing(ADD_PICTURE, 800,500,200,500,GREEN,test);  
Referee.arc_drawing(ADD_PICTURE, 800,500,300,500,30,150,PURPLE, test);  
Referee.character_drawing(ADD_PICTURE, 800,500,30,3, test, BLUE, test);  
Referee.oval_drawing(ADD_PICTURE, 800,500,200,500,GREEN,test);  
Referee.clean_one_picture(2, test);   
Referee.clean_all();  

## 三、注意事项  
一定要记得先Set_refereeUart()  
要等待一段时间(等串口、裁判系统稳定)，再发送clean、数据、UI等
建议每次初始化时要先clean  
UI注意：  
名字相同的图画，只显示第一张，后面发的都无效  
画图之后想要修改，请通过MODIFY_PICTURE操作实现  
要有图才能MODIFY_PICTURE操作，否则应ADD_PICTURE添加图片  
相同图层内，先画的覆盖后画的  
数字大的图层覆盖数字小的图层  
字符串的长度不应超过30  
屏幕坐标范围为左下角(0,0)~右上角(1920,1080)  
UI的每一次图形建议画多几次，有时候裁判系统接收不到  
UI函数的name参数要使用数组  



## 四、内部结构与实现
**1.裁判系统数据接收与外部读取**  
unPackDataFromRF()解包-->RefereeHandle()判断ID并赋值给相应数据结构体，当ID为机器人状态信息时，进行当局所有机器人、客户端ID计算，并赋值给robot_client_ID  
**解包思路**  
取出缓冲区的数据(并存进最后的解包数组)，逐个检验，如果每一步都通过校验，则进行数据解析，否则进行解包复位。  
解包过程共需要3个数组，串口接收的queue，unpack(解包)，handle(整包)  
使用queue和unpack是因为要与串口层解耦合  
由于unpack可能是数据包的碎片，所以还需要handle来存储整个包  

**2.机器人车间通信**  
CV_ToOtherRobot()发送给其他机器人    
**3.操作手界面数据**  
CV_ToServer()发送给数据给客户端  
**4.操作手界面UI**  
UI不同功能的数据包格式、长度都是一样的，所以不同的drawing()、clean()设置不同的drawing结构体，最后通过SendDrawData()加上协议格式再发送  
layer+operater_type+graphic_tpye+其他参数即可定义一张图片，其他参数的定义请参考裁判系统数据协议  

发送的数据帧ID有两个，ID1+ID2
ID1用来标注是车间通信还是裁判系统数据，ID2用来标注是UI(0x0100)/数据(0xD180)/车间通信(自定义，当前使用0x0233)  

## 五、备注