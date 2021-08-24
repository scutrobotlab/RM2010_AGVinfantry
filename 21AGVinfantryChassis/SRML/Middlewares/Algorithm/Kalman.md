# 卡尔曼滤波器与矩阵类
![](https://img.shields.io/badge/%E5%BD%93%E5%89%8D%E7%8A%B6%E6%80%81-%E7%AD%89%E5%BE%85%E7%BD%91%E7%AE%A1%E6%B5%8B%E8%AF%95-yellow.svg)

先进：符合现代C++规范，接口清晰简单。  
安全：大量使用模板元编程，将错误解决在编译期。  
~~可靠：通过网管3C测试，权威认证。~~  


## 矩阵类  

```C++
Mat<3, 3> a; //生成3*3全零矩阵

auto b = Mat<3, 3>::eye(); //生成3*3单位矩阵

auto c = Mat<1, 2>::eye(); //编译不通过！
                           //单位矩阵必须是方阵

auto d = Mat<2, 2>({1,2,2,1}); //调用复制构造函数

Mat<2, 2> e = {1,2,2,1};  //调用复制赋值函数

auto f = a * b;

auto g = a * d;  //编译不通过！
                 //矩阵大小无法相乘

//计算卡尔曼增益
K = Pminus * HT * (H * Pminus * HT + R).inverse();
```

## 卡尔曼类

```C++
KalmanFilter<1,2,3> kalman1;
kalman1.predict(1);

KalmanFilter<1,2> kalman2;
kalman2.predict(1); //编译不通过！
kalman2.predict();
```
