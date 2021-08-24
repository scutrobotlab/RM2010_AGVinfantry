#include "Tool_Kalman.h"

/**
 * @brief 构造一个新的卡尔曼滤波器实例
 * 构造完成后，必须按照物理模型手动给 A、B、H 赋值
 * 
 * @param stateDim 状态空间维度，即你整个卡尔曼滤波器会接触到多少种物理量。
 * 这些物理量之间必须是线性的，就是说他们之间的关系只有加减乘除、积分、微分
 * 例如 位置-速度-加速度 的状态空间，就有3个维度
 * 
 * @param measureDim 测量矩阵维度，即有多少个传感器
 * 例如你有 绝对码盘（角度）-相对码盘（角速度）-陀螺仪（角加速度），就有3个维度
 * 
 * @param controlDim 控制矩阵维度，即有多少个控制量可以用进去做预测。
 * 可以填0。一般也只有1个，就是你发给电调的电流，它与加速度有关
 * 
 * @param processErr 过程噪声
 * 整个环境中有多少不可控因素，比如向前运动突然撞车，就是一种过程噪声。
 * 数字越大，噪声越大。 
 * 
 * @param measureErr 测量噪声
 * 传感器引入的噪声。也就是传感器精度。
 * 数字越大，噪声越大。 
 */
KalmanFilter::KalmanFilter(uint16_t stateDim, uint16_t measureDim, uint16_t controlDim, float32_t processErr, float32_t measureErr)
{
    xhat = Mat::zeros(stateDim, 1);
    xhatminus = Mat::zeros(stateDim, 1);
    A = Mat::eye(stateDim, stateDim);

    Q = Mat::eye(stateDim, stateDim, processErr);
    H = Mat::zeros(measureDim, stateDim);
    R = Mat::eye(measureDim, measureDim, measureErr);

    // 默认构造假定初始状态不可知，故给予极大的协方差矩阵
    P = Mat::eye(stateDim, stateDim, 1e7);
    Pminus = Mat::zeros(stateDim, stateDim);
    K = Mat::zeros(stateDim, measureDim);

    if(controlDim > 0)
        B = Mat::zeros(stateDim, controlDim);

    u = Mat::zeros(controlDim, 1);
    z = Mat::zeros(measureDim, 1);

    HT = H.trans();
    AT = A.trans();
}

/**
 * @brief 初始化卡尔曼滤波器
 * 如果你知道初始状态，可以输入给滤波器。否则你可不必调用此函数。
 * @param state 数组，代表各个状态量
 * @param error 一个数字，代表状态量的可信度。可信度越高，数字越小。
 */
void KalmanFilter::init(float32_t* state, float32_t error)
{
    xhat.pData = state;
    P = Mat::eye(P.numRows, P.numCols, error);
}

float32_t* KalmanFilter::predict(float32_t* control)
{
    u.pData = control;

    //1. 预测状态
    xhatminus = A * xhat;

    if (control && B.numRows!=0 && B.numCols!=0)
        xhatminus = xhatminus + B * u;

    //2. 预测误差协方差
    Pminus = A * P * AT + Q;

    return xhatminus.pData;
}

float32_t* KalmanFilter::correct(float32_t* measurement)
{
    z.pData = measurement;

    //3. 计算卡尔曼增益
    K = H * Pminus * HT + R;
    P = K.inverse();
    K = Pminus * HT * P;

    //4. 用测量值更新估计
    xhat = xhatminus + K * (z - H * xhatminus);

    //5. 更新误差协方差
    P = (Q - K * H) * Pminus;

    return xhat.pData;
};
