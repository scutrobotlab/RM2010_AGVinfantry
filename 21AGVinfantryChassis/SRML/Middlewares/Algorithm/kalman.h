#pragma once
#ifdef __cplusplus
#include "myMat.h"
#include <array>

template <uint16_t StateDim, uint16_t MeasureDim, uint16_t ControlDim = 0, typename Enabled = void>
class KalmanFilter;

template <uint16_t StateDim, uint16_t MeasureDim, uint16_t ControlDim>
class KalmanFilter<StateDim, MeasureDim, ControlDim, std::enable_if_t<ControlDim == 0>>
{
public:
	KalmanFilter(float processErr = 1e-2, float measureErr = 1e-1)
	{
		A = A.eye();
		AT = A.eye();
		Q = Q.eye(processErr);
		R = R.eye(measureErr);
		// 默认构造假定初始状态不可知，故给予极大的协方差矩阵
		P = P.eye(1e7);
	};

	void init(const std::array<float, StateDim>& state, float error)
	{
		xhat = state;
		P = P.eye(error);
	}
	void setA(const Mat<StateDim, StateDim>& _A)
	{
		A = _A;
		AT = A.trans();
	};
	void setH(const Mat<MeasureDim, StateDim>& _H)
	{
		H = _H;
		HT = H.trans();
	};
	std::array<float, StateDim> predict()
	{
		//1. 预测状态
		xhatminus = A * xhat;

		//2. 预测误差协方差
		Pminus = A * P * AT + Q;

		return xhatminus.getData();
	}
	std::array<float, StateDim> correct(const std::array<float, MeasureDim>& measurement)
	{
		z = measurement;

		//3. 计算卡尔曼增益
		K = Pminus * HT * (H * Pminus * HT + R).inverse();

		//4. 用测量值更新估计
		xhat = xhatminus + K * (z - H * xhatminus);

		//5. 更新误差协方差
		P = (Q - K * H) * Pminus;

		return xhat.getData();
	};

	Mat<StateDim, 1> xhat;			//后验状态估计（当前估计值）
	Mat<StateDim, 1> xhatminus;		//先验状态估计（未来预测值）
	Mat<StateDim, StateDim> Q;		//过程噪声协方差（由环境因素引入的噪声）
	Mat<MeasureDim, MeasureDim> R;  //测量噪声协方差（传感器误差）
	Mat<StateDim, StateDim> P;		//后验误差协方差（当前估计误差）
	Mat<StateDim, StateDim> Pminus; //先验误差协方差（未来预测误差）
	Mat<StateDim, MeasureDim> K;	//卡尔曼增益
	Mat<MeasureDim, 1> z;			//测量值（传感器数据）

protected:
	Mat<StateDim, StateDim> A;		//状态转换模型（转移矩阵）
	Mat<MeasureDim, StateDim> H;  	//状态变量到测量值的转换矩阵（测量矩阵）
	Mat<StateDim, StateDim> AT;   	//A.trans
	Mat<StateDim, MeasureDim> HT; 	//H.trans
};


template <uint16_t StateDim, uint16_t MeasureDim, uint16_t ControlDim>
class KalmanFilter<StateDim, MeasureDim, ControlDim, std::enable_if_t<ControlDim != 0>> : public KalmanFilter<StateDim, MeasureDim, 0>
{
public:
	KalmanFilter(float processErr = 1e-2, float measureErr = 1e-1)
		: KalmanFilter<StateDim, MeasureDim, 0>(processErr, measureErr) {}

	std::array<float, StateDim> predict(const std::array<float, ControlDim>& control)
	{
		this->u = control;

		//1. 预测状态
		this->xhatminus = this->A * this->xhat + B * u;

		//2. 预测误差协方差
		this->Pminus = this->A * this->P * this->AT + this->Q;

		return this->xhatminus.getData();
	}
	Mat<StateDim, ControlDim> B;	//控制转换模型（控制矩阵）
	Mat<ControlDim, 1> u;			//控制量（控制器数据）
};

#endif
