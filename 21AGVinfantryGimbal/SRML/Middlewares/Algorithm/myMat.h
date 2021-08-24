#pragma once
#ifdef __cplusplus

#include <memory>
#include <array>

#ifdef ARM_MATH_CM4 
  #ifdef STM32F405xx
    //#include <stm32f4xx.h>
    #include <arm_math.h>
  #endif
#else
struct arm_matrix_instance_f32
{
	int numRows;
	int numCols;
	float* pData;
};
void arm_mat_init_f32(arm_matrix_instance_f32* a, int b, int c, float* _data) {}
void arm_mat_add_f32(const arm_matrix_instance_f32* a, const arm_matrix_instance_f32* b, const arm_matrix_instance_f32* c) {}
void arm_mat_sub_f32(const arm_matrix_instance_f32* a, const arm_matrix_instance_f32* b, const arm_matrix_instance_f32* c) {}
void arm_mat_mult_f32(const arm_matrix_instance_f32* a, const arm_matrix_instance_f32* b, const arm_matrix_instance_f32* c) {}
void arm_mat_trans_f32(const arm_matrix_instance_f32* a, arm_matrix_instance_f32* b) {}
void arm_mat_inverse_f32(const arm_matrix_instance_f32* a, arm_matrix_instance_f32* b) {}
#endif

/**
 * @brief Mat 矩阵类
 * 将arm中自带的矩阵结构体封装成C++类
 */
template <int Rows, int Cols>
class Mat
{
public:
	Mat() { arm_mat_init_f32(&_mat, Rows, Cols, data); }	//默认构造为全零矩阵
	Mat(const std::array<float, Rows* Cols>& obj)
	{
		std::copy(obj.begin(), obj.end(), std::begin(data));
		arm_mat_init_f32(&_mat, Rows, Cols, data);
	}
	Mat(float _data)
	{
		for (int i = 0; i < Rows * Cols; i++)
			data[i] = _data;
		arm_mat_init_f32(&_mat, Rows, Cols, data);
	}
	Mat<Rows, Cols>& operator=(const float(&_data)[Rows * Cols])
	{
		std::copy(std::begin(_data), std::end(_data), std::begin(data));
		return *this;
	}
	Mat<Rows, Cols>& operator=(const std::array<float, Rows* Cols>& obj)
	{
		std::copy(obj.begin(), obj.end(), std::begin(data));
		return *this;
	}

	template<class T = Mat<Rows, Cols>>
	static
		typename std::enable_if<Rows == Cols, T>
		::type eye(float num = 1)
	{
		auto m = Mat<Rows, Cols>();
		for (int i = 0; i < Cols; i++)
			m[i * Cols + i] = num;
		return m;
	}

	template<class T = Mat<Rows, Cols>>
	static
		typename std::enable_if<Rows == Cols, T>
		::type diag(float(&_pData)[Rows])
	{
		auto m = Mat<Rows, Cols>();
		for (int i = 0; i < Cols; i++)
			m[i * Cols + i] = _pData[i];
		return m;
	}

	float& operator[](int idx) { return data[idx]; }

	Mat<Rows, Cols> operator+(const Mat<Rows, Cols>& other)
	{
		Mat<Rows, Cols> result;
		arm_mat_add_f32(&_mat, &other._mat, &result._mat);
		return result;
	}
	Mat<Rows, Cols> operator-(const Mat<Rows, Cols>& other)
	{
		Mat<Rows, Cols> result;
		arm_mat_sub_f32(&_mat, &other._mat, &result._mat);
		return result;
	}

	template <int Cols, int B_Cols>
	friend class Mat;
	template <int B_Cols> /*!< B.Rows=A.Cols */
	Mat<Rows, B_Cols> operator*(const Mat<Cols, B_Cols>& other)
	{
		auto result = Mat<Rows, B_Cols>();
		arm_mat_mult_f32(&_mat, &other._mat, &result._mat);
		return result;
	}

	friend class Mat<Cols, Rows>;
	Mat<Cols, Rows> trans()
	{
		auto result = Mat<Cols, Rows>();
		arm_mat_trans_f32(&_mat, &result._mat);
		return result;
	}
	Mat<Rows, Cols> inverse()
	{
		auto result = Mat<Rows, Cols>();
		arm_mat_inverse_f32(&_mat, &result._mat);
		return result;
	}

	static const int cols = Cols;
	static const int rows = Rows;
	float data[Cols * Rows] = { 0 };

	std::array<float, Rows* Cols> getData() const
	{
		std::array<float, Rows* Cols> _data;
		std::copy(std::begin(data), std::end(data), _data.begin());
		return _data;
	}

protected:
	arm_matrix_instance_f32 _mat;
};
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
