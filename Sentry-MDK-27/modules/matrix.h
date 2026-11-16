/******************************************************************************
 * @file        matrix.h
 * @brief       定长（最大 3x3）单精度浮点矩阵运算库：行列式、转置、伴随、求逆、加法、数乘、乘法。
 * @note        非法输入统一以 row = 0、column = 0 的空矩阵表示，调用方先判 row/column 再用数据。
 ******************************************************************************/

#ifndef __MATRIX_H
#define __MATRIX_H

#include "main.h"
#include "math.h"

/*************************宏****************************/ 
/**
 * @brief 矩阵阶数上限：行数、列数各自最大值，合法取值 1 ~ 3
 */
#define MATRIX_MAX_LEN    3   //矩阵阶数上限：行数/列数各自最大为 3，取值 1~3

/*************************结构体****************************/ 
/**
  * @brief 通用矩阵实例：固定 3x3 存储 + 实际生效的行列数
  */
typedef struct
{
  float matrix[MATRIX_MAX_LEN][MATRIX_MAX_LEN];//行优先存储，matrix[i][j] = 第 i 行第 j 列；仅前 row x column 有效
  uint8_t row;//行数 1 ~ 3；0 表示空矩阵/运算失败
  uint8_t column;//列数 1 ~ 3；0 表示空矩阵/运算失败
}matrix_instance_s;


/*************************外部调用****************************/
/**
 * @brief  求方阵的行列式 |A|
 * @param  m  方阵，要求 row == column 且 1 <= row <= 3
 * @return 行列式值；非法入参返回 0
 * @note   返回 0 也可能是矩阵奇异，需区分时调用方先校验 row == column
 */
float matrix_det(matrix_instance_s m);

/**
 * @brief  求转置矩阵 A^T
 * @param  m  输入矩阵，row/column 不得超过 3
 * @return 转置结果，维数 row = m.column、column = m.row；失败返回空矩阵
 */
matrix_instance_s matrix_transpose(matrix_instance_s m);

/**
 * @brief  求伴随矩阵 A*（满足 A * A* = |A| * I）
 * @param  m  方阵，要求 row == column 且不超过 3
 * @return 与 m 同维的伴随矩阵；非方阵或越界返回空矩阵
 */
matrix_instance_s matrix_adj(matrix_instance_s m);

/**
 * @brief  求逆矩阵 A^(-1) = A* / |A|
 * @param  m  方阵，前置条件 row == column、不超过 3、行列式非零
 * @return 逆矩阵，维数同 m；非方阵、越界或 det == 0 返回空矩阵
 */
matrix_instance_s matrix_inverse(matrix_instance_s m);

/**
 * @brief  矩阵加法 C = A + B
 * @param  a  左矩阵，行列数须与 b 完全相同
 * @param  b  右矩阵
 * @return 结果矩阵，维数同输入；维数不一致或越界返回空矩阵
 */
matrix_instance_s matrix_sum(matrix_instance_s a,matrix_instance_s b);

/**
 * @brief  矩阵数乘 C = num * M
 * @param  num  标量系数，float
 * @param  m    输入矩阵，row/column 不得超过 3
 * @return 与 m 同维的结果矩阵；越界返回空矩阵
 * @note   参数顺序为“标量在前、矩阵在后”
 */
matrix_instance_s matrix_scalar(float num, matrix_instance_s m);

/**
 * @brief  矩阵乘法 C = A * B（C[i][j] = Σ A[i][k] * B[k][j]）
 * @param  a  左矩阵，其列数必须等于 b 的行数
 * @param  b  右矩阵
 * @return 结果矩阵，维数 a.row x b.column；a.column != b.row 或越界返回空矩阵
 * @note   维数不匹配只返回空矩阵且不报错，调用方须自行保证维数约定
 */
matrix_instance_s matrix_multiply(matrix_instance_s a,matrix_instance_s b);


#endif
