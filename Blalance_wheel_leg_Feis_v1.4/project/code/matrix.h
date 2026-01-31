#include "zf_common_headfile.h"
#ifndef CODE_MATRIX_H_
#define CODE_MATRIX_H_


/*********************************************************************参数*********************************************************************/
#define MAX_SIZE (4)
#define ASSERT(x) zf_assert(x)

typedef float matrix_type;
typedef struct
{
    int rows;
    int cols;
    matrix_type data[MAX_SIZE][MAX_SIZE];
}matrix_t;
extern matrix_t error;
extern matrix_t exf_x;

typedef struct
{
    matrix_type roll, pitch, yaw;
}EulerAngles;
extern EulerAngles euler_angle;
/*********************************************************************参数*********************************************************************/


/*********************************************************************函数*********************************************************************/
#define __weak                __attribute__((weak))                                                 //弱函数

#define clip(x, min, max)    (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))                //限幅函数

#define ABS(x)               (((x) > 0) ? (x) : (-(x)))                                             //绝对值函数

#define clip2(x, num)        (clip((x), (-ABS(num)), (ABS(num))))                                   //绝对值限幅函数

#define MAX(a, b)            (((a) > (b)) ? (a) : (b))                                              //求最大值函数

#define MIN(a, b)            (((a) < (b)) ? (a) : (b))                                              //求最小值函数

void Matrix_Init(matrix_t *martix, int rows, int col);                                              //创建零矩阵

void Matrix_From_Array(matrix_t *mat, const matrix_type *array, const int rows, const int cols);    //从数组中获取矩阵

void Matrix_Identity(matrix_t *matrix, int size);                                                   //创建单位矩阵

matrix_t Matrix_Transpose(const matrix_t *src);                                                     //求矩阵的转置

matrix_t multiply_matrices(const matrix_t *A, const matrix_t *B);                                   //求矩阵的乘法

matrix_t add_matrices(const matrix_t *A, const matrix_t *B);                                        //求矩阵的加法

matrix_t subtract_matrices(const matrix_t *A, const matrix_t *B);                                   //求矩阵的减法

int inverse_matrix(matrix_t *A, matrix_t *invA);                                                    //求矩阵的求逆

void normalize_vector(matrix_t *v);                                                                 //向量归一化

void print_matrix(const matrix_t *matrix);                                                          //输出矩阵
/*********************************************************************函数*********************************************************************/


#endif /* CODE_MATRIX_H_ */
