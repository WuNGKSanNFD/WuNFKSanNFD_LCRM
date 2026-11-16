#include "kalman_filter.h"

/**************************私有函数及变量***************************/
typedef struct
{
  uint8_t dataUpdate_flag;//数据更新标志位
  
  matrix_instance_s X_out;//最优估计
  matrix_instance_s X_measure;//测量值
  matrix_instance_s u;//控制器
  matrix_instance_s A;//状态转移矩阵
  matrix_instance_s B;//控制矩阵
  matrix_instance_s Q;//系统噪声协方差矩阵
  matrix_instance_s R;//测量噪声协方差矩阵
  matrix_instance_s H;//观测矩阵

  matrix_instance_s X_pre;//先验估计
  matrix_instance_s P_pre;//先验估计协方差矩阵
  matrix_instance_s K_k;//卡尔曼增益
  matrix_instance_s P_pro;//最有估计协方差矩阵
  
  /***********上一次状态*************/
  matrix_instance_s X_out_l;//最优估计
  matrix_instance_s X_measure_l;//测量值
  matrix_instance_s u_l;//控制器
  matrix_instance_s X_pre_l;//先验估计
  matrix_instance_s P_pre_l;//先验估计协方差矩阵
  matrix_instance_s K_k_l;//卡尔曼增益
  matrix_instance_s P_pro_l;//最有估计协方差矩阵
  
}kalman_instance_s;


kalman_instance_s *kal[KALMAN_CNT] = {NULL};
static uint8_t idx = 0;

static matrix_instance_s kalman_arr_to_matrix(float *arr,uint8_t row,uint8_t column)
{
  matrix_instance_s m = {{0,},row,column};
  for(uint8_t i = 0; i < row; i++)
  {
    for(uint8_t j = 0; j < column; j++)
    {
      m.matrix[i][j] = arr[column*i+j];
    }
  }
  return m;
}


/***********************外部调用*****************************/

//初始化1
uint8_t kalman_init1(kalman_config1_s *config)
{
  kal[idx] = (kalman_instance_s *)malloc(sizeof(kalman_instance_s));
  memset(kal[idx],0,sizeof(kalman_instance_s));
  
  kal[idx]->A = kalman_arr_to_matrix(&config->A[0][0],2,2);
  kal[idx]->B = kalman_arr_to_matrix(&config->B[0][0],2,1);
  kal[idx]->H = kalman_arr_to_matrix(&config->H[0][0],1,2);
  kal[idx]->Q = kalman_arr_to_matrix(&config->Q[0][0],2,2);
  kal[idx]->R = kalman_arr_to_matrix(&config->R[0][0],1,1);
  
  float init_matrix[2][2] = {0,0,0,0}; 
  
  kal[idx]->X_out       = kalman_arr_to_matrix(&init_matrix[0][0],2,1);
  kal[idx]->X_measure   = kalman_arr_to_matrix(&init_matrix[0][0],1,1);
  kal[idx]->u           = kalman_arr_to_matrix(&init_matrix[0][0],1,1);
  kal[idx]->X_pre       = kalman_arr_to_matrix(&init_matrix[0][0],2,1);//先验估计
  kal[idx]->P_pre       = kalman_arr_to_matrix(&init_matrix[0][0],2,2);//先验估计协方差矩阵
  kal[idx]->K_k         = kalman_arr_to_matrix(&init_matrix[0][0],2,1);//卡尔曼增益
  kal[idx]->P_pro       = kalman_arr_to_matrix(&init_matrix[0][0],2,2);//最有估计协方差矩阵
  kal[idx]->X_out_l     = kalman_arr_to_matrix(&init_matrix[0][0],2,1);//最优估计
  kal[idx]->X_measure_l = kalman_arr_to_matrix(&init_matrix[0][0],1,1);//测量值
  kal[idx]->u_l         = kalman_arr_to_matrix(&init_matrix[0][0],1,1);//控制器
  kal[idx]->X_pre_l     = kalman_arr_to_matrix(&init_matrix[0][0],2,1);//先验估计
  kal[idx]->P_pre_l     = kalman_arr_to_matrix(&init_matrix[0][0],2,2);//先验估计协方差矩阵
  kal[idx]->K_k_l       = kalman_arr_to_matrix(&init_matrix[0][0],2,1);//卡尔曼增益
  kal[idx]->P_pro_l     = kalman_arr_to_matrix(&init_matrix[0][0],2,2);//最有估计协方差矩阵
  
  idx++;
  
  return idx-1;
}

//初始化2
uint8_t kalman_init2(kalman_config2_s *config)
{
  kal[idx] = (kalman_instance_s *)malloc(sizeof(kalman_instance_s));
  memset(kal[idx],0,sizeof(kalman_instance_s));
  
  kal[idx]->A = kalman_arr_to_matrix(&config->A[0][0],2,2);
  kal[idx]->B = kalman_arr_to_matrix(&config->B[0][0],2,1);
  kal[idx]->H = kalman_arr_to_matrix(&config->H[0][0],2,2);
  kal[idx]->Q = kalman_arr_to_matrix(&config->Q[0][0],2,2);
  kal[idx]->R = kalman_arr_to_matrix(&config->R[0][0],2,2);
  
  float init_matrix[2][2] = {0,0,0,0}; 
  
  kal[idx]->X_out       = kalman_arr_to_matrix(&init_matrix[0][0],2,1);
  kal[idx]->X_measure   = kalman_arr_to_matrix(&init_matrix[0][0],2,1);
  kal[idx]->u           = kalman_arr_to_matrix(&init_matrix[0][0],1,1);
  kal[idx]->X_pre       = kalman_arr_to_matrix(&init_matrix[0][0],2,1);//先验估计
  kal[idx]->P_pre       = kalman_arr_to_matrix(&init_matrix[0][0],2,2);//先验估计协方差矩阵
  kal[idx]->K_k         = kalman_arr_to_matrix(&init_matrix[0][0],2,2);//卡尔曼增益
  kal[idx]->P_pro       = kalman_arr_to_matrix(&init_matrix[0][0],2,2);//最有估计协方差矩阵
  kal[idx]->X_out_l     = kalman_arr_to_matrix(&init_matrix[0][0],2,1);//最优估计
  kal[idx]->X_measure_l = kalman_arr_to_matrix(&init_matrix[0][0],2,1);//测量值
  kal[idx]->u_l         = kalman_arr_to_matrix(&init_matrix[0][0],1,1);//控制器
  kal[idx]->X_pre_l     = kalman_arr_to_matrix(&init_matrix[0][0],2,1);//先验估计
  kal[idx]->P_pre_l     = kalman_arr_to_matrix(&init_matrix[0][0],2,2);//先验估计协方差矩阵
  kal[idx]->K_k_l       = kalman_arr_to_matrix(&init_matrix[0][0],2,2);//卡尔曼增益
  kal[idx]->P_pro_l     = kalman_arr_to_matrix(&init_matrix[0][0],2,2);//最有估计协方差矩阵
  
  idx++;
  
  return idx-1;
}

//设置测量数据
void kalman_setMeasure(uint8_t id,matrix_instance_s measure,matrix_instance_s u)
{
  kal[id]->X_measure = measure;
  kal[id]->u = u;
  
  kal[id]->dataUpdate_flag = 1;
}

//work
kalman_status_e kalman_work(uint8_t id)
{
  //判断数据是否更新
  if(kal[id]->dataUpdate_flag != 1)
    return kalman_error;
  
  //先验估计 x_pre = A*x_out_l + B*u
  kal[id]->X_pre = matrix_sum(matrix_multiply(kal[id]->A, kal[id]->X_out_l), matrix_multiply(kal[id]->B, kal[id]->u));
  
  //先验估计协方差矩阵 P_pre = A*P_pro_l*A_T + Q
  matrix_instance_s A_T = matrix_transpose(kal[id]->A);
  kal[id]->P_pre = matrix_sum(matrix_multiply(matrix_multiply(kal[id]->A, kal[id]->P_pro_l), A_T), kal[id]->Q);
  
  //卡尔曼增益 K_k = P_pre*H_T/(H*P_pre*H_T + R)
  matrix_instance_s H_T = matrix_transpose(kal[id]->H);
  matrix_instance_s K_num = matrix_multiply(kal[id]->P_pre, H_T);//分子
  matrix_instance_s K_den = matrix_sum(matrix_multiply(kal[id]->H, K_num), kal[id]->R);
  kal[id]->K_k = matrix_multiply(K_num, matrix_inverse(K_den));
  
  //最优估计协方差矩阵 P_pro = (I - K_k*H)*P_pre
  matrix_instance_s KH = matrix_multiply(kal[id]->K_k, kal[id]->H);
  matrix_instance_s E2 = {{1,0,0,0,1,0,0,0,1},2,2}; //二阶单位矩阵
  kal[id]->P_pro =matrix_multiply(matrix_sum(E2, matrix_scalar(-1, KH)), kal[id]->P_pre);
  
  //最优估计 X_out = X_pre + K_k*(X_measure - H*X_pre)
  matrix_instance_s HX = matrix_multiply(kal[id]->H, kal[id]->X_pre);
  matrix_instance_s Z_HX = matrix_sum(kal[id]->X_measure, matrix_scalar(-1,HX));
  kal[id]->X_out = matrix_sum(kal[id]->X_pre, matrix_multiply(kal[id]->K_k, Z_HX));
  
  
  //传旧值
  kal[id]->X_out_l     = kal[id]->X_out;
  kal[id]->X_measure_l = kal[id]->X_measure;
  kal[id]->u_l         = kal[id]->u;
  kal[id]->X_pre_l     = kal[id]->X_pre;
  kal[id]->P_pre_l     = kal[id]->P_pre;
  kal[id]->K_k_l       = kal[id]->K_k;
  kal[id]->P_pro_l     = kal[id]->P_pro;
  
  //清除更新标志位
  kal[id]->dataUpdate_flag = 0;
  
  return kalman_ok;
}


//数据接口
kalman_data_s kalman_getData(uint8_t id)
{
  kalman_data_s data = {
    .X_out = {kal[id]->X_out_l.matrix[0][0],kal[id]->X_out_l.matrix[1][0]},
    .X_pre = {kal[id]->X_pre_l.matrix[0][0],kal[id]->X_pre_l.matrix[1][0]},
    .test = {0,0,0,0}
  };
  return data;
}

