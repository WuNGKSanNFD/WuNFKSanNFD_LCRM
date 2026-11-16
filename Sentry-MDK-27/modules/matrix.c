#include "matrix.h"

/**********************私有函数及变量*************************/

//越界检验
static uint8_t matrix_check(matrix_instance_s m)
{
  if(m.row > MATRIX_MAX_LEN || m.column > MATRIX_MAX_LEN)
    return 0;
  return 1;
}

/**********************外部调用**************************/
//求行列式
float matrix_det(matrix_instance_s m)
{
  //越界检验
  if(matrix_check(m) == 0)
    return 0;

  float det = 0;
  
  if(m.column != m.row)
    return 0;
    
  //计算
  //2x2
  if(m.row == 2)
  {
    det = m.matrix[0][0]*m.matrix[1][1] - m.matrix[0][1]*m.matrix[1][0];
    return det;
  }
  //3x3
  else if(m.row == 3)
  {
    det =   m.matrix[0][0]*m.matrix[1][1]*m.matrix[2][2]
          + m.matrix[0][1]*m.matrix[1][2]*m.matrix[2][0]
          + m.matrix[0][2]*m.matrix[1][0]*m.matrix[2][1]
          - m.matrix[0][2]*m.matrix[1][1]*m.matrix[2][0]
          - m.matrix[0][1]*m.matrix[1][0]*m.matrix[2][2]
          - m.matrix[0][0]*m.matrix[1][2]*m.matrix[2][1];
    
    return det;
  }
  //1x1
  else if(m.row == 1)
  {
    det = m.matrix[0][0];
    return det;
  }  
  else
    return 0;
}

//转置
matrix_instance_s matrix_transpose(matrix_instance_s m)
{
  matrix_instance_s m_t = {0,};
  m_t.row = m.column;
  m_t.column = m.row;
  
  //越界检验
  if(matrix_check(m) == 0)
  {
    m_t.row = 0;
    m_t.column = 0;
    return m_t;
  }
    
  for(uint8_t row = 0; row < m.row; row++)
  {
    for(uint8_t column = 0; column < m.column; column++)
    {
      m_t.matrix[column][row] = m.matrix[row][column];
    }
  }
  
  return m_t;
}

//伴随
matrix_instance_s matrix_adj(matrix_instance_s m)
{
  matrix_instance_s m_adj = {0,};
  
  //越界检验
  if(matrix_check(m) == 0)
  {
    m_adj.row = 0;
    m_adj.column = 0;
    return m_adj;
  }
  
  if(m.column != m.row)
  {
    m_adj.row = 0;
    m_adj.column = 0;
    return m_adj;
  }
    
  m_adj.row = m.row;
  m_adj.column = m.column;
  
  //计算
  //2x2
  if(m.row == 2)
  {     
    m_adj.matrix[0][0] =   m.matrix[1][1];
    m_adj.matrix[0][1] = - m.matrix[0][1];
    m_adj.matrix[1][0] = - m.matrix[1][0];
    m_adj.matrix[1][1] =   m.matrix[0][0]; 
    return m_adj;
  }
  //3x3
  else if(m.row == 3)
  {
    m_adj.matrix[0][0] = m.matrix[1][1]*m.matrix[2][2] - m.matrix[1][2]*m.matrix[2][1];
    m_adj.matrix[0][1] = m.matrix[0][2]*m.matrix[2][1] - m.matrix[0][1]*m.matrix[2][2];
    m_adj.matrix[0][2] = m.matrix[0][1]*m.matrix[1][2] - m.matrix[0][2]*m.matrix[1][1];
    m_adj.matrix[1][0] = m.matrix[1][2]*m.matrix[2][0] - m.matrix[1][0]*m.matrix[2][2];
    m_adj.matrix[1][1] = m.matrix[0][0]*m.matrix[2][2] - m.matrix[0][2]*m.matrix[2][0];
    m_adj.matrix[1][2] = m.matrix[0][2]*m.matrix[1][0] - m.matrix[0][0]*m.matrix[1][2];
    m_adj.matrix[2][0] = m.matrix[1][0]*m.matrix[2][1] - m.matrix[1][1]*m.matrix[2][0];
    m_adj.matrix[2][1] = m.matrix[0][1]*m.matrix[2][0] - m.matrix[0][0]*m.matrix[2][1];
    m_adj.matrix[2][2] = m.matrix[0][0]*m.matrix[1][1] - m.matrix[0][1]*m.matrix[1][0];
    
    return m_adj;
  }
  else if(m.row == 1)
  {
    m_adj.matrix[0][0] = 1;
    return m_adj;
  }
  else
  {
    m_adj.row = 0;
    m_adj.column = 0;
    return m_adj;
  }
}

//求逆
matrix_instance_s matrix_inverse(matrix_instance_s m)
{
  matrix_instance_s m_inv = {0,};
  
  //越界检验
  if(matrix_check(m) == 0)
  {
    m_inv.row = 0;
    m_inv.column = 0;
    return m_inv;
  }  
  
  if(m.column != m.row)
  {
    m_inv.row = 0;
    m_inv.column = 0;
    return m_inv;
  }
  
  matrix_instance_s m_adj = matrix_adj(m);
  float det = matrix_det(m);
  
  //判断是否为奇异矩阵，奇异矩阵不可逆
  if(det == 0)
  {
    m_inv.row = 0;
    m_inv.column = 0;
    return m_inv;
  }
  
  m_inv.row = m.row;
  m_inv.column = m.row;
  
  
  //计算
  //2x2
  if(m.row == 2)
  {     
    m_inv.matrix[0][0] = m_adj.matrix[0][0] / det;
    m_inv.matrix[0][1] = m_adj.matrix[0][1] / det;
    m_inv.matrix[1][0] = m_adj.matrix[1][0] / det;
    m_inv.matrix[1][1] = m_adj.matrix[1][1] / det;
    return m_inv;
  }
  //3x3
  else if(m.row == 3)
  {
    m_inv.matrix[0][0] = m_adj.matrix[0][0] / det;
    m_inv.matrix[0][1] = m_adj.matrix[0][1] / det;
    m_inv.matrix[0][2] = m_adj.matrix[0][2] / det;
    m_inv.matrix[1][0] = m_adj.matrix[1][0] / det;
    m_inv.matrix[1][1] = m_adj.matrix[1][1] / det;
    m_inv.matrix[1][2] = m_adj.matrix[1][2] / det;
    m_inv.matrix[2][0] = m_adj.matrix[2][0] / det;
    m_inv.matrix[2][1] = m_adj.matrix[2][1] / det;
    m_inv.matrix[2][2] = m_adj.matrix[2][2] / det;

    return m_inv;
  }
  //1x1
  else if(m.row == 1)
  {
    m_inv.matrix[0][0] = 1.0f / m.matrix[0][0];
    return m_inv;
  }  
  else
  {
    m_inv.row = 0;
    m_inv.column = 0;
    return m_inv;
  }  
}

//加法
matrix_instance_s matrix_sum(matrix_instance_s a,matrix_instance_s b)
{
  matrix_instance_s m_sum = {0,};
  
  //越界检测
  if(matrix_check(a) == 0 || matrix_check(b) == 0)
  {
    m_sum.row = 0;
    m_sum.column = 0;
    return m_sum;
  }   
  
  
  if(a.row != b.row || a.column != b.column)
  {
    m_sum.row = 0;
    m_sum.column = 0;
    return m_sum;
  }
  
  m_sum.row = a.row;
  m_sum.column = b.column; 

  for(uint8_t row = 0; row < m_sum.row; row++)
  {
    for(uint8_t column = 0; column < m_sum.column; column++)
    {
      m_sum.matrix[row][column] = a.matrix[row][column] + b.matrix[row][column];
    }
  } 

  return m_sum;
}

//数乘
matrix_instance_s matrix_scalar(float num, matrix_instance_s m)
{
  matrix_instance_s c = {0,};
  
  //越界检测
  if(matrix_check(m) == 0)
  {
    c.row = 0;
    c.column = 0;
    return c;
  } 

  c.row = m.row;
  c.column = m.column;
  
  for(uint8_t row = 0; row < c.row; row++)
  {
    for(uint8_t column = 0; column < c.column; column++)
    {
      c.matrix[row][column] = num * m.matrix[row][column];
    }
  } 
  
  return c;
  
}

//矩阵相乘
matrix_instance_s matrix_multiply(matrix_instance_s a,matrix_instance_s b)
{
  matrix_instance_s c = {0,};
  
  //越界检测
  if(matrix_check(a) == 0 || matrix_check(b) == 0)
  {
    c.row = 0;
    c.column = 0;
    return c;
  } 
  
  if(a.column != b.row)
  {
    c.row = 0;
    c.column = 0;
    return c;
  }
  
  c.row = a.row;
  c.column = b.column;
  
  for(uint8_t row = 0; row < c.row; row++)
  {
    for(uint8_t column = 0; column < c.column; column++)
    {
      c.matrix[row][column] = 0;
      for(uint8_t i = 0; i < a.column; i++)
      {
        c.matrix[row][column] += a.matrix[row][i] * b.matrix[i][column];
      }
    }
  } 
  return c;
}

