#ifndef IMU_H
#define IMU_H

#include <stdint.h>

float imu_inv_sqrt(const float x);
void imu_update_filter(float w_x, float w_y, float w_z,
                       float a_x, float a_y, float a_z,
                       float m_x, float m_y, float m_z,
                       float imu_filter_beta,
                       float imu_filter_zeta,
                       float imu_filter_delta_t,// sampling period in seconds e.g. 0.002f
                       float state[9]//w x y z ref_b_x ref_b_z w_bias_x w_bias_y w_bias_z
);
#endif
