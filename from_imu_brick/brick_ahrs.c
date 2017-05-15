#include <math.h>
#include "brick_ahrs.h"

//#define IMU_DEFAULT_CONVERGENCE_SPEED 30

// sqrtf(3.0f / 4.0f)
//#define SQRT3DIV4 0.8660254037844386

// gyroscope measurement error in rad/s (shown as 30 deg/s)
//float imu_filter_beta = SQRT3DIV4 * M_PI * \
//(IMU_DEFAULT_CONVERGENCE_SPEED / 180.0f);

// gyroscope measurement error in rad/s/s (shown as 0.5f deg/s/s)
//float imu_filter_zeta = SQRT3DIV4 * M_PI * \
//((IMU_DEFAULT_CONVERGENCE_SPEED/60.0f) / 180.0f);


float imu_inv_sqrt(const float x) {
    return 1.0f/sqrtf(x);
}


// Based on: S. O. Madgwick, "An efficient orientation filter for inertial and
// inertial/magnetic sensor arrays", University of Bristol, April 2010.
void imu_update_filter(float w_x, float w_y, float w_z,
                       float a_x, float a_y, float a_z,
                       float m_x, float m_y, float m_z,
                       float imu_filter_beta,//0.45375 for 30 deg/s
                       float imu_filter_zeta,//0.00756 for 0.5 deg/s/s
                       float imu_filter_delta_t,// sampling period in seconds e.g. 0.002f
                       float state[9]//w x y z ref_b_x ref_b_z w_bias_x w_bias_y w_bias_z
) {
    // estimated orientation quaternion elements with initial conditions
    float imu_filter_seq_1 = state[0]; // 1.0
    float imu_filter_seq_2 = state[1];
    float imu_filter_seq_3 = state[2];
    float imu_filter_seq_4 = state[3];
    
    // reference direction of flux in earth frame
    float imu_filter_b_x = state[4];//1.0
    float imu_filter_b_z = state[5];//0.0
    
    
    // estimate gyroscope biases error
    float imu_filter_w_bx = state[6];
    float imu_filter_w_by = state[7];
    float imu_filter_w_bz = state[8];
    
	// Normalize accelerometer measurement
	float norm = imu_inv_sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x *= norm;
	a_y *= norm;
	a_z *= norm;
    
	// Normalize magnetometer measurement
	norm = imu_inv_sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
	m_x *= norm;
	m_y *= norm;
	m_z *= norm;
    
	// auxiliary variables to avoid repeated calculations
	const float halfSEq_1 = 0.5f * imu_filter_seq_1;
	const float halfSEq_2 = 0.5f * imu_filter_seq_2;
	const float halfSEq_3 = 0.5f * imu_filter_seq_3;
	const float halfSEq_4 = 0.5f * imu_filter_seq_4;
	const float twoSEq_1 = 2.0f * imu_filter_seq_1;
	const float twoSEq_2 = 2.0f * imu_filter_seq_2;
	const float twoSEq_3 = 2.0f * imu_filter_seq_3;
	const float twoSEq_4 = 2.0f * imu_filter_seq_4;
	const float twob_x = 2.0f * imu_filter_b_x;
	const float twob_z = 2.0f * imu_filter_b_z;
	const float twob_xSEq_1 = 2.0f * imu_filter_b_x * imu_filter_seq_1;
	const float twob_xSEq_2 = 2.0f * imu_filter_b_x * imu_filter_seq_2;
	const float twob_xSEq_3 = 2.0f * imu_filter_b_x * imu_filter_seq_3;
	const float twob_xSEq_4 = 2.0f * imu_filter_b_x * imu_filter_seq_4;
	const float twob_zSEq_1 = 2.0f * imu_filter_b_z * imu_filter_seq_1;
	const float twob_zSEq_2 = 2.0f * imu_filter_b_z * imu_filter_seq_2;
	const float twob_zSEq_3 = 2.0f * imu_filter_b_z * imu_filter_seq_3;
	const float twob_zSEq_4 = 2.0f * imu_filter_b_z * imu_filter_seq_4;
	const float twom_x = 2.0f * m_x;
	const float twom_y = 2.0f * m_y;
	const float twom_z = 2.0f * m_z;
    
	float SEq_1SEq_2;
	float SEq_1SEq_3 = imu_filter_seq_1 * imu_filter_seq_3;
	float SEq_1SEq_4;
	float SEq_2SEq_3;
	float SEq_2SEq_4 = imu_filter_seq_2 * imu_filter_seq_4;
	float SEq_3SEq_4;
    
	// compute the objective function and Jacobian
	const float f_1 = twoSEq_2 * imu_filter_seq_4 -
    twoSEq_1 * imu_filter_seq_3 - a_x;
	const float f_2 = twoSEq_1 * imu_filter_seq_2 +
    twoSEq_3 * imu_filter_seq_4 - a_y;
	const float f_3 = 1.0f -
    twoSEq_2 * imu_filter_seq_2 -
    twoSEq_3 * imu_filter_seq_3 - a_z;
	const float f_4 = twob_x * (0.5f -
	                            imu_filter_seq_3 * imu_filter_seq_3 -
	                            imu_filter_seq_4 * imu_filter_seq_4) +
    twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
	const float f_5 = twob_x * (imu_filter_seq_2 * imu_filter_seq_3 -
	                            imu_filter_seq_1 * imu_filter_seq_4) +
    twob_z * (imu_filter_seq_1 * imu_filter_seq_2 +
              imu_filter_seq_3 * imu_filter_seq_4) - m_y;
	const float f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) +
    twob_z * (0.5f -
              imu_filter_seq_2 * imu_filter_seq_2 -
              imu_filter_seq_3 * imu_filter_seq_3) - m_z;
    
	// J_11 negated in matrix multiplication
	const float J_11or24 = twoSEq_3;
	const float J_12or23 = 2.0f * imu_filter_seq_4;
	// J_12 negated in matrix multiplication
	const float J_13or22 = twoSEq_1;
	const float J_14or21 = twoSEq_2;
	// negated in matrix multiplication
	const float J_32 = 2.0f * J_14or21;
	// negated in matrix multiplication
	const float J_33 = 2.0f * J_11or24;
	// negated in matrix multiplication
	const float J_41 = twob_zSEq_3;
	const float J_42 = twob_zSEq_4;
	// negated in matrix multiplication
	const float J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1;
	// negated in matrix multiplication
	const float J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2;
	// negated in matrix multiplication
	const float J_51 = twob_xSEq_4 - twob_zSEq_2;
	const float J_52 = twob_xSEq_3 + twob_zSEq_1;
	const float J_53 = twob_xSEq_2 + twob_zSEq_4;
	// negated in matrix multiplication
	const float J_54 = twob_xSEq_1 - twob_zSEq_3;
	const float J_61 = twob_xSEq_3;
	const float J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
	const float J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
	const float J_64 = twob_xSEq_2;
    
	// compute the gradient (matrix multiplication)
	float SEqHatDot_1 = J_14or21 * f_2 -
    J_11or24 * f_1 -
    J_41 * f_4 -
    J_51 * f_5 +
    J_61 * f_6;
	float SEqHatDot_2 = J_12or23 * f_1 +
    J_13or22 * f_2 -
    J_32 * f_3 +
    J_42 * f_4 +
    J_52 * f_5 +
    J_62 * f_6;
	float SEqHatDot_3 = J_12or23 * f_2 -
    J_33 * f_3 -
    J_13or22 * f_1 -
    J_43 * f_4 +
    J_53 * f_5 +
    J_63 * f_6;
	float SEqHatDot_4 = J_14or21 * f_1 +
    J_11or24 * f_2 -
    J_44 * f_4 -
    J_54 * f_5 +
    J_64 * f_6;
    
	// normalise the gradient to estimate direction of the gyroscope error
	norm = imu_inv_sqrt(SEqHatDot_1 * SEqHatDot_1 +
	                    SEqHatDot_2 * SEqHatDot_2 +
	                    SEqHatDot_3 * SEqHatDot_3 +
	                    SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 *= norm;
	SEqHatDot_2 *= norm;
	SEqHatDot_3 *= norm;
	SEqHatDot_4 *= norm;
    
	// compute angular estimated direction of the gyroscope error
	const float w_err_x = twoSEq_1 * SEqHatDot_2 -
    twoSEq_2 * SEqHatDot_1 -
    twoSEq_3 * SEqHatDot_4 +
    twoSEq_4 * SEqHatDot_3;
	const float w_err_y = twoSEq_1 * SEqHatDot_3 +
    twoSEq_2 * SEqHatDot_4 -
    twoSEq_3 * SEqHatDot_1 -
    twoSEq_4 * SEqHatDot_2;
	const float w_err_z = twoSEq_1 * SEqHatDot_4 -
    twoSEq_2 * SEqHatDot_3 +
    twoSEq_3 * SEqHatDot_2 -
    twoSEq_4 * SEqHatDot_1;
    
	// compute and remove the gyroscope baises
	imu_filter_w_bx += w_err_x * imu_filter_delta_t * imu_filter_zeta;
	imu_filter_w_by += w_err_y * imu_filter_delta_t * imu_filter_zeta;
	imu_filter_w_bz += w_err_z * imu_filter_delta_t * imu_filter_zeta;
	w_x -= imu_filter_w_bx;
	w_y -= imu_filter_w_by;
	w_z -= imu_filter_w_bz;
    
	// compute the quaternion rate measured by gyroscopes
	const float SEqDot_omega_1 = -halfSEq_2 * w_x -
    halfSEq_3 * w_y -
    halfSEq_4 * w_z;
	const float SEqDot_omega_2 = halfSEq_1 * w_x +
    halfSEq_3 * w_z -
    halfSEq_4 * w_y;
	const float SEqDot_omega_3 = halfSEq_1 * w_y -
    halfSEq_2 * w_z +
    halfSEq_4 * w_x;
	const float SEqDot_omega_4 = halfSEq_1 * w_z +
    halfSEq_2 * w_y -
    halfSEq_3 * w_x;
    
	// compute then integrate the estimated quaternion rate
	imu_filter_seq_1 += (SEqDot_omega_1 -
	                     (imu_filter_beta * SEqHatDot_1)) * imu_filter_delta_t;
	imu_filter_seq_2 += (SEqDot_omega_2 -
	                     (imu_filter_beta * SEqHatDot_2)) * imu_filter_delta_t;
	imu_filter_seq_3 += (SEqDot_omega_3 -
	                     (imu_filter_beta * SEqHatDot_3)) * imu_filter_delta_t;
	imu_filter_seq_4 += (SEqDot_omega_4 -
	                     (imu_filter_beta * SEqHatDot_4)) * imu_filter_delta_t;
    
	// normalize quaternion
	norm = imu_inv_sqrt(imu_filter_seq_1 * imu_filter_seq_1 +
	                    imu_filter_seq_2 * imu_filter_seq_2 +
	                    imu_filter_seq_3 * imu_filter_seq_3 +
	                    imu_filter_seq_4 * imu_filter_seq_4);
	imu_filter_seq_1 *= norm;
	imu_filter_seq_2 *= norm;
	imu_filter_seq_3 *= norm;
	imu_filter_seq_4 *= norm;
    
	// compute flux in the earth frame
	SEq_1SEq_2 = imu_filter_seq_1 * imu_filter_seq_2;
	SEq_1SEq_3 = imu_filter_seq_1 * imu_filter_seq_3;
	SEq_1SEq_4 = imu_filter_seq_1 * imu_filter_seq_4;
	SEq_3SEq_4 = imu_filter_seq_3 * imu_filter_seq_4;
	SEq_2SEq_3 = imu_filter_seq_2 * imu_filter_seq_3;
	SEq_2SEq_4 = imu_filter_seq_2 * imu_filter_seq_4;
    
	const float h_x = twom_x * (0.5f -
	                            imu_filter_seq_3 * imu_filter_seq_3 -
	                            imu_filter_seq_4 * imu_filter_seq_4) +
    twom_y * (SEq_2SEq_3 - SEq_1SEq_4) +
    twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    
	const float h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) +
    twom_y * (0.5f -
              imu_filter_seq_2 * imu_filter_seq_2 -
              imu_filter_seq_4 * imu_filter_seq_4) +
    twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
    
	const float h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) +
    twom_y * (SEq_3SEq_4 + SEq_1SEq_2) +
    twom_z * (0.5f -
              imu_filter_seq_2 * imu_filter_seq_2 -
              imu_filter_seq_3 * imu_filter_seq_3);
    
	// normalize the flux vector to have only components in the x and z
	imu_filter_b_x = sqrtf((h_x * h_x) + (h_y * h_y));
	imu_filter_b_z = h_z;
    
    state[0]=imu_filter_seq_1;
    state[1]=imu_filter_seq_2;
    state[2]=imu_filter_seq_3;
    state[3]=imu_filter_seq_4;
    
    state[4]=imu_filter_b_x;
    state[5]=imu_filter_b_z;
    
    state[6]=imu_filter_w_bx;
    state[7]=imu_filter_w_by;
    state[8]=imu_filter_w_bz;

}

int main()
{ return 0;/*
    std::cout<<imu_filter_seq_1<<" "<<imu_filter_seq_2<<" "<<imu_filter_seq_3<<" "<<imu_filter_seq_4<<std::endl;
    
    while(1){
        float gx; float gy; float gz; float ax; float ay; float az; float mx; float my; float mz; float b;
        int n;
        std::cin>>b>>gx>>gy>>gz>>ax>>ay>>az>>mx>>my>>mz>>n;
        imu_filter_beta = b;
        if(b==-2)
            return 0;
        
        for(int i=0;i<n;i++){
            imu_update_filter(gx,gy,gz,ax,ay,az,mx,my,mz);
        }
        std::cout<<imu_filter_seq_1<<" "<<imu_filter_seq_2<<" "<<imu_filter_seq_3<<" "<<imu_filter_seq_4<<std::endl;
    }*/
}

