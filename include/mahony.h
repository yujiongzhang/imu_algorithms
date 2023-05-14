#ifndef IMU_FILTER_MAHONY_IMU_FILTER_H
#define IMU_FILTER_MAHONY_IMU_FILTER_H

#include "world_frame.h"
#include <iostream>
#include <cmath>

// #define sampleFreq	512.0f			// sample frequency in Hz
#define sampleFreq	100.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

class MahonyFilter
{
  public:
    MahonyFilter();
    virtual ~MahonyFilter();

  private:
    // **** paramaters

    volatile double twoKp = twoKpDef;											// 2 * proportional gain (Kp)
    volatile double twoKi = twoKiDef;											// 2 * integral gain (Ki)
    volatile double q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
    volatile double integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki


    // double gain_;                         // algorithm gain
    // double zeta_;                         // gyro drift bias gain
    // WorldFrame::WorldFrame world_frame_;  // NWU, ENU, NED

    // **** state variables
      
    float w_bx_, w_by_, w_bz_;  //

  public:

    void getOrientation(double& q0, double& q1, double& q2, double& q3)
    {
        q0 = this->q0;
        q1 = this->q1;
        q2 = this->q2;
        q3 = this->q3;

        // perform precise normalization of the output, using 1/sqrt()
        // instead of the fast invSqrt() approximation. Without this,
        // TF2 complains that the quaternion is not normalized.
        double recipNorm = 1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    void setOrientation(double q0, double q1, double q2, double q3)
    {
        this->q0 = q0;
        this->q1 = q1;
        this->q2 = q2;
        this->q3 = q3;

        w_bx_ = 0;
        w_by_ = 0;
        w_bz_ = 0;
    }

    void mahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay,
                            float az, float mx, float my, float mz, float dt);

    void mahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay,
                               float az, float dt);

    void getGravity(float& rx, float& ry, float& rz, float gravity = 9.80665);

    //! \brief Reset the filter to the initial state.
    void reset();
};

#endif  
