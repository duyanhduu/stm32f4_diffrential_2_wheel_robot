#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <MadgwickAHRS.h>
#include "gy85_driver.h" // Chứa struct ImuData

extern Madgwick filter;
extern float q0, q1, q2, q3; // Quaternion xuất ra

void initImuFilter(float sampleFrequency);
void updateImuFilter(const ImuData &rawData);

#endif /* IMU_FILTER_H */