#include "magnet_model/sensor.h"

Sensor::Sensor(Vec3 sensor_pos_global, Mat3 sensor_gain )
    : sensor_pos_global(sensor_pos_global), sensor_gain(sensor_gain)
    {}

// Evaluate the field a sensor sees from a given magnet, and a given translation from the global frame to the knob frame
void Sensor::evaluate(const MagnetModel &magnet, const Vec3& t,  
        const Mat3& R, 
        Eigen::Matrix<float, 3, 1> &B_field_global, 
        Eigen::Matrix<float, 3, 6> &J) const {
   
    Mat3 R_T = R.transpose();
    const Mat3 &R_mag = magnet.magnet_rotation;

    // 1. Global geometry
    Vec3 v = sensor_pos_global - t;
    
    // 2. Transform to local knob frame
    const Vec3 &magnet_pos_knob = magnet.magnet_pos_knob;
    Vec3 sensor_pos_knob = R_T * (sensor_pos_global - t);

    // 2.1. Transform to local magnet frame
    Vec3 sensor_magnet_rel = magnet.magnet_rotation.transpose() * (sensor_pos_knob - magnet_pos_knob);

    // 3. Evaluate local field and Jacobian
    Mat3 J_local;
    Vec3 B_local = magnet.evaluate(sensor_magnet_rel, J_local);

    // 3.1 Rotate back to knob frame
    J_local = R_mag * J_local * R_mag.transpose();
    B_field_global = R_mag * B_local;

    // 4. Rotate field back to global frame to get the field as measured by the sensors
    B_field_global = R *  B_local;
    
    // 6. Assemble the Jacobian blocks
    Mat3 M = R * J_local * R_T;
    Mat3 J_trans = M * -1.0f;
      
    // J_rot = M * [v]_x - [B_field_global]_x
    Mat3 J_rot = (M * skew_matrix(v)) - skew_matrix(B_field_global);

    // 7. Apply the sensor gain
    B_field_global = sensor_gain * B_field_global;
    J.block<3, 3>(0, 0) = sensor_gain * J_trans;
    J.block<3, 3>(0, 3) = sensor_gain * J_rot; 
}
