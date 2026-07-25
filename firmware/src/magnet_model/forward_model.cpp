#include "magnet_model/forward_model.h"

ForwardModel::ForwardModel(const Vec3 sensor_positions[3], const MagnetModel* magnet_models)
    : magnets_(magnet_models) {
    for (int i = 0; i < 3; ++i) {
        sensors_[i] = sensor_positions[i];
    }
}

void ForwardModel::evaluate(const Vec3& t,  
                            const Mat3& R, 
                            const Vec3 measured_fields[3], 
                            Eigen::Matrix<float, 9, 1> &residual, 
                            Eigen::Matrix<float, 9, 6> &J) const {
                            
    Mat3 R_T = R.transpose();

    for (int i = 0; i < 3; ++i) {
        // 1. Global geometry
        Vec3 v = sensors_[i] - t;
        
        // 2. Transform to local knob frame
        Vec3 magnet_pos_knob = magnets_[i].get_m_local();
        Vec3 sensor_pos_knob = R_T * (sensors_[i] - t);
        Vec3 sensor_magnet_rel = sensor_pos_knob - magnet_pos_knob;

        // 3. Evaluate local field and Jacobian
        Mat3 J_local;
        Vec3 B_local = magnets_[i].evaluate(sensor_magnet_rel, J_local);

        // 4. Rotate field back to global frame to get the field as measured by the sensors
        Vec3 B_global = R * B_local;

        // 5. Populate the residual vector (Predicted - Measured)
        residual[i*3 + 0] = B_global[0] - measured_fields[i][0];
        residual[i*3 + 1] = B_global[1] - measured_fields[i][1];
        residual[i*3 + 2] = B_global[2] - measured_fields[i][2];

        // 6. Assemble the Jacobian blocks
        Mat3 M = (R * J_local) * R_T;
        Mat3 J_trans = M * -1.0f;
        
        Mat3 v_skew = skew_matrix(v);
        Mat3 B_global_skew = skew_matrix(B_global);
        
        // J_rot = M * [v]_x - [B_global]_x
        Mat3 M_v_skew = M * v_skew;
        Mat3 J_rot = (M * skew_matrix(v)) - skew_matrix(B_global);

        // 7. Copy blocks into the flat 9x6 global Jacobian
        J.block<3, 3>(i * 3, 0) = J_trans;
        J.block<3, 3>(i * 3, 3) = J_rot; 
    }
}