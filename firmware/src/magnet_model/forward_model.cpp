#include "magnet_model/forward_model.h"

ForwardModel::ForwardModel(const Sensor (&sensors)[3], 
                           const MagnetModel (&magnets)[3])
        : sensors_{sensors[0], sensors[1], sensors[2]},
          magnets_{magnets[0], magnets[1], magnets[2]} 
    {}

void ForwardModel::evaluate(const Vec3& t,  
                            const Mat3& R, 
                            Eigen::Matrix<float, 9, 1> &B_field, 
                            Eigen::Matrix<float, 9, 6> &J) const {
                            
    Mat3 R_T = R.transpose();

    for (int i = 0; i < 3; ++i) {

        // 4. Rotate field back to global frame to get the field as measured by the sensors
        Vec3 B_sensor_magnet_global;
        Eigen::Matrix<float, 3, 6> J_sensor_magnet;

        sensors_[i].evaluate(magnets_[i], t, R, B_sensor_magnet_global, J_sensor_magnet);

        // 7. Copy blocks into the flat 9x6 global Jacobian and the expected field
        J.block<3, 6>(i * 3, 0) = J_sensor_magnet;
        B_field.block<3, 1>(i * 3, 0) = B_sensor_magnet_global;
    }
}
