#include "magnet_model/forward_model.h"

#include "magnet_model/positions.h"

ForwardModel::ForwardModel(const Sensor (&sensors)[3],
                           const MagnetModel (&magnets)[3])
        : sensors_{sensors[0], sensors[1], sensors[2]},
          magnets_{magnets[0], magnets[1], magnets[2]}
    {}

// Everything is built in the member-init list because sensors_ and magnets_ are
// const arrays of types that are themselves const-membered -- there is no
// assign-afterwards path, by design. The model is immutable once built, which is
// what makes "the calibration is constant for the lifetime of the controllers"
// a compiler-enforced statement rather than a convention.
ForwardModel::ForwardModel(const CalibrationParams& cal)
        : sensors_{
              Sensor(Positions::sensor_1_world),
              Sensor(Positions::sensor_2_world),
              Sensor(Positions::sensor_3_world),
          },
          magnets_{
              MagnetModel(CALCULATED_BICUBIC_FIELD, cal.magnet_pos_knob[0], cal.magnet_rotation[0]),
              MagnetModel(CALCULATED_BICUBIC_FIELD, cal.magnet_pos_knob[1], cal.magnet_rotation[1]),
              MagnetModel(CALCULATED_BICUBIC_FIELD, cal.magnet_pos_knob[2], cal.magnet_rotation[2]),
          }
    {}

void __not_in_flash_func(ForwardModel::evaluate)(const Vec3& t,  
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
