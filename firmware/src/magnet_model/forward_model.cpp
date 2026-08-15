#include "magnet_model/forward_model.h"

#include "magnet_model/positions.h"

ForwardModel::ForwardModel(const VirtualSensor (&sensors)[3],
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
              VirtualSensor(Positions::sensor_1_world),
              VirtualSensor(Positions::sensor_2_world),
              VirtualSensor(Positions::sensor_3_world),
          },
          magnets_{
              MagnetModel(CALCULATED_BICUBIC_FIELD, toVec3(cal.magnet_pos_knob[0]), toMat3(cal.magnet_rotation[0]), cal.magnet_strength_mT[0]),
              MagnetModel(CALCULATED_BICUBIC_FIELD, toVec3(cal.magnet_pos_knob[1]), toMat3(cal.magnet_rotation[1]), cal.magnet_strength_mT[1]),
              MagnetModel(CALCULATED_BICUBIC_FIELD, toVec3(cal.magnet_pos_knob[2]), toMat3(cal.magnet_rotation[2]), cal.magnet_strength_mT[2]),
          }
    {}

void __not_in_flash_func(ForwardModel::evaluate)(const Vec3& t,
                            const Mat3& R,
                            Eigen::Matrix<float, 9, 1> &B_field,
                            Eigen::Matrix<float, 9, 6> &J) const {

    // Place each magnet in the world once, ahead of the sensor loop. Every
    // sensor sees every magnet, so these three would otherwise be rebuilt
    // three times each -- and R * magnet_rotation alone is a full 3x3 product.
    const MagnetPlacement placements[3] = {
        magnets_[0].place(t, R),
        magnets_[1].place(t, R),
        magnets_[2].place(t, R),
    };

    for (int i = 0; i < 3; ++i) {

        // Sensor i sits under magnet i; the other two are the cross terms.
        // Fixed by the knob's layout, so it is an index relationship rather
        // than anything measured per call.
        const int cross_a = (i + 1) % 3;
        const int cross_b = (i + 2) % 3;

        // 4. Rotate field back to global frame to get the field as measured by the sensors
        Vec3 B_sensor_magnet_global;
        Eigen::Matrix<float, 3, 6> J_sensor_magnet;

        sensors_[i].evaluate(magnets_[i], placements[i],
                             placements[cross_a], placements[cross_b],
                             t, B_sensor_magnet_global, J_sensor_magnet);

        // 7. Copy blocks into the flat 9x6 global Jacobian and the expected field
        J.block<3, 6>(i * 3, 0) = J_sensor_magnet;
        B_field.block<3, 1>(i * 3, 0) = B_sensor_magnet_global;
    }
}
