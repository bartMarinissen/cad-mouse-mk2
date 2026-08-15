#pragma once
#include "math3D.h"
#include "magnet_local_model.h"


struct VirtualSensor {
    Vec3 sensor_pos_global;

    VirtualSensor(Vec3 sensor_pos_global);

    // The field this sensor sees from ALL THREE magnets, and its 3x6 Jacobian
    // with respect to the knob pose.
    //
    // The two kinds of magnet are separate arguments rather than an array plus
    // an index because they are modelled differently and the call site should
    // say so: `paired` is the magnet this sensor sits under, close enough to
    // need the interpolated near field, while `cross_a`/`cross_b` are a
    // triangle side away and are point dipoles. Which is which is fixed by the
    // knob's geometry, not decided per call -- the paired magnet never gets
    // far enough away, and the cross magnets never get close enough, for the
    // assignment to be wrong (design documentation/Math.md).
    //
    // The cross magnets arrive as MagnetPlacement alone, with no MagnetModel:
    // a dipole needs only a moment and a position, which is precisely why it
    // is cheap enough to add six of per evaluation.
    //
    // Superposition is applied to the field and its world-frame gradient
    // BEFORE the 3x6 blocks are assembled. Both blocks are linear in those two
    // quantities, so summing first is exact, and it means the skew products
    // and block assembly run once per sensor instead of once per pair.
    void evaluate(const MagnetModel &paired, const MagnetPlacement &paired_placement,
        const MagnetPlacement &cross_a,
        const MagnetPlacement &cross_b,
        const Vec3& t,
        Eigen::Matrix<float, 3, 1> &B_field_global,
        Eigen::Matrix<float, 3, 6> &J) const;
};
