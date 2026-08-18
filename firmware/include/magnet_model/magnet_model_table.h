
#pragma once
#include "math3D.h"

struct Point { float r; float z; };

constexpr int NR = 51;
constexpr int NZ = 91;

constexpr Point BICUBIC_ORIGIN = { 0.0, -20.0 };
constexpr Point BICUBIC_FAR    = { 10.0, -0.5 };

// The polarization (remanence, Br) this table was generated at. A real
// magnet is not this strong or weak -- MagnetModel divides its own
// magnet_strength_mT by this to get the ratio it scales the table by.
constexpr float BICUBIC_FIELD_REFERENCE_MT = 1000.0f;

// --- Far-field (dipole) constants, for the cross-magnet terms ---
//
// The table above covers one magnet's own sensor. Every sensor also sees the
// other two magnets, far enough away to be modelled as point dipoles instead
// of interpolated; see design documentation/Math.md for the derivation and
// dipole_field() in magnet_local_model.h for the implementation.

// Where the equivalent point dipole sits in the magnet's local frame. The
// frame's origin is the magnet's BOTTOM FACE (see local_field.py), but the
// dipole belongs at the geometric centre -- placing it at the origin instead
// is not a small error, it is tens of percent at cross-magnet range.
constexpr float MAGNET_HALF_HEIGHT_MM = 3.0f;

// Dipole moment magnitude at BICUBIC_FIELD_REFERENCE_MT, in mT*mm^3, so it
// scales by exactly the same ratio the table does and the two models can
// never describe magnets of different strength.
//
// Units: with the moment in mT*mm^3 and distances in mm, the field
//   B = (1/4pi)(3(m.rhat)rhat - m)/rho^3
// comes out in mT with no further conversion. NOTE this is polarization x
// volume, which is NOT magpylib's `dipole_moment` property -- that one is
// SI (A*m^2) and differs from this by a factor of 1/mu0.
constexpr float DIPOLE_MOMENT_AT_REFERENCE_MT_MM3 = 169646.00329384883f;

extern const Vec2 BICUBIC_INTERPOLATION_TABLE[NZ][NR];

