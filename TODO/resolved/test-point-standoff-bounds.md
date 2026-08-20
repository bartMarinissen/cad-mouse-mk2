> **RESOLVED — fixed, archived for the reasoning.**
>
> `t_z_steps` now derives from `Positions::magnet_z_pos_from_pivot + standoff`
> instead of hardcoding the raw standoff as `t.z`; `BASE_T`/`BASE_ROTATION_AXIS`
> (unused dead code carrying the identical mistake) were deleted. Fixed in
> `85146bd`. See `test_forward_model_jacobian_grid` and
> `test_cross_magnet_terms_are_actually_present` in
> `firmware/test/test_jacobian.cpp` for the live pattern — the latter already
> did this correctly and is what the fix now matches.
>
> Verified with `pio test -e native_test`: all 8 cases pass, including
> `test_forward_model_jacobian_grid` now exercising genuinely in-domain
> points. `pio test -e seeed_xiao_rp2040_test` was not run to completion for
> this fix (no physical board attached in the environment it was fixed in),
> though the build itself completed without error before the upload stage.

---

# `test_forward_model_jacobian_grid`'s pose sweep evaluates points outside the bicubic table's domain

`firmware/test/test_jacobian.cpp` validates every analytic Jacobian in the
solver chain by finite-differencing it at hand-picked test points. Most of
those points are either local-frame `(r,z)`/`v_l` probes with no geometry to
derive (fine as literals), or derived correctly from
`firmware/include/magnet_model/positions.h`'s `Positions::` constants. One
isn't.

`test_forward_model_jacobian_grid`'s `t_z_steps` is `{8.0f, 6.0f, 4.3f,
2.5f}`, used directly as `t.z`, where `t` is the knob pivot's world position
passed into `ForwardModel::evaluate(t, R, ...)`. But
`MagnetModel::place` (`firmware/src/magnet_model/magnet_local_model.cpp`)
computes `origin_world = t + R * magnet_pos_knob`, and
`magnet_pos_knob.z = -Positions::magnet_z_pos_from_pivot` (`-15`). A physical
sensor-to-magnet standoff `s` therefore requires
`t.z = Positions::magnet_z_pos_from_pivot + s`, not `s` alone —
exactly the derivation `test_cross_magnet_terms_are_actually_present` already
uses (`Positions::magnet_z_pos_from_pivot + standoff`) and that
`Positions::approx_rest_pos` itself encodes
(`magnet_z_pos_from_pivot + magnet_rest_distance_sensor`).

Skipping the +15mm offset sends the local-frame `z` fed into the bicubic
table **positive** (roughly +7 to +12.5mm across the sweep) instead of
negative. That's not merely outside the table's `z ∈ [-20,-0.5]` domain
(`magnet_model_table.h`'s `BICUBIC_ORIGIN`/`BICUBIC_FAR`) — it's the wrong
sign entirely, nowhere near the physical region the sweep is meant to cover.

This doesn't currently fail the test: `BicubicField::evaluate` clamps the
cell index and falls back to linear extrapolation outside the grid rather
than erroring, and the test only checks the analytic Jacobian against a
finite-difference Jacobian of that same (extrapolated) function — a
self-consistency check that passes regardless of whether the evaluated point
is physically meaningful. So the "Massive Grid Test" silently exercises
extrapolation on the wrong side of the table instead of the near-table poses
it's supposed to validate, without any assertion failure.

The file's own `test_magnet_model_jacobian_generic` already carries an
unresolved comment flagging exactly this suspicion: "`test_forward_model_
jacobian_grid`'s pose sweep is not checked against this [-12,-0.5]/r<=6
margin — confirm it actually stays inside it, or the margin claim here is
unverified."

Separately, `BASE_T` (`Vec3(2.0f, -1.5f, 6.0f)`) and `BASE_ROTATION_AXIS` are
declared near the top of the file but never referenced anywhere else in it.
`BASE_T.z = 6.0f` carries the identical bug — a raw standoff used as `t.z`
with no pivot offset — labeled in its own comment as "the real standoff."

## Likely fix shape

Derive `t_z_steps` the same way `test_cross_magnet_terms_are_actually_present`
already does: `Positions::magnet_z_pos_from_pivot + standoff` for each of the
same standoff values, so the physical scenario being swept is unchanged and
only the world-frame `z` it turns into is corrected. Delete `BASE_T`/
`BASE_ROTATION_AXIS` rather than fixing them, since nothing references either
one.
