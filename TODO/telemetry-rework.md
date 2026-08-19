# Telemetry rework

Originated from ARCHITECTURE.md issues #7 and #8, merged into one effort per
discussion.

## Issue A: `rcond` is a hardcoded stub — RESOLVED

Done, upstream of this pass: `IdleState.cpp`'s `float rcond = 1.0;` stub and
its commented-out `jacobiSvd()` line are gone, and
`TelemetryController::publish()` no longer takes an `rcond` parameter at
all (the cross-magnet-refactor branch dropped both; ported as dropped, see
`TODO/eigen-to-bla-migration.md`'s "Extended" section).

## Issue B: `TelemetryController::publish()`'s signature keeps growing

Currently 8 parameters (`motion`, `residual_percent`, `buttonBits`,
`hidReportSent`, `stats`, `raw_field`, `last_pos`, `last_rot` — `rcond` is
gone, Issue A above) in
`firmware/include/controllers/TelemetryController.h` /
`firmware/src/controllers/TelemetryController.cpp`, and it's grown with every
diagnostic added so far. 

## Issue C: we want more telemetry.
Specifically, we want to know the number of iterations the solve took.
Ideally we want some idea of how well the solve did (e.g. optimiality of the 
jacobian, or something based on the local gradient).

The work here isn't just threading this data into the telemetry, but also figuring out how to display it.

## Issue D: profiling
We want to be able to profile things.
For that, we should have a separate end-point at the telemetry controller (in the header, for inlining)
to submit such information. With a separate way to get the profile information out.

The interface should have a header-defined struct of things to profile. With one entry per interesting thing.
Then at the call site we do: `telemetry.profile(ENUM_CONSTANT_THAT_DEFINES_OPERATION).start()` before
and `telemetry.profile(ENUM_CONSTANT_THAT_DEFINES_OPERATION).stop()` after.
Start just subtracts the current time from the accumulator, and stop adds the current time to the accumulator.
Stop also increments a total runs counter.

Some extra work to allow turning these function calls for a specific operation into a no-op would also be nice.
