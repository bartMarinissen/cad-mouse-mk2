#pragma once
// Host-build stub: BicubicField.h includes <Arduino.h> but (as of this
// writing) uses nothing from it directly -- constexpr/noexcept are
// standard C++, not Arduino macros. Empty on purpose.

// Minimal Serial/delay stubs so firmware sources that log on an error path
// (solve_pose.cpp's NAN guard) link in host builds. Never expected to fire.
#include <cstdio>
struct SerialStub {
    void println(const char* m) { std::printf("%s\n", m); }
    void print(const char* m) { std::printf("%s", m); }
    template <class... A> void printf(const char* f, A... a) { std::printf(f, a...); }
    void begin(unsigned long) {}
    explicit operator bool() const { return true; }
};
inline SerialStub Serial;
inline void delay(unsigned long) {}
