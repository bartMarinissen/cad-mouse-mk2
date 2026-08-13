// Host-build entry point: test_bundle_shared_jacobian.cpp is written as an
// Arduino sketch (setup()/loop(), matching firmware/test/test_jacobian.cpp's
// own style) so it can eventually move into firmware/test/ verbatim. This
// just gives it a real main() to run under a plain host g++ build.
void setup();
int main() { setup(); return 0; }
