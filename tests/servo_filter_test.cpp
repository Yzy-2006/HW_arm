#include <bits/stdc++.h>
#include <yaml-cpp/yaml.h>
#define private public
#include "JntController/ForceComp.h"
#undef private

double TEMPTIME = .001;
int LARMDOF = 7, RARMDOF = 7;

int main() {
    for (double q : {1.0, 12000.0}) {
        kalman_filter f;
        f.FilterInit(0, q, q == 1 ? .01 : 300, TEMPTIME);
        for (int n = 0; n < 180000; ++n) {
            const double result = f.FilteredResCal(.01 * std::sin(2 * M_PI * 2 * n * TEMPTIME));
            assert(std::isfinite(result) && std::abs(result) < .03);
            assert(f.Pmtrx[0][0] >= 0 && f.Pmtrx[1][1] >= 0);
            assert(std::abs(f.Pmtrx[0][1] - f.Pmtrx[1][0]) < 1e-10);
            assert(f.Pmtrx[0][0] * f.Pmtrx[1][1] - f.Pmtrx[0][1] * f.Pmtrx[1][0] >= -1e-8);
        }
    }
    // Initializing away from zero must preserve a stationary trajectory.
    runin_control interpolator;
    std::vector<double> position(7, 1.0), zero(7, 0.0), kp(7, 100), kd(7, 12);
    interpolator.init(position, position, 3);
    interpolator.ModelRunin(position, zero, zero, zero, zero,
                            position, zero, zero, zero, zero, kp, kd, kp, kd, 1, 3);
    for (int i = 0; i < 5; ++i) {
        auto result = interpolator.dataOutp();
        for (double p : result.runinData.lArm_posDot) assert(std::abs(p - 1) < 1e-9);
    }
}
