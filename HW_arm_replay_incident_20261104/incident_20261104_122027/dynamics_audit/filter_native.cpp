// Offline C ABI around unmodified production filter methods. No ROS dependency.
#include <bits/stdc++.h>
#define private public
#include "controller/JntController.h"
#undef private

double TEMPTIME = 1.0 / 300.0;
extern "C" {
void* filter_create(double initial, double q, double r) {
    auto* f = new Kernel::Filter;
    f->FilterInit(initial, q, r);
    return f;
}
void filter_destroy(void* pointer) { delete static_cast<Kernel::Filter*>(pointer); }
double filter_step(void* pointer, double input, int older) {
    auto* f = static_cast<Kernel::Filter*>(pointer);
    return older ? f->FilteredResCaloldVersion(input) : f->FilteredResCal(input);
}
double filter_next_gain(void* pointer) {
    auto* f = static_cast<Kernel::Filter*>(pointer);
    double predicted = f->Pmtrx[0][0] + .001 * f->Pmtrx[1][0] + (.001 * .001 * .001 / 3.0);
    return predicted / (predicted + .01 * .01);
}
}
