// Link this probe to the unmodified production JntController.cpp.
// Expose state in this diagnostic translation unit only; never run a node.
#include <bits/stdc++.h>
#define private public
#include "controller/JntController.h"
#undef private

double TEMPTIME = 1.0 / 300.0;

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    std::ofstream stream(argv[1]);
    stream << std::setprecision(17);
    stream << "iteration,time_at_300hz,input,output,gain,p00,p11,asymmetry,old_version_output\n";
    Kernel::Filter actual, older;
    actual.FilterInit(0.0, 1.0, 0.01);
    older.FilterInit(0.0, 1.0, 0.01);
    double max_gain = 0, max_output = 0, old_max_output = 0;
    int gain_iteration = 0, output_iteration = 0, first_negative_p00 = 0;
    for (int n = 1; n <= 36000; ++n) {
        const double input = 0.01 * std::sin(2 * M_PI * 2 * n / 300.0);
        const double predicted = actual.Pmtrx[0][0] + .001 * actual.Pmtrx[1][0]
                               + (.001 * .001 * .001 / 3.0);
        const double gain = predicted / (predicted + .01 * .01);
        const double output = actual.FilteredResCal(input);
        const double old_output = older.FilteredResCaloldVersion(input);
        if (std::abs(gain) > max_gain) { max_gain = std::abs(gain); gain_iteration = n; }
        if (std::abs(output) > max_output) { max_output = std::abs(output); output_iteration = n; }
        old_max_output = std::max(old_max_output, std::abs(old_output));
        if (!first_negative_p00 && actual.Pmtrx[0][0] < 0) first_negative_p00 = n;
        stream << n << ',' << n / 300.0 << ',' << input << ',' << output << ',' << gain << ','
               << actual.Pmtrx[0][0] << ',' << actual.Pmtrx[1][1] << ','
               << actual.Pmtrx[0][1] - actual.Pmtrx[1][0] << ',' << old_output << '\n';
    }
    std::cout << std::setprecision(17)
              << "{\"max_abs_gain\":" << max_gain << ",\"gain_iteration\":" << gain_iteration
              << ",\"max_abs_output\":" << max_output << ",\"output_iteration\":" << output_iteration
              << ",\"old_version_max_abs_output\":" << old_max_output
              << ",\"first_negative_p00\":" << first_negative_p00 << "}\n";
}
