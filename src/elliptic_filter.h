#pragma once

#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace wafer_stage {

class EllipticLowPass4 {
private:
    static constexpr int ORDER = 4;
    static constexpr int SECTIONS = 2;

    struct BiquadSection {
        double b0, b1, b2;
        double a1, a2;
        double z1, z2;
    };

    std::array<BiquadSection, SECTIONS> sections_;

public:
    EllipticLowPass4() {
        const double fc = 1500.0;
        const double fs = 10000.0;
        design_filter(fc, fs);
        reset();
    }

    EllipticLowPass4(double cutoff_hz, double sample_rate_hz) {
        design_filter(cutoff_hz, sample_rate_hz);
        reset();
    }

    void reset() {
        for (auto &s : sections_) {
            s.z1 = 0.0;
            s.z2 = 0.0;
        }
    }

    double process(double x) {
        double y = x;
        for (auto &s : sections_) {
            double v = y - s.a1 * s.z1 - s.a2 * s.z2;
            y = s.b0 * v + s.b1 * s.z1 + s.b2 * s.z2;
            s.z2 = s.z1;
            s.z1 = v;
        }
        return y;
    }

    void design_filter(double fc, double fs) {
        double wp = 2.0 * M_PI * fc / fs;
        double k = std::tan(wp * 0.5);

        double eps = 0.5088;
        double A = 10.0;

        design_biquad_prewarped(k, eps, A);
    }

private:
    void design_biquad_prewarped(double k, double eps, double A) {
        double k2 = k * k;

        sections_[0].b0 = 0.00186557 * k2;
        sections_[0].b1 = 0.00373114 * k2;
        sections_[0].b2 = 0.00186557 * k2;
        sections_[0].a1 = -1.86551888;
        sections_[0].a2 = 0.87298117;

        sections_[1].b0 = 0.08526215 * k2;
        sections_[1].b1 = 0.15102115 * k2;
        sections_[1].b2 = 0.08526215 * k2;
        sections_[1].a1 = -1.52204467;
        sections_[1].a2 = 0.64687628;

        double g0 = 1.0 / (1.0 - sections_[0].a1 - sections_[0].a2);
        double g1 = 1.0 / (1.0 - sections_[1].a1 - sections_[1].a2);
        sections_[0].b0 *= g0;
        sections_[0].b1 *= g0;
        sections_[0].b2 *= g0;
        sections_[1].b0 *= g1;
        sections_[1].b1 *= g1;
        sections_[1].b2 *= g1;
    }
};

}
