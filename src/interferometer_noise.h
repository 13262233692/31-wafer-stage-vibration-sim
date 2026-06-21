#pragma once

#include <array>
#include <cmath>
#include <random>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace wafer_stage {

class KolmogorovNoise {
private:
    static constexpr int NUM_SECTIONS = 6;

    struct PinkSection {
        double a1, a2;
        double b0, b1;
        double z1;
    };

    std::array<PinkSection, NUM_SECTIONS> sections_;
    std::mt19937_64 rng_;
    std::normal_distribution<double> gauss_;
    double sample_rate_;
    double cutoff_hz_;
    double amplitude_;

public:
    KolmogorovNoise()
        : rng_(std::random_device{}())
        , gauss_(0.0, 1.0)
        , sample_rate_(10000.0)
        , cutoff_hz_(100.0)
        , amplitude_(1.0) {
        design_filter(100.0, 10000.0);
        reset();
    }

    KolmogorovNoise(double cutoff_frequency_hz, double sample_rate_hz, double amplitude = 1.0, uint64_t seed = 0)
        : rng_(seed ? seed : std::random_device{}())
        , gauss_(0.0, 1.0)
        , sample_rate_(sample_rate_hz)
        , cutoff_hz_(cutoff_frequency_hz)
        , amplitude_(amplitude) {
        design_filter(cutoff_frequency_hz, sample_rate_hz);
        reset();
    }

    void set_seed(uint64_t seed) {
        rng_.seed(seed);
    }

    void set_amplitude(double amp) {
        amplitude_ = amp;
    }

    double get_amplitude() const {
        return amplitude_;
    }

    void set_sample_rate(double sr) {
        sample_rate_ = sr;
        design_filter(cutoff_hz_, sr);
    }

    void set_cutoff(double fc) {
        cutoff_hz_ = fc;
        design_filter(fc, sample_rate_);
    }

    void reset() {
        for (auto &s : sections_) {
            s.z1 = 0.0;
        }
    }

    double tick() {
        double white = gauss_(rng_);
        double pink = white;

        for (int i = 0; i < NUM_SECTIONS; i++) {
            auto &s = sections_[i];
            double v = pink - s.a1 * s.z1;
            pink = s.b0 * v + s.b1 * s.z1;
            s.z1 = v;
        }

        return pink * amplitude_;
    }

    double sample_rate() const { return sample_rate_; }

private:
    void design_filter(double fc, double fs) {
        double T = 1.0 / fs;
        double wc = 2.0 * M_PI * fc;

        double f0 = 0.0001;
        double f1 = 0.001;
        double f2 = 0.01;
        double f3 = 0.1;
        double f4 = 1.0;
        double f5 = 10.0;

        double pole_freqs[NUM_SECTIONS] = {f0, f1, f2, f3, f4, f5};

        double scale = 1.0;

        for (int i = 0; i < NUM_SECTIONS; i++) {
            double fp = pole_freqs[i] * fc;
            double wp = 2.0 * M_PI * fp;

            if (wp * T >= 2.0) continue;

            double alpha = 2.0 / T;
            double s_pole = -wp;
            double z_pole = (alpha + s_pole) / (alpha - s_pole);

            sections_[i].a1 = -z_pole;
            sections_[i].a2 = 0.0;
            sections_[i].b0 = 1.0;
            sections_[i].b1 = -1.0;
            sections_[i].z1 = 0.0;

            scale *= 2.0 / (alpha - s_pole);
        }

        double norm_scale = 1.0 / (scale * fc * 0.5);
        amplitude_ *= 1.0;
    }
};

class MultiAxisInterferometerNoise {
private:
    std::vector<KolmogorovNoise> axes_;
    double sample_rate_;
    double acoustic_cutoff_hz_;
    double refractive_drift_hz_;
    double acoustic_amplitude_nm_;
    double drift_amplitude_nm_;

    std::mt19937_64 drift_rng_;
    double drift_state_;
    double drift_rate_;

public:
    MultiAxisInterferometerNoise(int num_axes = 6)
        : sample_rate_(10000.0)
        , acoustic_cutoff_hz_(150.0)
        , refractive_drift_hz_(0.05)
        , acoustic_amplitude_nm_(0.05)
        , drift_amplitude_nm_(0.02)
        , drift_rng_(std::random_device{}())
        , drift_state_(0.0)
        , drift_rate_(0.0) {

        axes_.reserve(num_axes);
        for (int i = 0; i < num_axes; i++) {
            axes_.emplace_back(acoustic_cutoff_hz_, sample_rate_, 1.0, i * 7919ULL + 1234ULL);
        }
    }

    void set_sample_rate(double sr) {
        sample_rate_ = sr;
        for (auto &a : axes_) {
            a.set_sample_rate(sr);
        }
    }

    void set_acoustic_cutoff(double hz) {
        acoustic_cutoff_hz_ = hz;
        for (auto &a : axes_) {
            a.set_cutoff(hz);
        }
    }

    void set_acoustic_amplitude(double amp_nm) {
        acoustic_amplitude_nm_ = amp_nm;
    }

    void set_drift_amplitude(double amp_nm) {
        drift_amplitude_nm_ = amp_nm;
    }

    void set_drift_bandwidth(double hz) {
        refractive_drift_hz_ = hz;
    }

    void set_seed(uint64_t seed) {
        for (size_t i = 0; i < axes_.size(); i++) {
            axes_[i].set_seed(seed + i * 7919ULL);
        }
        drift_rng_.seed(seed + 0xDEADBEEFULL);
    }

    void reset() {
        for (auto &a : axes_) {
            a.reset();
        }
        drift_state_ = 0.0;
        drift_rate_ = 0.0;
    }

    size_t num_axes() const { return axes_.size(); }

    double get_noise(int axis) {
        if (axis < 0 || axis >= static_cast<int>(axes_.size())) return 0.0;

        double acoustic = axes_[axis].tick() * acoustic_amplitude_nm_;

        update_drift();
        double per_axis_drift = drift_state_ * drift_amplitude_nm_;
        per_axis_drift *= (1.0 + 0.3 * std::sin(axis * 1.7 + drift_state_ * 10.0));

        return (acoustic + per_axis_drift) * 1e-9;
    }

    void get_all_noises(std::vector<double> &out) {
        update_drift();
        out.resize(axes_.size());
        for (size_t i = 0; i < axes_.size(); i++) {
            double acoustic = axes_[i].tick() * acoustic_amplitude_nm_;
            double per_axis_drift = drift_state_ * drift_amplitude_nm_;
            per_axis_drift *= (1.0 + 0.3 * std::sin(i * 1.7 + drift_state_ * 10.0));
            out[i] = (acoustic + per_axis_drift) * 1e-9;
        }
    }

    double get_drift_state() const { return drift_state_ * drift_amplitude_nm_; }
    double get_acoustic_amplitude() const { return acoustic_amplitude_nm_; }
    double get_drift_amplitude() const { return drift_amplitude_nm_; }

private:
    void update_drift() {
        double dt = 1.0 / sample_rate_;
        std::normal_distribution<double> nd(0.0, 1.0);

        double damping = 2.0 * M_PI * refractive_drift_hz_;
        double noise_strength = std::sqrt(2.0 * damping) * drift_amplitude_nm_;

        double accel = -damping * drift_rate_ + noise_strength * nd(drift_rng_) / std::sqrt(dt);
        drift_rate_ += accel * dt;
        drift_state_ += drift_rate_ * dt;

        double max_drift = 3.0;
        if (std::abs(drift_state_) > max_drift) {
            drift_state_ = std::copysign(max_drift, drift_state_);
            drift_rate_ = -drift_rate_ * 0.5;
        }
    }
};

}
