#pragma once

#include "rk4_integrator.h"
#include "elliptic_filter.h"
#include "interferometer_noise.h"
#include <cstdint>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <memory>

namespace wafer_stage {

enum class MotionPhase : int {
    IDLE = 0,
    ACCELERATE = 1,
    DECELERATE = 2,
    SETTLE = 3,
    SETTLED = 4
};

struct WaveformSample {
    double time;
    double position;
    double velocity;
    double error;
    double reference_pos;
    double force_lorentz;
    double force_spring;
    double force_damping;
    double current;
};

struct StageParams {
    double mass;
    double lorentz_force_constant;
    double spring_stiffness;
    double passive_damping;
    double active_stiffness;
    double active_damping;
    double active_integral_gain;
    double target_position;
    double accel_command;
    double simulation_dt;
    double settling_threshold_nm;
    double max_force;
    double derivative_filter_cutoff;
    double derivative_filter_sample_rate;

    double noise_acoustic_amplitude_nm;
    double noise_acoustic_cutoff_hz;
    double noise_drift_amplitude_nm;
    double noise_drift_bandwidth_hz;
    bool noise_enabled;
    uint64_t noise_seed;
};

class StageDynamics {
public:
    static constexpr double GRAVITY = 9.80665;
    static constexpr double THREAD_DT = 1.0e-4;

private:
    StageParams params_;
    StageState state_;
    StageState state_snapshot_;
    double sim_time_;
    double sim_time_snapshot_;
    double integral_error_;
    double prev_error_;
    double filtered_derivative_;
    MotionPhase phase_;
    MotionPhase phase_snapshot_;
    double phase_start_time_;
    double accel_duration_;
    double decel_start_time_;
    double settle_start_time_;
    double settling_time_;
    double settling_time_snapshot_;
    bool settled_;
    bool settled_snapshot_;

    std::unique_ptr<EllipticLowPass4> derivative_filter_;
    std::unique_ptr<EllipticLowPass4> velocity_filter_;

    std::unique_ptr<MultiAxisInterferometerNoise> interferometer_noise_;
    double current_noise_m_;
    double current_noise_nm_snapshot_;

    std::vector<WaveformSample> waveform_;
    std::vector<WaveformSample> waveform_snapshot_;
    double record_interval_;
    double last_record_time_;

    std::atomic<bool> thread_running_;
    std::atomic<bool> sim_active_;
    std::thread sim_thread_;
    mutable std::mutex state_mutex_;
    std::condition_variable sim_cv_;
    std::mutex sim_start_mutex_;

    std::atomic<uint64_t> step_counter_;
    std::atomic<bool> nan_detected_;
    std::atomic<uint32_t> nan_recovery_count_;
    double stable_since_;

    double compute_lorentz_force(double current) const;
    double compute_spring_force(double position) const;
    double compute_passive_damping_force(double velocity) const;
    double compute_active_force(double error, double filtered_deriv, double integral_error) const;
    double compute_feedforward_accel(double t_local) const;
    double compute_reference_position(double t) const;
    double compute_reference_velocity(double t) const;
    double compute_tracking_error(double t) const;

    void update_phase(double t);
    void record_sample_internal(double force_lorentz, double force_spring,
                                double force_damping, double current);

    void snapshot_state_locked();
    void orthonormalize_coupling();
    bool check_and_recover_nan();

    void thread_loop();
    void step_internal(double dt);

public:
    StageDynamics();
    ~StageDynamics();

    void initialize(const StageParams &params);
    void reset();

    void start_threaded_simulation();
    void stop_threaded_simulation();
    bool is_thread_running() const { return thread_running_.load(); }

    StageDerivative dynamics(const StageState &s, double t) const;

    void step();

    StageState get_state() const;
    double get_sim_time() const;
    MotionPhase get_phase() const;
    double get_position() const;
    double get_velocity() const;
    double get_position_error() const;
    double get_position_error_nm() const;
    double get_settling_time() const;
    bool is_settled() const;
    const StageParams &get_params() const { return params_; }

    std::vector<WaveformSample> get_waveform_copy() const;
    size_t get_waveform_size() const;
    void clear_waveform();

    void set_target_position(double target);
    void set_accel_command(double accel);

    uint64_t get_step_counter() const { return step_counter_.load(); }
    bool has_nan() const { return nan_detected_.load(); }
    uint32_t get_nan_recovery_count() const { return nan_recovery_count_.load(); }

    double get_current_noise_nm() const;
    bool is_noise_enabled() const;
    void set_noise_enabled(bool enabled);
    void set_noise_acoustic_amplitude_nm(double amp);
    void set_noise_drift_amplitude_nm(double amp);
    void set_noise_seed(uint64_t seed);
};

}
