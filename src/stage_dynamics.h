#pragma once

#include "rk4_integrator.h"
#include <cstdint>
#include <vector>

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
};

class StageDynamics {
public:
    static constexpr double GRAVITY = 9.80665;

private:
    StageParams params_;
    StageState state_;
    double sim_time_;
    double integral_error_;
    double prev_error_;
    MotionPhase phase_;
    double phase_start_time_;
    double accel_duration_;
    double decel_start_time_;
    double settle_start_time_;
    double settling_time_;
    bool settled_;

    std::vector<WaveformSample> waveform_;
    double record_interval_;
    double last_record_time_;

    double compute_lorentz_force(double current) const;
    double compute_spring_force(double position) const;
    double compute_passive_damping_force(double velocity) const;
    double compute_active_force(double error, double velocity, double integral_error) const;
    double compute_feedforward_accel() const;
    double compute_reference_position(double t) const;
    double compute_reference_velocity(double t) const;
    double compute_tracking_error() const;

    void update_phase();
    void record_sample(double force_lorentz, double force_spring, double force_damping, double current);

public:
    StageDynamics();

    void initialize(const StageParams &params);
    void reset();

    StageDerivative dynamics(const StageState &s, double t) const;

    void step();

    const StageState &get_state() const { return state_; }
    double get_sim_time() const { return sim_time_; }
    MotionPhase get_phase() const { return phase_; }
    double get_position() const { return state_.x; }
    double get_velocity() const { return state_.v; }
    double get_position_error() const { return params_.target_position - state_.x; }
    double get_position_error_nm() const;
    double get_settling_time() const { return settling_time_; }
    bool is_settled() const { return settled_; }
    const StageParams &get_params() const { return params_; }

    const std::vector<WaveformSample> &get_waveform() const { return waveform_; }
    void clear_waveform() { waveform_.clear(); }

    void set_target_position(double target) { params_.target_position = target; }
    void set_accel_command(double accel) { params_.accel_command = accel; }
};

}
