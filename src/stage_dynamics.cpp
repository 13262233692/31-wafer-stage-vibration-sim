#include "stage_dynamics.h"
#include <algorithm>
#include <cmath>

namespace wafer_stage {

StageDynamics::StageDynamics()
    : sim_time_(0.0)
    , integral_error_(0.0)
    , prev_error_(0.0)
    , phase_(MotionPhase::IDLE)
    , phase_start_time_(0.0)
    , accel_duration_(0.0)
    , decel_start_time_(0.0)
    , settle_start_time_(0.0)
    , settling_time_(0.0)
    , settled_(false)
    , record_interval_(1e-5)
    , last_record_time_(0.0) {
    params_.mass = 50.0;
    params_.lorentz_force_constant = 50.0;
    params_.spring_stiffness = 0.0;
    params_.passive_damping = 2000.0;
    params_.active_stiffness = 1.5e7;
    params_.active_damping = 60000.0;
    params_.active_integral_gain = 5.0e6;
    params_.target_position = 0.01;
    params_.accel_command = 2.0 * GRAVITY;
    params_.simulation_dt = 1e-5;
    params_.settling_threshold_nm = 0.1;
    params_.max_force = 50000.0;
    state_ = StageState(0.0, 0.0);
}

void StageDynamics::initialize(const StageParams &params) {
    params_ = params;
    reset();
}

void StageDynamics::reset() {
    state_ = StageState(0.0, 0.0);
    sim_time_ = 0.0;
    integral_error_ = 0.0;
    prev_error_ = 0.0;
    phase_ = MotionPhase::IDLE;
    phase_start_time_ = 0.0;
    settled_ = false;
    settling_time_ = 0.0;
    waveform_.clear();
    last_record_time_ = 0.0;

    double distance = std::abs(params_.target_position);
    double accel = std::abs(params_.accel_command);
    if (distance > 0.0 && accel > 0.0) {
        accel_duration_ = std::sqrt(distance / accel);
    } else {
        accel_duration_ = 0.001;
    }
    decel_start_time_ = accel_duration_;
    settle_start_time_ = 2.0 * accel_duration_;

    phase_ = MotionPhase::ACCELERATE;
    phase_start_time_ = 0.0;
}

double StageDynamics::compute_lorentz_force(double current) const {
    return params_.lorentz_force_constant * current;
}

double StageDynamics::compute_spring_force(double position) const {
    return -params_.spring_stiffness * (position - params_.target_position);
}

double StageDynamics::compute_passive_damping_force(double velocity) const {
    return -params_.passive_damping * velocity;
}

double StageDynamics::compute_active_force(double error, double velocity, double integral_error) const {
    double f_stiffness = params_.active_stiffness * error;
    double f_damping = params_.active_damping * velocity;
    double f_integral = params_.active_integral_gain * integral_error;
    return f_stiffness + f_damping + f_integral;
}

double StageDynamics::compute_feedforward_accel() const {
    if (phase_ == MotionPhase::ACCELERATE) {
        return params_.accel_command;
    } else if (phase_ == MotionPhase::DECELERATE) {
        return -params_.accel_command;
    }
    return 0.0;
}

double StageDynamics::compute_reference_position(double t) const {
    double a = params_.accel_command;
    double t_a = accel_duration_;
    double t_d = decel_start_time_;

    if (t <= t_a) {
        return 0.5 * a * t * t;
    } else if (t <= 2.0 * t_a) {
        double v_peak = a * t_a;
        double x_mid = 0.5 * a * t_a * t_a;
        double dt = t - t_a;
        return x_mid + v_peak * dt - 0.5 * a * dt * dt;
    } else {
        return params_.target_position;
    }
}

double StageDynamics::compute_reference_velocity(double t) const {
    double a = params_.accel_command;
    double t_a = accel_duration_;

    if (t <= t_a) {
        return a * t;
    } else if (t <= 2.0 * t_a) {
        double v_peak = a * t_a;
        double dt = t - t_a;
        return v_peak - a * dt;
    } else {
        return 0.0;
    }
}

double StageDynamics::compute_tracking_error() const {
    if (phase_ == MotionPhase::ACCELERATE || phase_ == MotionPhase::DECELERATE) {
        double ref_pos = compute_reference_position(sim_time_);
        return ref_pos - state_.x;
    }
    return params_.target_position - state_.x;
}

void StageDynamics::update_phase() {
    double t = sim_time_;

    if (phase_ == MotionPhase::ACCELERATE && t >= decel_start_time_) {
        phase_ = MotionPhase::DECELERATE;
        phase_start_time_ = t;
    } else if (phase_ == MotionPhase::DECELERATE && t >= settle_start_time_) {
        phase_ = MotionPhase::SETTLE;
        phase_start_time_ = t;
        settle_start_time_ = t;
        integral_error_ = 0.0;
    } else if (phase_ == MotionPhase::SETTLE && !settled_) {
        double error_nm = std::abs(get_position_error_nm());
        if (error_nm < params_.settling_threshold_nm) {
            settling_time_ = t - settle_start_time_;
            settled_ = true;
            phase_ = MotionPhase::SETTLED;
        }
    }
}

void StageDynamics::record_sample(double force_lorentz, double force_spring, double force_damping, double current) {
    if (sim_time_ - last_record_time_ >= record_interval_ || waveform_.empty()) {
        WaveformSample sample;
        sample.time = sim_time_;
        sample.position = state_.x;
        sample.velocity = state_.v;
        sample.error = params_.target_position - state_.x;
        sample.reference_pos = compute_reference_position(sim_time_);
        sample.force_lorentz = force_lorentz;
        sample.force_spring = force_spring;
        sample.force_damping = force_damping;
        sample.current = current;
        waveform_.push_back(sample);
        last_record_time_ = sim_time_;
    }
}

double StageDynamics::get_position_error_nm() const {
    return (params_.target_position - state_.x) * 1e9;
}

void StageDynamics::step() {
    double dt = params_.simulation_dt;

    double tracking_error = compute_tracking_error();
    double ref_vel = compute_reference_velocity(sim_time_);
    double velocity_error = ref_vel - state_.v;

    if (phase_ == MotionPhase::SETTLE || phase_ == MotionPhase::SETTLED) {
        if (tracking_error * integral_error_ < 0.0) {
            integral_error_ *= 0.99;
        }
        integral_error_ += tracking_error * dt;
        integral_error_ = std::clamp(integral_error_, -1e-7, 1e-7);
    }

    double ff_accel = compute_feedforward_accel();
    double ff_current = (params_.mass * ff_accel) / params_.lorentz_force_constant;
    double lin_spring = (params_.spring_stiffness * (state_.x - params_.target_position)) / params_.lorentz_force_constant;
    double lin_damping = (params_.passive_damping * state_.v) / params_.lorentz_force_constant;
    double active_current = compute_active_force(tracking_error, velocity_error, integral_error_) / params_.lorentz_force_constant;
    double total_current = ff_current + lin_spring + lin_damping + active_current;

    double max_current = params_.max_force / params_.lorentz_force_constant;
    total_current = std::clamp(total_current, -max_current, max_current);

    double f_lorentz = compute_lorentz_force(total_current);
    double f_spring = compute_spring_force(state_.x);
    double f_damping = compute_passive_damping_force(state_.v);

    auto dynamics_func = [this](const StageState &s, double t) -> StageDerivative {
        double ref_p = compute_reference_position(t);
        double ref_v = compute_reference_velocity(t);
        double t_err = ref_p - s.x;
        double v_err = ref_v - s.v;

        double ff_a = 0.0;
        if (phase_ == MotionPhase::ACCELERATE) {
            ff_a = params_.accel_command;
        } else if (phase_ == MotionPhase::DECELERATE) {
            ff_a = -params_.accel_command;
        }

        double ff_c = (params_.mass * ff_a) / params_.lorentz_force_constant;
        double ls = (params_.spring_stiffness * (s.x - params_.target_position)) / params_.lorentz_force_constant;
        double ld = (params_.passive_damping * s.v) / params_.lorentz_force_constant;
        double af = compute_active_force(t_err, v_err, integral_error_);
        double ac = af / params_.lorentz_force_constant;
        double tc = ff_c + ls + ld + ac;

        double mc = params_.max_force / params_.lorentz_force_constant;
        tc = std::clamp(tc, -mc, mc);

        double f_lor = compute_lorentz_force(tc);
        double f_spr = compute_spring_force(s.x);
        double f_dmp = compute_passive_damping_force(s.v);

        double total_force = f_lor + f_spr + f_dmp;
        double accel = total_force / params_.mass;

        return StageDerivative(s.v, accel);
    };

    state_ = RK4Integrator::step(state_, sim_time_, dt, dynamics_func);
    sim_time_ += dt;

    update_phase();
    record_sample(f_lorentz, f_spring, f_damping, total_current);
}

}
