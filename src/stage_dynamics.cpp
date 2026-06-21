#include "stage_dynamics.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstring>

namespace wafer_stage {

StageDynamics::StageDynamics()
    : sim_time_(0.0)
    , sim_time_snapshot_(0.0)
    , integral_error_(0.0)
    , prev_error_(0.0)
    , filtered_derivative_(0.0)
    , phase_(MotionPhase::IDLE)
    , phase_snapshot_(MotionPhase::IDLE)
    , phase_start_time_(0.0)
    , accel_duration_(0.0)
    , decel_start_time_(0.0)
    , settle_start_time_(0.0)
    , settling_time_(0.0)
    , settling_time_snapshot_(0.0)
    , settled_(false)
    , settled_snapshot_(false)
    , record_interval_(1e-5)
    , last_record_time_(0.0)
    , thread_running_(false)
    , sim_active_(false)
    , step_counter_(0)
    , nan_detected_(false)
    , nan_recovery_count_(0)
    , stable_since_(-1.0)
    , current_noise_m_(0.0)
    , current_noise_nm_snapshot_(0.0) {

    params_.mass = 50.0;
    params_.lorentz_force_constant = 50.0;
    params_.spring_stiffness = 0.0;
    params_.passive_damping = 2000.0;
    params_.active_stiffness = 1.5e7;
    params_.active_damping = 60000.0;
    params_.active_integral_gain = 5.0e6;
    params_.target_position = 0.01;
    params_.accel_command = 2.0 * GRAVITY;
    params_.simulation_dt = THREAD_DT;
    params_.settling_threshold_nm = 0.1;
    params_.max_force = 50000.0;
    params_.derivative_filter_cutoff = 1500.0;
    params_.derivative_filter_sample_rate = 1.0 / THREAD_DT;

    params_.noise_acoustic_amplitude_nm = 0.05;
    params_.noise_acoustic_cutoff_hz = 150.0;
    params_.noise_drift_amplitude_nm = 0.02;
    params_.noise_drift_bandwidth_hz = 0.05;
    params_.noise_enabled = false;
    params_.noise_seed = 0;

    state_ = StageState(0.0, 0.0);
    state_snapshot_ = state_;

    derivative_filter_ = std::make_unique<EllipticLowPass4>(
        params_.derivative_filter_cutoff,
        params_.derivative_filter_sample_rate
    );
    velocity_filter_ = std::make_unique<EllipticLowPass4>(
        500.0,
        params_.derivative_filter_sample_rate
    );

    interferometer_noise_ = std::make_unique<MultiAxisInterferometerNoise>(6);
    interferometer_noise_->set_sample_rate(1.0 / params_.simulation_dt);
    interferometer_noise_->set_acoustic_cutoff(params_.noise_acoustic_cutoff_hz);
    interferometer_noise_->set_acoustic_amplitude(params_.noise_acoustic_amplitude_nm);
    interferometer_noise_->set_drift_amplitude(params_.noise_drift_amplitude_nm);
    interferometer_noise_->set_drift_bandwidth(params_.noise_drift_bandwidth_hz);
    if (params_.noise_seed != 0) {
        interferometer_noise_->set_seed(params_.noise_seed);
    }
}

StageDynamics::~StageDynamics() {
    stop_threaded_simulation();
}

void StageDynamics::initialize(const StageParams &params) {
    stop_threaded_simulation();
    params_ = params;
    if (params_.simulation_dt <= 0.0) {
        params_.simulation_dt = THREAD_DT;
    }
    if (params_.derivative_filter_sample_rate <= 0.0) {
        params_.derivative_filter_sample_rate = 1.0 / params_.simulation_dt;
    }
    if (params_.derivative_filter_cutoff <= 0.0) {
        params_.derivative_filter_cutoff = 1500.0;
    }
    derivative_filter_ = std::make_unique<EllipticLowPass4>(
        params_.derivative_filter_cutoff,
        params_.derivative_filter_sample_rate
    );
    velocity_filter_ = std::make_unique<EllipticLowPass4>(
        500.0,
        params_.derivative_filter_sample_rate
    );

    interferometer_noise_ = std::make_unique<MultiAxisInterferometerNoise>(6);
    interferometer_noise_->set_sample_rate(1.0 / params_.simulation_dt);
    interferometer_noise_->set_acoustic_cutoff(params_.noise_acoustic_cutoff_hz);
    interferometer_noise_->set_acoustic_amplitude(params_.noise_acoustic_amplitude_nm);
    interferometer_noise_->set_drift_amplitude(params_.noise_drift_amplitude_nm);
    interferometer_noise_->set_drift_bandwidth(params_.noise_drift_bandwidth_hz);
    if (params_.noise_seed != 0) {
        interferometer_noise_->set_seed(params_.noise_seed);
    }

    reset();
}

void StageDynamics::reset() {
    state_ = StageState(0.0, 0.0);
    state_snapshot_ = state_;
    sim_time_ = 0.0;
    sim_time_snapshot_ = 0.0;
    integral_error_ = 0.0;
    prev_error_ = 0.0;
    filtered_derivative_ = 0.0;
    phase_ = MotionPhase::IDLE;
    phase_snapshot_ = MotionPhase::IDLE;
    settled_ = false;
    settled_snapshot_ = false;
    settling_time_ = 0.0;
    settling_time_snapshot_ = 0.0;
    waveform_.clear();
    waveform_snapshot_.clear();
    last_record_time_ = 0.0;
    step_counter_.store(0);
    nan_detected_.store(false);
    nan_recovery_count_.store(0);
    stable_since_ = -1.0;

    if (derivative_filter_) derivative_filter_->reset();
    if (velocity_filter_) velocity_filter_->reset();
    if (interferometer_noise_) interferometer_noise_->reset();
    current_noise_m_ = 0.0;
    current_noise_nm_snapshot_ = 0.0;

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

    snapshot_state_locked();
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

double StageDynamics::compute_active_force(double error, double filtered_deriv, double integral_error) const {
    double f_stiffness = params_.active_stiffness * error;
    double f_damping = params_.active_damping * filtered_deriv;
    double f_integral = params_.active_integral_gain * integral_error;

    double max_f = params_.max_force;
    double total = f_stiffness + f_damping + f_integral;
    if (std::abs(total) > max_f) {
        double scale = max_f / std::abs(total);
        f_stiffness *= scale;
        f_damping *= scale;
        f_integral *= scale;
    }
    return f_stiffness + f_damping + f_integral;
}

double StageDynamics::compute_feedforward_accel(double t_local) const {
    if (t_local < decel_start_time_) {
        return params_.accel_command;
    } else if (t_local < settle_start_time_) {
        return -params_.accel_command;
    }
    return 0.0;
}

double StageDynamics::compute_reference_position(double t) const {
    double a = params_.accel_command;
    double t_a = accel_duration_;

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
        return a * t_a - a * (t - t_a);
    }
    return 0.0;
}

double StageDynamics::compute_tracking_error(double t) const {
    if (phase_ == MotionPhase::ACCELERATE || phase_ == MotionPhase::DECELERATE) {
        return compute_reference_position(t) - state_.x;
    }
    return params_.target_position - state_.x;
}

void StageDynamics::update_phase(double t) {
    if (phase_ == MotionPhase::ACCELERATE && t >= decel_start_time_) {
        phase_ = MotionPhase::DECELERATE;
        phase_start_time_ = t;
    } else if (phase_ == MotionPhase::DECELERATE && t >= settle_start_time_) {
        phase_ = MotionPhase::SETTLE;
        phase_start_time_ = t;
        settle_start_time_ = t;
        integral_error_ = 0.0;
        if (derivative_filter_) derivative_filter_->reset();
    } else if (phase_ == MotionPhase::SETTLE && !settled_) {
        double error_nm = std::abs((params_.target_position - state_.x) * 1e9);
        if (error_nm < params_.settling_threshold_nm) {
            if (stable_since_ < 0.0) {
                stable_since_ = t;
            } else if (t - stable_since_ > 0.005) {
                settling_time_ = t - settle_start_time_;
                settled_ = true;
                phase_ = MotionPhase::SETTLED;
                stable_since_ = -1.0;
            }
        } else {
            stable_since_ = -1.0;
        }
    }
}

void StageDynamics::record_sample_internal(double force_lorentz, double force_spring,
                                            double force_damping, double current) {
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
        if (waveform_.size() > 500000) {
            waveform_.erase(waveform_.begin(), waveform_.begin() + 100000);
        }
        last_record_time_ = sim_time_;
    }
}

void StageDynamics::snapshot_state_locked() {
    state_snapshot_ = state_;
    sim_time_snapshot_ = sim_time_;
    phase_snapshot_ = phase_;
    settling_time_snapshot_ = settling_time_;
    settled_snapshot_ = settled_;
    waveform_snapshot_ = waveform_;
    current_noise_nm_snapshot_ = current_noise_m_ * 1e9;
}

void StageDynamics::orthonormalize_coupling() {
    double k = params_.active_stiffness;
    double c = params_.active_damping;
    double m = params_.mass;

    if (m <= 0.0) m = 1.0;

    double omega_n = std::sqrt(k / m);
    if (omega_n <= 0.0) omega_n = 1.0;
    double zeta = c / (2.0 * std::sqrt(k * m));

    static constexpr double ZETA_MIN = 0.4;
    static constexpr double ZETA_MAX = 2.0;
    static constexpr double OMEGA_MAX = 2000.0;

    if (omega_n > OMEGA_MAX) {
        double scale = (OMEGA_MAX * OMEGA_MAX) * m / k;
        params_.active_stiffness = k * scale;
        params_.active_damping = c * std::sqrt(scale);
    }
    if (zeta < ZETA_MIN) {
        params_.active_damping = 2.0 * ZETA_MIN * std::sqrt(params_.active_stiffness * m);
    } else if (zeta > ZETA_MAX) {
        params_.active_damping = 2.0 * ZETA_MAX * std::sqrt(params_.active_stiffness * m);
    }
}

bool StageDynamics::check_and_recover_nan() {
    bool is_nan = std::isnan(state_.x) || std::isnan(state_.v) ||
                  std::isinf(state_.x) || std::isinf(state_.v) ||
                  std::isnan(integral_error_) || std::isinf(integral_error_) ||
                  std::isnan(filtered_derivative_) || std::isinf(filtered_derivative_);

    if (!is_nan) {
        double abs_x = std::abs(state_.x);
        double abs_v = std::abs(state_.v);
        double safety_limit = 100.0 * std::max(1e-3, std::abs(params_.target_position));
        if (abs_x > safety_limit || abs_v > 100.0) {
            is_nan = true;
        }
    }

    if (is_nan) {
        nan_detected_.store(true);
        nan_recovery_count_.fetch_add(1);

        state_.x = compute_reference_position(sim_time_);
        state_.v = compute_reference_velocity(sim_time_);
        integral_error_ = 0.0;
        prev_error_ = 0.0;
        filtered_derivative_ = 0.0;
        if (derivative_filter_) derivative_filter_->reset();
        if (velocity_filter_) velocity_filter_->reset();

        double err = params_.target_position - state_.x;
        if (std::abs(err) * 1e9 < params_.settling_threshold_nm) {
            phase_ = MotionPhase::SETTLED;
            settled_ = true;
        } else if (sim_time_ < decel_start_time_) {
            phase_ = MotionPhase::ACCELERATE;
        } else if (sim_time_ < settle_start_time_) {
            phase_ = MotionPhase::DECELERATE;
        } else {
            phase_ = MotionPhase::SETTLE;
        }
        return true;
    }
    return false;
}

void StageDynamics::step_internal(double dt) {
    if (params_.noise_enabled && interferometer_noise_) {
        current_noise_m_ = interferometer_noise_->get_noise(0);
    } else {
        current_noise_m_ = 0.0;
    }

    double step_noise = current_noise_m_;
    double ref_pos = compute_reference_position(sim_time_);
    double ref_vel = compute_reference_velocity(sim_time_);

    double sensed_pos0 = state_.x + step_noise;
    double tracking_error_0;
    if (phase_ == MotionPhase::ACCELERATE || phase_ == MotionPhase::DECELERATE) {
        tracking_error_0 = ref_pos - sensed_pos0;
    } else {
        tracking_error_0 = params_.target_position - sensed_pos0;
    }

    double raw_deriv_0 = (tracking_error_0 - prev_error_) / dt;
    prev_error_ = tracking_error_0;

    if (derivative_filter_) {
        filtered_derivative_ = derivative_filter_->process(raw_deriv_0);
    } else {
        filtered_derivative_ = raw_deriv_0;
    }

    double vel_err_0 = ref_vel - state_.v;
    double combined_deriv_0 = 0.7 * filtered_derivative_ + 0.3 * vel_err_0;

    if (phase_ == MotionPhase::SETTLE || phase_ == MotionPhase::SETTLED) {
        if (tracking_error_0 * integral_error_ < 0.0) {
            integral_error_ *= 0.9;
        }
        integral_error_ += tracking_error_0 * dt;
        integral_error_ = std::clamp(integral_error_, -1e-8, 1e-8);
    } else {
        double decay = std::exp(-dt * 100.0);
        integral_error_ *= decay;
    }

    auto dynamics_func = [this, step_noise]
                         (const StageState &s, double t) -> StageDerivative {
        double ref_p = compute_reference_position(t);
        double ref_v_local = compute_reference_velocity(t);

        double sensed_x = s.x + step_noise;

        double t_err;
        MotionPhase ph = phase_;
        if (ph == MotionPhase::ACCELERATE || ph == MotionPhase::DECELERATE) {
            t_err = ref_p - sensed_x;
        } else {
            t_err = params_.target_position - sensed_x;
        }

        double v_err = ref_v_local - s.v;

        double ff_a = 0.0;
        if (t < decel_start_time_) {
            ff_a = params_.accel_command;
        } else if (t < settle_start_time_) {
            ff_a = -params_.accel_command;
        }

        double ff_c = (params_.mass * ff_a) / params_.lorentz_force_constant;
        double ls = (params_.spring_stiffness * (sensed_x - params_.target_position)) / params_.lorentz_force_constant;
        double ld = (params_.passive_damping * s.v) / params_.lorentz_force_constant;

        double deriv_est = v_err;
        double af = compute_active_force(t_err, deriv_est, integral_error_);
        double ac = af / params_.lorentz_force_constant;
        double tc = ff_c + ls + ld + ac;

        double mc = params_.max_force / params_.lorentz_force_constant;
        tc = std::clamp(tc, -mc, mc);

        double f_lor = compute_lorentz_force(tc);
        double f_spr = compute_spring_force(s.x);
        double f_dmp = compute_passive_damping_force(s.v);

        double total_force = f_lor + f_spr + f_dmp;
        total_force = std::clamp(total_force, -params_.max_force, params_.max_force);
        double accel = total_force / params_.mass;
        accel = std::clamp(accel, -1000.0, 1000.0);

        return StageDerivative(s.v, accel);
    };

    state_ = RK4Integrator::step(state_, sim_time_, dt, dynamics_func);

    sim_time_ += dt;

    check_and_recover_nan();
    orthonormalize_coupling();
    update_phase(sim_time_);

    double f_lorentz = compute_lorentz_force(
        (params_.mass * compute_feedforward_accel(sim_time_)) / params_.lorentz_force_constant
        + (params_.spring_stiffness * (state_.x + step_noise - params_.target_position)) / params_.lorentz_force_constant
        + (params_.passive_damping * state_.v) / params_.lorentz_force_constant
        + compute_active_force(tracking_error_0, combined_deriv_0, integral_error_) / params_.lorentz_force_constant
    );
    f_lorentz = std::clamp(f_lorentz, -params_.max_force, params_.max_force);

    record_sample_internal(f_lorentz, compute_spring_force(state_.x),
                          compute_passive_damping_force(state_.v),
                          f_lorentz / params_.lorentz_force_constant);

    step_counter_.fetch_add(1);
}

void StageDynamics::thread_loop() {
    using namespace std::chrono;
    auto next_tick = high_resolution_clock::now();
    const auto tick_period = duration<double>(THREAD_DT);

    while (thread_running_.load()) {
        {
            std::unique_lock<std::mutex> lk(sim_start_mutex_);
            sim_cv_.wait(lk, [this]() {
                return sim_active_.load() || !thread_running_.load();
            });
        }

        if (!thread_running_.load()) break;
        if (!sim_active_.load()) continue;

        double target_dt = params_.simulation_dt > 0 ? params_.simulation_dt : THREAD_DT;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            step_internal(target_dt);
            if (step_counter_.load() % 10 == 0) {
                snapshot_state_locked();
            }
        }

        next_tick += duration_cast<high_resolution_clock::duration>(
            duration<double>(target_dt)
        );
        auto now = high_resolution_clock::now();
        if (now < next_tick) {
            std::this_thread::sleep_until(next_tick);
        } else {
            next_tick = now;
        }
    }
}

void StageDynamics::step() {
    double dt = params_.simulation_dt > 0 ? params_.simulation_dt : THREAD_DT;
    std::lock_guard<std::mutex> lock(state_mutex_);
    step_internal(dt);
    snapshot_state_locked();
}

void StageDynamics::start_threaded_simulation() {
    if (thread_running_.load()) {
        sim_active_.store(true);
        sim_cv_.notify_one();
        return;
    }
    thread_running_.store(true);
    sim_active_.store(true);
    sim_thread_ = std::thread(&StageDynamics::thread_loop, this);
}

void StageDynamics::stop_threaded_simulation() {
    sim_active_.store(false);
    thread_running_.store(false);
    sim_cv_.notify_all();
    if (sim_thread_.joinable()) {
        sim_thread_.join();
    }
}

StageState StageDynamics::get_state() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_snapshot_;
}

double StageDynamics::get_sim_time() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return sim_time_snapshot_;
}

MotionPhase StageDynamics::get_phase() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return phase_snapshot_;
}

double StageDynamics::get_position() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_snapshot_.x;
}

double StageDynamics::get_velocity() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_snapshot_.v;
}

double StageDynamics::get_position_error() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return params_.target_position - state_snapshot_.x;
}

double StageDynamics::get_position_error_nm() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return (params_.target_position - state_snapshot_.x) * 1e9;
}

double StageDynamics::get_settling_time() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return settling_time_snapshot_;
}

bool StageDynamics::is_settled() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return settled_snapshot_;
}

std::vector<WaveformSample> StageDynamics::get_waveform_copy() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return waveform_snapshot_;
}

size_t StageDynamics::get_waveform_size() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return waveform_snapshot_.size();
}

void StageDynamics::clear_waveform() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    waveform_.clear();
    waveform_snapshot_.clear();
}

void StageDynamics::set_target_position(double target) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    params_.target_position = target;

    double distance = std::abs(target);
    double accel = std::abs(params_.accel_command);
    if (distance > 0.0 && accel > 0.0) {
        accel_duration_ = std::sqrt(distance / accel);
    } else {
        accel_duration_ = 0.001;
    }
    decel_start_time_ = sim_time_ + accel_duration_;
    settle_start_time_ = sim_time_ + 2.0 * accel_duration_;

    phase_ = MotionPhase::ACCELERATE;
    phase_start_time_ = sim_time_;
    settled_ = false;
    integral_error_ = 0.0;
    if (derivative_filter_) derivative_filter_->reset();
    if (velocity_filter_) velocity_filter_->reset();

    snapshot_state_locked();
}

void StageDynamics::set_accel_command(double accel) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    params_.accel_command = accel;

    double distance = std::abs(params_.target_position);
    double a = std::abs(accel);
    if (distance > 0.0 && a > 0.0) {
        accel_duration_ = std::sqrt(distance / a);
    }
    snapshot_state_locked();
}

double StageDynamics::get_current_noise_nm() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_noise_nm_snapshot_;
}

bool StageDynamics::is_noise_enabled() const {
    return params_.noise_enabled;
}

void StageDynamics::set_noise_enabled(bool enabled) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    params_.noise_enabled = enabled;
}

void StageDynamics::set_noise_acoustic_amplitude_nm(double amp) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    params_.noise_acoustic_amplitude_nm = amp;
    if (interferometer_noise_) {
        interferometer_noise_->set_acoustic_amplitude(amp);
    }
}

void StageDynamics::set_noise_drift_amplitude_nm(double amp) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    params_.noise_drift_amplitude_nm = amp;
    if (interferometer_noise_) {
        interferometer_noise_->set_drift_amplitude(amp);
    }
}

void StageDynamics::set_noise_seed(uint64_t seed) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    params_.noise_seed = seed;
    if (interferometer_noise_ && seed != 0) {
        interferometer_noise_->set_seed(seed);
    }
}

}
