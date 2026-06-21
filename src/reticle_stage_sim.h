#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include "stage_dynamics.h"

namespace godot {

class ReticleStageSim : public Node {
    GDCLASS(ReticleStageSim, Node)

private:
    wafer_stage::StageDynamics dynamics_;
    bool running_;
    bool use_threaded_mode_;
    double time_scale_;
    int steps_per_frame_;
    double max_sim_time_;

    double mass;
    double lorentz_force_constant;
    double spring_stiffness;
    double passive_damping;
    double active_stiffness;
    double active_damping;
    double active_integral_gain;
    double target_position;
    double accel_g;
    double simulation_frequency;
    double settling_threshold_nm;
    double max_force;
    double derivative_filter_cutoff;
    double derivative_filter_sample_rate;

    bool noise_enabled;
    double noise_acoustic_amplitude_nm;
    double noise_acoustic_cutoff_hz;
    double noise_drift_amplitude_nm;
    double noise_drift_bandwidth_hz;
    int64_t noise_seed;

protected:
    static void _bind_methods();

public:
    ReticleStageSim();
    ~ReticleStageSim();

    void _ready() override;
    void _process(double delta) override;

    void start_simulation();
    void reset_simulation();
    void stop_simulation();

    double get_mass() const;
    void set_mass(double p_mass);

    double get_lorentz_force_constant() const;
    void set_lorentz_force_constant(double p_kf);

    double get_spring_stiffness() const;
    void set_spring_stiffness(double p_ks);

    double get_passive_damping() const;
    void set_passive_damping(double p_cd);

    double get_active_stiffness() const;
    void set_active_stiffness(double p_ka);

    double get_active_damping() const;
    void set_active_damping(double p_ca);

    double get_active_integral_gain() const;
    void set_active_integral_gain(double p_ki);

    double get_target_position() const;
    void set_target_position(double p_target);

    double get_accel_g() const;
    void set_accel_g(double p_g);

    double get_simulation_frequency() const;
    void set_simulation_frequency(double p_freq);

    double get_settling_threshold_nm() const;
    void set_settling_threshold_nm(double p_nm);

    double get_max_force() const;
    void set_max_force(double p_force);

    double get_derivative_filter_cutoff() const;
    void set_derivative_filter_cutoff(double p_hz);

    double get_derivative_filter_sample_rate() const;
    void set_derivative_filter_sample_rate(double p_hz);

    bool get_use_threaded_mode() const;
    void set_use_threaded_mode(bool p_enable);

    bool get_noise_enabled() const;
    void set_noise_enabled(bool p_enable);

    double get_noise_acoustic_amplitude_nm() const;
    void set_noise_acoustic_amplitude_nm(double p_amp);

    double get_noise_acoustic_cutoff_hz() const;
    void set_noise_acoustic_cutoff_hz(double p_hz);

    double get_noise_drift_amplitude_nm() const;
    void set_noise_drift_amplitude_nm(double p_amp);

    double get_noise_drift_bandwidth_hz() const;
    void set_noise_drift_bandwidth_hz(double p_hz);

    int64_t get_noise_seed() const;
    void set_noise_seed(int64_t p_seed);

    double get_current_noise_nm() const;

    double get_time_scale() const;
    void set_time_scale(double p_scale);

    double get_max_sim_time() const;
    void set_max_sim_time(double p_time);

    double get_position() const;
    double get_velocity() const;
    double get_position_error() const;
    double get_position_error_nm() const;
    double get_sim_time() const;
    int get_phase() const;
    double get_settling_time() const;
    bool is_settled() const;
    bool is_running() const;
    bool has_nan() const;
    uint32_t get_nan_recovery_count() const;
    uint64_t get_step_counter() const;

    PackedFloat64Array get_error_time_series() const;
    PackedFloat64Array get_position_time_series() const;
    PackedFloat64Array get_velocity_time_series() const;
    PackedFloat64Array get_time_series() const;
    PackedFloat64Array get_lorentz_force_series() const;
    PackedFloat64Array get_spring_force_series() const;
    PackedFloat64Array get_damping_force_series() const;

    int get_sample_count() const;
};

}
