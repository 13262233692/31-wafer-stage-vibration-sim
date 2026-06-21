#include "reticle_stage_sim.h"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/memory.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

ReticleStageSim::ReticleStageSim()
    : running_(false)
    , use_threaded_mode_(true)
    , time_scale_(1.0)
    , steps_per_frame_(100)
    , max_sim_time_(10.0)
    , mass(50.0)
    , lorentz_force_constant(50.0)
    , spring_stiffness(0.0)
    , passive_damping(2000.0)
    , active_stiffness(1.5e7)
    , active_damping(60000.0)
    , active_integral_gain(5.0e6)
    , target_position(0.01)
    , accel_g(2.0)
    , simulation_frequency(10000.0)
    , settling_threshold_nm(0.1)
    , max_force(50000.0)
    , derivative_filter_cutoff(1500.0)
    , derivative_filter_sample_rate(10000.0)
    , noise_enabled(true)
    , noise_acoustic_amplitude_nm(0.05)
    , noise_acoustic_cutoff_hz(150.0)
    , noise_drift_amplitude_nm(0.02)
    , noise_drift_bandwidth_hz(0.05)
    , noise_seed(42) {}

ReticleStageSim::~ReticleStageSim() {
    stop_simulation();
}

void ReticleStageSim::_ready() {
}

void ReticleStageSim::_process(double delta) {
    if (!running_) return;

    if (dynamics_.get_sim_time() >= max_sim_time_) {
        stop_simulation();
        return;
    }

    if (use_threaded_mode_) {
        return;
    }

    int n_steps = static_cast<int>(delta * simulation_frequency * time_scale_);
    n_steps = std::max(1, std::min(n_steps, 2000));
    for (int i = 0; i < n_steps; i++) {
        dynamics_.step();
        if (dynamics_.is_settled() || dynamics_.get_sim_time() >= max_sim_time_) {
            break;
        }
    }
}

void ReticleStageSim::start_simulation() {
    wafer_stage::StageParams params;
    params.mass = mass;
    params.lorentz_force_constant = lorentz_force_constant;
    params.spring_stiffness = spring_stiffness;
    params.passive_damping = passive_damping;
    params.active_stiffness = active_stiffness;
    params.active_damping = active_damping;
    params.active_integral_gain = active_integral_gain;
    params.target_position = target_position;
    params.accel_command = accel_g * wafer_stage::StageDynamics::GRAVITY;
    params.simulation_dt = 1.0 / simulation_frequency;
    params.settling_threshold_nm = settling_threshold_nm;
    params.max_force = max_force;
    params.derivative_filter_cutoff = derivative_filter_cutoff;
    params.derivative_filter_sample_rate = derivative_filter_sample_rate;
    params.noise_enabled = noise_enabled;
    params.noise_acoustic_amplitude_nm = noise_acoustic_amplitude_nm;
    params.noise_acoustic_cutoff_hz = noise_acoustic_cutoff_hz;
    params.noise_drift_amplitude_nm = noise_drift_amplitude_nm;
    params.noise_drift_bandwidth_hz = noise_drift_bandwidth_hz;
    params.noise_seed = static_cast<uint64_t>(noise_seed);

    dynamics_.initialize(params);
    running_ = true;

    if (use_threaded_mode_) {
        dynamics_.start_threaded_simulation();
    }
}

void ReticleStageSim::reset_simulation() {
    stop_simulation();
    wafer_stage::StageParams params;
    params.mass = mass;
    params.lorentz_force_constant = lorentz_force_constant;
    params.spring_stiffness = spring_stiffness;
    params.passive_damping = passive_damping;
    params.active_stiffness = active_stiffness;
    params.active_damping = active_damping;
    params.active_integral_gain = active_integral_gain;
    params.target_position = target_position;
    params.accel_command = accel_g * wafer_stage::StageDynamics::GRAVITY;
    params.simulation_dt = 1.0 / simulation_frequency;
    params.settling_threshold_nm = settling_threshold_nm;
    params.max_force = max_force;
    params.derivative_filter_cutoff = derivative_filter_cutoff;
    params.derivative_filter_sample_rate = derivative_filter_sample_rate;
    params.noise_enabled = noise_enabled;
    params.noise_acoustic_amplitude_nm = noise_acoustic_amplitude_nm;
    params.noise_acoustic_cutoff_hz = noise_acoustic_cutoff_hz;
    params.noise_drift_amplitude_nm = noise_drift_amplitude_nm;
    params.noise_drift_bandwidth_hz = noise_drift_bandwidth_hz;
    params.noise_seed = static_cast<uint64_t>(noise_seed);

    dynamics_.initialize(params);
}

void ReticleStageSim::stop_simulation() {
    if (use_threaded_mode_) {
        dynamics_.stop_threaded_simulation();
    }
    running_ = false;
}

double ReticleStageSim::get_mass() const { return mass; }
void ReticleStageSim::set_mass(double p_mass) { mass = p_mass; }

double ReticleStageSim::get_lorentz_force_constant() const { return lorentz_force_constant; }
void ReticleStageSim::set_lorentz_force_constant(double p_kf) { lorentz_force_constant = p_kf; }

double ReticleStageSim::get_spring_stiffness() const { return spring_stiffness; }
void ReticleStageSim::set_spring_stiffness(double p_ks) { spring_stiffness = p_ks; }

double ReticleStageSim::get_passive_damping() const { return passive_damping; }
void ReticleStageSim::set_passive_damping(double p_cd) { passive_damping = p_cd; }

double ReticleStageSim::get_active_stiffness() const { return active_stiffness; }
void ReticleStageSim::set_active_stiffness(double p_ka) { active_stiffness = p_ka; }

double ReticleStageSim::get_active_damping() const { return active_damping; }
void ReticleStageSim::set_active_damping(double p_ca) { active_damping = p_ca; }

double ReticleStageSim::get_active_integral_gain() const { return active_integral_gain; }
void ReticleStageSim::set_active_integral_gain(double p_ki) { active_integral_gain = p_ki; }

double ReticleStageSim::get_target_position() const { return target_position; }
void ReticleStageSim::set_target_position(double p_target) {
    target_position = p_target;
    dynamics_.set_target_position(p_target);
}

double ReticleStageSim::get_accel_g() const { return accel_g; }
void ReticleStageSim::set_accel_g(double p_g) {
    accel_g = p_g;
    dynamics_.set_accel_command(p_g * wafer_stage::StageDynamics::GRAVITY);
}

double ReticleStageSim::get_simulation_frequency() const { return simulation_frequency; }
void ReticleStageSim::set_simulation_frequency(double p_freq) { simulation_frequency = p_freq; }

double ReticleStageSim::get_settling_threshold_nm() const { return settling_threshold_nm; }
void ReticleStageSim::set_settling_threshold_nm(double p_nm) { settling_threshold_nm = p_nm; }

double ReticleStageSim::get_max_force() const { return max_force; }
void ReticleStageSim::set_max_force(double p_force) { max_force = p_force; }

double ReticleStageSim::get_derivative_filter_cutoff() const { return derivative_filter_cutoff; }
void ReticleStageSim::set_derivative_filter_cutoff(double p_hz) { derivative_filter_cutoff = p_hz; }

double ReticleStageSim::get_derivative_filter_sample_rate() const { return derivative_filter_sample_rate; }
void ReticleStageSim::set_derivative_filter_sample_rate(double p_hz) { derivative_filter_sample_rate = p_hz; }

bool ReticleStageSim::get_use_threaded_mode() const { return use_threaded_mode_; }
void ReticleStageSim::set_use_threaded_mode(bool p_enable) {
    if (running_) {
        stop_simulation();
    }
    use_threaded_mode_ = p_enable;
}

bool ReticleStageSim::get_noise_enabled() const { return noise_enabled; }
void ReticleStageSim::set_noise_enabled(bool p_enable) {
    noise_enabled = p_enable;
    dynamics_.set_noise_enabled(p_enable);
}

double ReticleStageSim::get_noise_acoustic_amplitude_nm() const { return noise_acoustic_amplitude_nm; }
void ReticleStageSim::set_noise_acoustic_amplitude_nm(double p_amp) {
    noise_acoustic_amplitude_nm = p_amp;
    dynamics_.set_noise_acoustic_amplitude_nm(p_amp);
}

double ReticleStageSim::get_noise_acoustic_cutoff_hz() const { return noise_acoustic_cutoff_hz; }
void ReticleStageSim::set_noise_acoustic_cutoff_hz(double p_hz) {
    noise_acoustic_cutoff_hz = p_hz;
}

double ReticleStageSim::get_noise_drift_amplitude_nm() const { return noise_drift_amplitude_nm; }
void ReticleStageSim::set_noise_drift_amplitude_nm(double p_amp) {
    noise_drift_amplitude_nm = p_amp;
    dynamics_.set_noise_drift_amplitude_nm(p_amp);
}

double ReticleStageSim::get_noise_drift_bandwidth_hz() const { return noise_drift_bandwidth_hz; }
void ReticleStageSim::set_noise_drift_bandwidth_hz(double p_hz) {
    noise_drift_bandwidth_hz = p_hz;
}

int64_t ReticleStageSim::get_noise_seed() const { return noise_seed; }
void ReticleStageSim::set_noise_seed(int64_t p_seed) {
    noise_seed = p_seed;
    dynamics_.set_noise_seed(static_cast<uint64_t>(p_seed));
}

double ReticleStageSim::get_current_noise_nm() const {
    return dynamics_.get_current_noise_nm();
}

double ReticleStageSim::get_time_scale() const { return time_scale_; }
void ReticleStageSim::set_time_scale(double p_scale) { time_scale_ = p_scale; }

double ReticleStageSim::get_max_sim_time() const { return max_sim_time_; }
void ReticleStageSim::set_max_sim_time(double p_time) { max_sim_time_ = p_time; }

double ReticleStageSim::get_position() const { return dynamics_.get_position(); }
double ReticleStageSim::get_velocity() const { return dynamics_.get_velocity(); }
double ReticleStageSim::get_position_error() const { return dynamics_.get_position_error(); }
double ReticleStageSim::get_position_error_nm() const { return dynamics_.get_position_error_nm(); }
double ReticleStageSim::get_sim_time() const { return dynamics_.get_sim_time(); }
int ReticleStageSim::get_phase() const { return static_cast<int>(dynamics_.get_phase()); }
double ReticleStageSim::get_settling_time() const { return dynamics_.get_settling_time(); }
bool ReticleStageSim::is_settled() const { return dynamics_.is_settled(); }
bool ReticleStageSim::is_running() const { return running_; }
bool ReticleStageSim::has_nan() const { return dynamics_.has_nan(); }
uint32_t ReticleStageSim::get_nan_recovery_count() const { return dynamics_.get_nan_recovery_count(); }
uint64_t ReticleStageSim::get_step_counter() const { return dynamics_.get_step_counter(); }

PackedFloat64Array ReticleStageSim::get_error_time_series() const {
    auto wf = dynamics_.get_waveform_copy();
    PackedFloat64Array arr;
    arr.resize(wf.size());
    for (size_t i = 0; i < wf.size(); i++) {
        arr[i] = wf[i].error * 1e9;
    }
    return arr;
}

PackedFloat64Array ReticleStageSim::get_position_time_series() const {
    auto wf = dynamics_.get_waveform_copy();
    PackedFloat64Array arr;
    arr.resize(wf.size());
    for (size_t i = 0; i < wf.size(); i++) {
        arr[i] = wf[i].position * 1000.0;
    }
    return arr;
}

PackedFloat64Array ReticleStageSim::get_velocity_time_series() const {
    auto wf = dynamics_.get_waveform_copy();
    PackedFloat64Array arr;
    arr.resize(wf.size());
    for (size_t i = 0; i < wf.size(); i++) {
        arr[i] = wf[i].velocity;
    }
    return arr;
}

PackedFloat64Array ReticleStageSim::get_time_series() const {
    auto wf = dynamics_.get_waveform_copy();
    PackedFloat64Array arr;
    arr.resize(wf.size());
    for (size_t i = 0; i < wf.size(); i++) {
        arr[i] = wf[i].time * 1000.0;
    }
    return arr;
}

PackedFloat64Array ReticleStageSim::get_lorentz_force_series() const {
    auto wf = dynamics_.get_waveform_copy();
    PackedFloat64Array arr;
    arr.resize(wf.size());
    for (size_t i = 0; i < wf.size(); i++) {
        arr[i] = wf[i].force_lorentz;
    }
    return arr;
}

PackedFloat64Array ReticleStageSim::get_spring_force_series() const {
    auto wf = dynamics_.get_waveform_copy();
    PackedFloat64Array arr;
    arr.resize(wf.size());
    for (size_t i = 0; i < wf.size(); i++) {
        arr[i] = wf[i].force_spring;
    }
    return arr;
}

PackedFloat64Array ReticleStageSim::get_damping_force_series() const {
    auto wf = dynamics_.get_waveform_copy();
    PackedFloat64Array arr;
    arr.resize(wf.size());
    for (size_t i = 0; i < wf.size(); i++) {
        arr[i] = wf[i].force_damping;
    }
    return arr;
}

int ReticleStageSim::get_sample_count() const {
    return static_cast<int>(dynamics_.get_waveform_size());
}

void ReticleStageSim::_bind_methods() {
    ClassDB::bind_method(D_METHOD("_process", "delta"), &ReticleStageSim::_process);
    ClassDB::bind_method(D_METHOD("_ready"), &ReticleStageSim::_ready);

    ClassDB::bind_method(D_METHOD("start_simulation"), &ReticleStageSim::start_simulation);
    ClassDB::bind_method(D_METHOD("reset_simulation"), &ReticleStageSim::reset_simulation);
    ClassDB::bind_method(D_METHOD("stop_simulation"), &ReticleStageSim::stop_simulation);

    ClassDB::bind_method(D_METHOD("get_mass"), &ReticleStageSim::get_mass);
    ClassDB::bind_method(D_METHOD("set_mass", "mass"), &ReticleStageSim::set_mass);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass", PROPERTY_HINT_RANGE, "1,200,0.1"), "set_mass", "get_mass");

    ClassDB::bind_method(D_METHOD("get_lorentz_force_constant"), &ReticleStageSim::get_lorentz_force_constant);
    ClassDB::bind_method(D_METHOD("set_lorentz_force_constant", "kf"), &ReticleStageSim::set_lorentz_force_constant);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "lorentz_force_constant", PROPERTY_HINT_RANGE, "1,200,0.1"), "set_lorentz_force_constant", "get_lorentz_force_constant");

    ClassDB::bind_method(D_METHOD("get_spring_stiffness"), &ReticleStageSim::get_spring_stiffness);
    ClassDB::bind_method(D_METHOD("set_spring_stiffness", "ks"), &ReticleStageSim::set_spring_stiffness);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spring_stiffness", PROPERTY_HINT_RANGE, "0,1e8,1e3"), "set_spring_stiffness", "get_spring_stiffness");

    ClassDB::bind_method(D_METHOD("get_passive_damping"), &ReticleStageSim::get_passive_damping);
    ClassDB::bind_method(D_METHOD("set_passive_damping", "cd"), &ReticleStageSim::set_passive_damping);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "passive_damping", PROPERTY_HINT_RANGE, "0,10000,10"), "set_passive_damping", "get_passive_damping");

    ClassDB::bind_method(D_METHOD("get_active_stiffness"), &ReticleStageSim::get_active_stiffness);
    ClassDB::bind_method(D_METHOD("set_active_stiffness", "ka"), &ReticleStageSim::set_active_stiffness);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "active_stiffness", PROPERTY_HINT_RANGE, "1e5,1e9,1e4"), "set_active_stiffness", "get_active_stiffness");

    ClassDB::bind_method(D_METHOD("get_active_damping"), &ReticleStageSim::get_active_damping);
    ClassDB::bind_method(D_METHOD("set_active_damping", "ca"), &ReticleStageSim::set_active_damping);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "active_damping", PROPERTY_HINT_RANGE, "1e3,1e6,1e2"), "set_active_damping", "get_active_damping");

    ClassDB::bind_method(D_METHOD("get_active_integral_gain"), &ReticleStageSim::get_active_integral_gain);
    ClassDB::bind_method(D_METHOD("set_active_integral_gain", "ki"), &ReticleStageSim::set_active_integral_gain);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "active_integral_gain", PROPERTY_HINT_RANGE, "1e4,1e10,1e4"), "set_active_integral_gain", "get_active_integral_gain");

    ClassDB::bind_method(D_METHOD("get_target_position"), &ReticleStageSim::get_target_position);
    ClassDB::bind_method(D_METHOD("set_target_position", "target"), &ReticleStageSim::set_target_position);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "target_position", PROPERTY_HINT_RANGE, "-0.1,0.1,0.0001"), "set_target_position", "get_target_position");

    ClassDB::bind_method(D_METHOD("get_accel_g"), &ReticleStageSim::get_accel_g);
    ClassDB::bind_method(D_METHOD("set_accel_g", "g"), &ReticleStageSim::set_accel_g);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "accel_g", PROPERTY_HINT_RANGE, "0.1,10,0.1"), "set_accel_g", "get_accel_g");

    ClassDB::bind_method(D_METHOD("get_simulation_frequency"), &ReticleStageSim::get_simulation_frequency);
    ClassDB::bind_method(D_METHOD("set_simulation_frequency", "freq"), &ReticleStageSim::set_simulation_frequency);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "simulation_frequency", PROPERTY_HINT_RANGE, "1000,100000,1000"), "set_simulation_frequency", "get_simulation_frequency");

    ClassDB::bind_method(D_METHOD("get_settling_threshold_nm"), &ReticleStageSim::get_settling_threshold_nm);
    ClassDB::bind_method(D_METHOD("set_settling_threshold_nm", "nm"), &ReticleStageSim::set_settling_threshold_nm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "settling_threshold_nm", PROPERTY_HINT_RANGE, "0.01,10,0.01"), "set_settling_threshold_nm", "get_settling_threshold_nm");

    ClassDB::bind_method(D_METHOD("get_max_force"), &ReticleStageSim::get_max_force);
    ClassDB::bind_method(D_METHOD("set_max_force", "force"), &ReticleStageSim::set_max_force);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_force", PROPERTY_HINT_RANGE, "100,100000,100"), "set_max_force", "get_max_force");

    ClassDB::bind_method(D_METHOD("get_derivative_filter_cutoff"), &ReticleStageSim::get_derivative_filter_cutoff);
    ClassDB::bind_method(D_METHOD("set_derivative_filter_cutoff", "hz"), &ReticleStageSim::set_derivative_filter_cutoff);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "derivative_filter_cutoff_hz", PROPERTY_HINT_RANGE, "10,5000,10"), "set_derivative_filter_cutoff", "get_derivative_filter_cutoff");

    ClassDB::bind_method(D_METHOD("get_derivative_filter_sample_rate"), &ReticleStageSim::get_derivative_filter_sample_rate);
    ClassDB::bind_method(D_METHOD("set_derivative_filter_sample_rate", "hz"), &ReticleStageSim::set_derivative_filter_sample_rate);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "derivative_filter_sample_rate_hz", PROPERTY_HINT_RANGE, "1000,100000,1000"), "set_derivative_filter_sample_rate", "get_derivative_filter_sample_rate");

    ClassDB::bind_method(D_METHOD("get_use_threaded_mode"), &ReticleStageSim::get_use_threaded_mode);
    ClassDB::bind_method(D_METHOD("set_use_threaded_mode", "enable"), &ReticleStageSim::set_use_threaded_mode);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_threaded_10kHz_physics"), "set_use_threaded_mode", "get_use_threaded_mode");

    ClassDB::bind_method(D_METHOD("get_noise_enabled"), &ReticleStageSim::get_noise_enabled);
    ClassDB::bind_method(D_METHOD("set_noise_enabled", "enable"), &ReticleStageSim::set_noise_enabled);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "noise/ffu_turbulence_enabled"), "set_noise_enabled", "get_noise_enabled");

    ClassDB::bind_method(D_METHOD("get_noise_acoustic_amplitude_nm"), &ReticleStageSim::get_noise_acoustic_amplitude_nm);
    ClassDB::bind_method(D_METHOD("set_noise_acoustic_amplitude_nm", "amp_nm"), &ReticleStageSim::set_noise_acoustic_amplitude_nm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "noise/acoustic_amplitude_nm", PROPERTY_HINT_RANGE, "0.0,5.0,0.01"), "set_noise_acoustic_amplitude_nm", "get_noise_acoustic_amplitude_nm");

    ClassDB::bind_method(D_METHOD("get_noise_acoustic_cutoff_hz"), &ReticleStageSim::get_noise_acoustic_cutoff_hz);
    ClassDB::bind_method(D_METHOD("set_noise_acoustic_cutoff_hz", "hz"), &ReticleStageSim::set_noise_acoustic_cutoff_hz);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "noise/acoustic_cutoff_hz", PROPERTY_HINT_RANGE, "10,5000,10"), "set_noise_acoustic_cutoff_hz", "get_noise_acoustic_cutoff_hz");

    ClassDB::bind_method(D_METHOD("get_noise_drift_amplitude_nm"), &ReticleStageSim::get_noise_drift_amplitude_nm);
    ClassDB::bind_method(D_METHOD("set_noise_drift_amplitude_nm", "amp_nm"), &ReticleStageSim::set_noise_drift_amplitude_nm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "noise/drift_amplitude_nm", PROPERTY_HINT_RANGE, "0.0,2.0,0.001"), "set_noise_drift_amplitude_nm", "get_noise_drift_amplitude_nm");

    ClassDB::bind_method(D_METHOD("get_noise_drift_bandwidth_hz"), &ReticleStageSim::get_noise_drift_bandwidth_hz);
    ClassDB::bind_method(D_METHOD("set_noise_drift_bandwidth_hz", "hz"), &ReticleStageSim::set_noise_drift_bandwidth_hz);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "noise/drift_bandwidth_hz", PROPERTY_HINT_RANGE, "0.001,1.0,0.001"), "set_noise_drift_bandwidth_hz", "get_noise_drift_bandwidth_hz");

    ClassDB::bind_method(D_METHOD("get_noise_seed"), &ReticleStageSim::get_noise_seed);
    ClassDB::bind_method(D_METHOD("set_noise_seed", "seed"), &ReticleStageSim::set_noise_seed);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "noise/seed"), "set_noise_seed", "get_noise_seed");

    ClassDB::bind_method(D_METHOD("get_current_noise_nm"), &ReticleStageSim::get_current_noise_nm);

    ClassDB::bind_method(D_METHOD("get_time_scale"), &ReticleStageSim::get_time_scale);
    ClassDB::bind_method(D_METHOD("set_time_scale", "scale"), &ReticleStageSim::set_time_scale);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "time_scale", PROPERTY_HINT_RANGE, "0.01,10,0.01"), "set_time_scale", "get_time_scale");

    ClassDB::bind_method(D_METHOD("get_max_sim_time"), &ReticleStageSim::get_max_sim_time);
    ClassDB::bind_method(D_METHOD("set_max_sim_time", "time"), &ReticleStageSim::set_max_sim_time);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_sim_time", PROPERTY_HINT_RANGE, "0.1,100,0.1"), "set_max_sim_time", "get_max_sim_time");

    ClassDB::bind_method(D_METHOD("get_position"), &ReticleStageSim::get_position);
    ClassDB::bind_method(D_METHOD("get_velocity"), &ReticleStageSim::get_velocity);
    ClassDB::bind_method(D_METHOD("get_position_error"), &ReticleStageSim::get_position_error);
    ClassDB::bind_method(D_METHOD("get_position_error_nm"), &ReticleStageSim::get_position_error_nm);
    ClassDB::bind_method(D_METHOD("get_sim_time"), &ReticleStageSim::get_sim_time);
    ClassDB::bind_method(D_METHOD("get_phase"), &ReticleStageSim::get_phase);
    ClassDB::bind_method(D_METHOD("get_settling_time"), &ReticleStageSim::get_settling_time);
    ClassDB::bind_method(D_METHOD("is_settled"), &ReticleStageSim::is_settled);
    ClassDB::bind_method(D_METHOD("is_running"), &ReticleStageSim::is_running);
    ClassDB::bind_method(D_METHOD("has_nan"), &ReticleStageSim::has_nan);
    ClassDB::bind_method(D_METHOD("get_nan_recovery_count"), &ReticleStageSim::get_nan_recovery_count);
    ClassDB::bind_method(D_METHOD("get_step_counter"), &ReticleStageSim::get_step_counter);

    ClassDB::bind_method(D_METHOD("get_error_time_series"), &ReticleStageSim::get_error_time_series);
    ClassDB::bind_method(D_METHOD("get_position_time_series"), &ReticleStageSim::get_position_time_series);
    ClassDB::bind_method(D_METHOD("get_velocity_time_series"), &ReticleStageSim::get_velocity_time_series);
    ClassDB::bind_method(D_METHOD("get_time_series"), &ReticleStageSim::get_time_series);
    ClassDB::bind_method(D_METHOD("get_lorentz_force_series"), &ReticleStageSim::get_lorentz_force_series);
    ClassDB::bind_method(D_METHOD("get_spring_force_series"), &ReticleStageSim::get_spring_force_series);
    ClassDB::bind_method(D_METHOD("get_damping_force_series"), &ReticleStageSim::get_damping_force_series);
    ClassDB::bind_method(D_METHOD("get_sample_count"), &ReticleStageSim::get_sample_count);
}
