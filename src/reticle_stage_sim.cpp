#include "reticle_stage_sim.h"
#include <godot_cpp/core/class_db.hpp>

namespace godot {

ReticleStageSim::ReticleStageSim()
    : running_(false)
    , time_scale_(1.0)
    , steps_per_frame_(0)
    , max_sim_time_(0.2)
    , mass(50.0)
    , lorentz_force_constant(50.0)
    , spring_stiffness(0.0)
    , passive_damping(2000.0)
    , active_stiffness(1.5e7)
    , active_damping(60000.0)
    , active_integral_gain(5.0e6)
    , target_position(0.01)
    , accel_g(2.0)
    , simulation_frequency(100000.0)
    , settling_threshold_nm(0.1)
    , max_force(50000.0) {}

ReticleStageSim::~ReticleStageSim() {}

void ReticleStageSim::_ready() {
    reset_simulation();
}

void ReticleStageSim::_process(double delta) {
    if (!running_) return;

    double sim_delta = delta * time_scale_;
    double dt = 1.0 / simulation_frequency;
    int steps = static_cast<int>(sim_delta / dt);
    steps = std::min(steps, 100000);

    for (int i = 0; i < steps; i++) {
        if (dynamics_.get_sim_time() >= max_sim_time_) {
            running_ = false;
            emit_signal("simulation_completed");
            return;
        }
        dynamics_.step();

        if (dynamics_.is_settled() && !running_) {
            return;
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

    dynamics_.initialize(params);
    running_ = true;
}

void ReticleStageSim::reset_simulation() {
    running_ = false;
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

    dynamics_.initialize(params);
}

void ReticleStageSim::stop_simulation() {
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
void ReticleStageSim::set_target_position(double p_target) { target_position = p_target; }

double ReticleStageSim::get_accel_g() const { return accel_g; }
void ReticleStageSim::set_accel_g(double p_g) { accel_g = p_g; }

double ReticleStageSim::get_simulation_frequency() const { return simulation_frequency; }
void ReticleStageSim::set_simulation_frequency(double p_freq) { simulation_frequency = p_freq; }

double ReticleStageSim::get_settling_threshold_nm() const { return settling_threshold_nm; }
void ReticleStageSim::set_settling_threshold_nm(double p_nm) { settling_threshold_nm = p_nm; }

double ReticleStageSim::get_max_force() const { return max_force; }
void ReticleStageSim::set_max_force(double p_force) { max_force = p_force; }

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

PackedFloat64Array ReticleStageSim::get_error_time_series() const {
    const auto &waveform = dynamics_.get_waveform();
    PackedFloat64Array arr;
    arr.resize(waveform.size());
    for (size_t i = 0; i < waveform.size(); i++) {
        arr[i] = waveform[i].error * 1e9;
    }
    return arr;
}

PackedFloat64Array ReticleStageSim::get_position_time_series() const {
    const auto &waveform = dynamics_.get_waveform();
    PackedFloat64Array arr;
    arr.resize(waveform.size());
    for (size_t i = 0; i < waveform.size(); i++) {
        arr[i] = waveform[i].position;
    }
    return arr;
}

PackedFloat64Array ReticleStageSim::get_velocity_time_series() const {
    const auto &waveform = dynamics_.get_waveform();
    PackedFloat64Array arr;
    arr.resize(waveform.size());
    for (size_t i = 0; i < waveform.size(); i++) {
        arr[i] = waveform[i].velocity;
    }
    return arr;
}

PackedFloat64Array ReticleStageSim::get_time_series() const {
    const auto &waveform = dynamics_.get_waveform();
    PackedFloat64Array arr;
    arr.resize(waveform.size());
    for (size_t i = 0; i < waveform.size(); i++) {
        arr[i] = waveform[i].time * 1000.0;
    }
    return arr;
}

PackedFloat64Array ReticleStageSim::get_lorentz_force_series() const {
    const auto &waveform = dynamics_.get_waveform();
    PackedFloat64Array arr;
    arr.resize(waveform.size());
    for (size_t i = 0; i < waveform.size(); i++) {
        arr[i] = waveform[i].force_lorentz;
    }
    return arr;
}

PackedFloat64Array ReticleStageSim::get_spring_force_series() const {
    const auto &waveform = dynamics_.get_waveform();
    PackedFloat64Array arr;
    arr.resize(waveform.size());
    for (size_t i = 0; i < waveform.size(); i++) {
        arr[i] = waveform[i].force_spring;
    }
    return arr;
}

PackedFloat64Array ReticleStageSim::get_damping_force_series() const {
    const auto &waveform = dynamics_.get_waveform();
    PackedFloat64Array arr;
    arr.resize(waveform.size());
    for (size_t i = 0; i < waveform.size(); i++) {
        arr[i] = waveform[i].force_damping;
    }
    return arr;
}

int ReticleStageSim::get_sample_count() const {
    return static_cast<int>(dynamics_.get_waveform().size());
}

void ReticleStageSim::_bind_methods() {
    ClassDB::bind_method(D_METHOD("start_simulation"), &ReticleStageSim::start_simulation);
    ClassDB::bind_method(D_METHOD("reset_simulation"), &ReticleStageSim::reset_simulation);
    ClassDB::bind_method(D_METHOD("stop_simulation"), &ReticleStageSim::stop_simulation);

    ClassDB::bind_method(D_METHOD("get_mass"), &ReticleStageSim::get_mass);
    ClassDB::bind_method(D_METHOD("set_mass", "mass"), &ReticleStageSim::set_mass);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass", PROPERTY_HINT_RANGE, "1,500,0.1"), "set_mass", "get_mass");

    ClassDB::bind_method(D_METHOD("get_lorentz_force_constant"), &ReticleStageSim::get_lorentz_force_constant);
    ClassDB::bind_method(D_METHOD("set_lorentz_force_constant", "kf"), &ReticleStageSim::set_lorentz_force_constant);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "lorentz_force_constant", PROPERTY_HINT_RANGE, "1,500,0.1"), "set_lorentz_force_constant", "get_lorentz_force_constant");

    ClassDB::bind_method(D_METHOD("get_spring_stiffness"), &ReticleStageSim::get_spring_stiffness);
    ClassDB::bind_method(D_METHOD("set_spring_stiffness", "ks"), &ReticleStageSim::set_spring_stiffness);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spring_stiffness", PROPERTY_HINT_RANGE, "1e4,1e8,1e3"), "set_spring_stiffness", "get_spring_stiffness");

    ClassDB::bind_method(D_METHOD("get_passive_damping"), &ReticleStageSim::get_passive_damping);
    ClassDB::bind_method(D_METHOD("set_passive_damping", "cd"), &ReticleStageSim::set_passive_damping);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "passive_damping", PROPERTY_HINT_RANGE, "0,50000,100"), "set_passive_damping", "get_passive_damping");

    ClassDB::bind_method(D_METHOD("get_active_stiffness"), &ReticleStageSim::get_active_stiffness);
    ClassDB::bind_method(D_METHOD("set_active_stiffness", "ka"), &ReticleStageSim::set_active_stiffness);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "active_stiffness", PROPERTY_HINT_RANGE, "1e4,1e8,1e3"), "set_active_stiffness", "get_active_stiffness");

    ClassDB::bind_method(D_METHOD("get_active_damping"), &ReticleStageSim::get_active_damping);
    ClassDB::bind_method(D_METHOD("set_active_damping", "ca"), &ReticleStageSim::set_active_damping);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "active_damping", PROPERTY_HINT_RANGE, "0,100000,100"), "set_active_damping", "get_active_damping");

    ClassDB::bind_method(D_METHOD("get_active_integral_gain"), &ReticleStageSim::get_active_integral_gain);
    ClassDB::bind_method(D_METHOD("set_active_integral_gain", "ki"), &ReticleStageSim::set_active_integral_gain);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "active_integral_gain", PROPERTY_HINT_RANGE, "1e4,1e10,1e4"), "set_active_integral_gain", "get_active_integral_gain");

    ClassDB::bind_method(D_METHOD("get_target_position"), &ReticleStageSim::get_target_position);
    ClassDB::bind_method(D_METHOD("set_target_position", "target"), &ReticleStageSim::set_target_position);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "target_position", PROPERTY_HINT_RANGE, "0.001,0.5,0.001"), "set_target_position", "get_target_position");

    ClassDB::bind_method(D_METHOD("get_accel_g"), &ReticleStageSim::get_accel_g);
    ClassDB::bind_method(D_METHOD("set_accel_g", "g"), &ReticleStageSim::set_accel_g);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "accel_g", PROPERTY_HINT_RANGE, "0.1,10,0.1"), "set_accel_g", "get_accel_g");

    ClassDB::bind_method(D_METHOD("get_simulation_frequency"), &ReticleStageSim::get_simulation_frequency);
    ClassDB::bind_method(D_METHOD("set_simulation_frequency", "freq"), &ReticleStageSim::set_simulation_frequency);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "simulation_frequency", PROPERTY_HINT_RANGE, "10000,1000000,1000"), "set_simulation_frequency", "get_simulation_frequency");

    ClassDB::bind_method(D_METHOD("get_settling_threshold_nm"), &ReticleStageSim::get_settling_threshold_nm);
    ClassDB::bind_method(D_METHOD("set_settling_threshold_nm", "nm"), &ReticleStageSim::set_settling_threshold_nm);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "settling_threshold_nm", PROPERTY_HINT_RANGE, "0.01,10,0.01"), "set_settling_threshold_nm", "get_settling_threshold_nm");

    ClassDB::bind_method(D_METHOD("get_max_force"), &ReticleStageSim::get_max_force);
    ClassDB::bind_method(D_METHOD("set_max_force", "force"), &ReticleStageSim::set_max_force);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_force", PROPERTY_HINT_RANGE, "100,100000,100"), "set_max_force", "get_max_force");

    ClassDB::bind_method(D_METHOD("get_time_scale"), &ReticleStageSim::get_time_scale);
    ClassDB::bind_method(D_METHOD("set_time_scale", "scale"), &ReticleStageSim::set_time_scale);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "time_scale", PROPERTY_HINT_RANGE, "0.01,10,0.01"), "set_time_scale", "get_time_scale");

    ClassDB::bind_method(D_METHOD("get_max_sim_time"), &ReticleStageSim::get_max_sim_time);
    ClassDB::bind_method(D_METHOD("set_max_sim_time", "time"), &ReticleStageSim::set_max_sim_time);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_sim_time", PROPERTY_HINT_RANGE, "0.01,5,0.01"), "set_max_sim_time", "get_max_sim_time");

    ClassDB::bind_method(D_METHOD("get_position"), &ReticleStageSim::get_position);
    ClassDB::bind_method(D_METHOD("get_velocity"), &ReticleStageSim::get_velocity);
    ClassDB::bind_method(D_METHOD("get_position_error"), &ReticleStageSim::get_position_error);
    ClassDB::bind_method(D_METHOD("get_position_error_nm"), &ReticleStageSim::get_position_error_nm);
    ClassDB::bind_method(D_METHOD("get_sim_time"), &ReticleStageSim::get_sim_time);
    ClassDB::bind_method(D_METHOD("get_phase"), &ReticleStageSim::get_phase);
    ClassDB::bind_method(D_METHOD("get_settling_time"), &ReticleStageSim::get_settling_time);
    ClassDB::bind_method(D_METHOD("is_settled"), &ReticleStageSim::is_settled);
    ClassDB::bind_method(D_METHOD("is_running"), &ReticleStageSim::is_running);

    ClassDB::bind_method(D_METHOD("get_error_time_series"), &ReticleStageSim::get_error_time_series);
    ClassDB::bind_method(D_METHOD("get_position_time_series"), &ReticleStageSim::get_position_time_series);
    ClassDB::bind_method(D_METHOD("get_velocity_time_series"), &ReticleStageSim::get_velocity_time_series);
    ClassDB::bind_method(D_METHOD("get_time_series"), &ReticleStageSim::get_time_series);
    ClassDB::bind_method(D_METHOD("get_lorentz_force_series"), &ReticleStageSim::get_lorentz_force_series);
    ClassDB::bind_method(D_METHOD("get_spring_force_series"), &ReticleStageSim::get_spring_force_series);
    ClassDB::bind_method(D_METHOD("get_damping_force_series"), &ReticleStageSim::get_damping_force_series);
    ClassDB::bind_method(D_METHOD("get_sample_count"), &ReticleStageSim::get_sample_count);

    ADD_SIGNAL(MethodInfo("simulation_completed"));
    ADD_SIGNAL(MethodInfo("settling_achieved", PropertyInfo(Variant::FLOAT, "settling_time_ms")));
}

}
