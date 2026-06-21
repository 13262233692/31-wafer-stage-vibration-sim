#include "stage_dynamics.h"
#include <cstdio>
#include <cmath>
#include <fstream>
#include <iomanip>

int main() {
    wafer_stage::StageDynamics dynamics;

    wafer_stage::StageParams params;
    params.mass = 50.0;
    params.lorentz_force_constant = 50.0;
    params.spring_stiffness = 0.0;
    params.passive_damping = 2000.0;
    params.active_stiffness = 1.5e7;
    params.active_damping = 60000.0;
    params.active_integral_gain = 5.0e6;
    params.target_position = 0.01;
    params.accel_command = 2.0 * wafer_stage::StageDynamics::GRAVITY;
    params.simulation_dt = 1.0e-4;
    params.settling_threshold_nm = 0.1;
    params.max_force = 50000.0;
    params.derivative_filter_cutoff = 1500.0;
    params.derivative_filter_sample_rate = 1.0 / params.simulation_dt;

    dynamics.initialize(params);

    std::ofstream csv("settling_data_5step.csv");
    csv << "time_ms,position_mm,error_nm,phase,velocity_ms,nan_recovery\n";

    const char *phase_names[] = {"IDLE", "ACCEL", "DECEL", "SETTLE", "SETTLED"};

    printf("=== Wafer Stage Vibration - 5 Consecutive Step Scan Stability Test ===\n");
    printf("Parameters:\n");
    printf("  Mass:              %.1f kg\n", params.mass);
    printf("  Lorentz K_f:       %.1f N/A\n", params.lorentz_force_constant);
    printf("  Spring K_s:        %.2e N/m\n", params.spring_stiffness);
    printf("  Passive damping:   %.1f N.s/m\n", params.passive_damping);
    printf("  Active stiffness:  %.2e N/m\n", params.active_stiffness);
    printf("  Active damping:    %.1f N.s/m\n", params.active_damping);
    printf("  Active integral:   %.2e N/(m.s)\n", params.active_integral_gain);
    printf("  Simulation dt:     %.1e s (%.0f Hz)\n", params.simulation_dt, 1.0/params.simulation_dt);
    printf("  Derivative LPF:    %.0f Hz (4th-order Elliptic)\n", params.derivative_filter_cutoff);
    printf("  Settle threshold:  %.1f nm\n", params.settling_threshold_nm);
    printf("  Max force:         %.0f N\n", params.max_force);
    printf("\n");

    double targets[] = {0.01, -0.005, 0.015, 0.0, 0.01};
    int num_steps = 5;
    double max_error_nm = 0.0;
    double min_error_nm = 0.0;
    double settle_times[5] = {0};
    bool all_passed = true;
    int total_steps = 0;

    for (int s = 0; s < num_steps; s++) {
        printf("\n--- Step %d: Target = %.3f mm ---\n", s + 1, targets[s] * 1000.0);

        if (s == 0) {
            dynamics.reset();
        } else {
            dynamics.set_target_position(targets[s]);
        }

        int settle_count = 0;
        bool achieved = false;
        double settle_time = 0.0;
        double step_start_time = dynamics.get_sim_time();

        for (int i = 0; i < 50000; i++) {
            dynamics.step();
            total_steps++;

            double t_ms = dynamics.get_sim_time() * 1000.0;
            double pos_mm = dynamics.get_position() * 1000.0;
            double err_nm = dynamics.get_position_error_nm();
            int phase = static_cast<int>(dynamics.get_phase());
            double vel = dynamics.get_velocity();
            uint32_t nan_rec = dynamics.get_nan_recovery_count();

            csv << std::fixed << std::setprecision(4) << t_ms << ","
                << std::setprecision(8) << pos_mm << ","
                << std::setprecision(4) << err_nm << ","
                << phase << ","
                << std::setprecision(8) << vel << ","
                << nan_rec << "\n";

            if (err_nm > max_error_nm) max_error_nm = err_nm;
            if (err_nm < min_error_nm) min_error_nm = err_nm;

            if (dynamics.is_settled() && !achieved) {
                achieved = true;
                settle_time = dynamics.get_settling_time() * 1000.0;
                settle_times[s] = settle_time;
                printf("  [OK] Settled at t_rel=%.3f ms, error=%.4f nm, vel=%.8f m/s\n",
                       settle_time, err_nm, vel);
            }

            if (i % 5000 == 0) {
                printf("  t=%8.2fms  pos=%9.5fmm  err=%+11.4fnm  phase=%-8s  nan_rec=%u\n",
                       t_ms, pos_mm, err_nm,
                       phase < 5 ? phase_names[phase] : "???",
                       nan_rec);
            }

            if (dynamics.has_nan()) {
                printf("  [WARN] NaN detected & recovered! Count: %u\n", dynamics.get_nan_recovery_count());
            }
        }

        if (!achieved) {
            all_passed = false;
            printf("  [FAIL] Not settled within 50000 steps (5.0s)\n");
        }

        double post_err = dynamics.get_position_error_nm();
        printf("  Post-settle error: %.6f nm\n", post_err);
        if (std::abs(post_err) > params.settling_threshold_nm) {
            all_passed = false;
            printf("  [FAIL] Post-settle stability violated!\n");
        }
    }

    printf("\n==================== SUMMARY ====================\n");
    for (int s = 0; s < num_steps; s++) {
        printf("  Step %d: target=%+.3fmm  settle_time=%.3f ms\n",
               s + 1, targets[s] * 1000.0, settle_times[s]);
    }
    printf("  Total simulation steps: %d\n", total_steps);
    printf("  Error range during test: [%.4f nm, %.4f nm]\n", min_error_nm, max_error_nm);
    printf("  NaN recovery events:     %u\n", dynamics.get_nan_recovery_count());
    printf("  Integrator steps:        %llu\n",
           (unsigned long long)dynamics.get_step_counter());

    if (all_passed && !dynamics.has_nan()) {
        printf("\n  [PASS] ALL 5 CONSECUTIVE STEP SCANS STABLE!\n");
        printf("  [PASS] Sub-nanometer stability verified (|e| < %.1f nm)\n",
               params.settling_threshold_nm);
        printf("  [PASS] No NaN divergence in 10kHz physics loop\n");
    } else {
        printf("\n  [FAIL] Stability verification failed!\n");
    }

    csv.close();
    printf("\nData exported to settling_data_5step.csv\n");

    return (all_passed && !dynamics.has_nan()) ? 0 : 1;
}
