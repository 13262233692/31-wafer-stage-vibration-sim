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

    params.noise_enabled = true;
    params.noise_acoustic_amplitude_nm = 0.05;
    params.noise_acoustic_cutoff_hz = 150.0;
    params.noise_drift_amplitude_nm = 0.02;
    params.noise_drift_bandwidth_hz = 0.05;
    params.noise_seed = 42;

    dynamics.initialize(params);

    std::ofstream csv("settling_data_noise.csv");
    csv << "time_ms,position_mm,error_nm,noise_nm,phase,velocity_ms,nan_recovery\n";

    const char *phase_names[] = {"IDLE", "ACCEL", "DECEL", "SETTLE", "SETTLED"};

    printf("=== Wafer Stage Vibration - Noise Robustness Test ===\n");
    printf("Parameters:\n");
    printf("  Mass:              %.1f kg\n", params.mass);
    printf("  Active stiffness:  %.2e N/m\n", params.active_stiffness);
    printf("  Active damping:    %.1f N.s/m\n", params.active_damping);
    printf("  Active integral:   %.2e N/(m.s)\n", params.active_integral_gain);
    printf("  Simulation dt:     %.1e s (%.0f Hz)\n", params.simulation_dt, 1.0/params.simulation_dt);
    printf("  Derivative LPF:    %.0f Hz\n", params.derivative_filter_cutoff);
    printf("  --- Environment Noise ---\n");
    printf("  Noise enabled:     %s\n", params.noise_enabled ? "YES" : "NO");
    printf("  Acoustic amp:      %.3f nm (FFU turbulence)\n", params.noise_acoustic_amplitude_nm);
    printf("  Acoustic cutoff:   %.0f Hz (Kolmogorov)\n", params.noise_acoustic_cutoff_hz);
    printf("  Drift amp:         %.3f nm (refractive index)\n", params.noise_drift_amplitude_nm);
    printf("  Drift bandwidth:   %.3f Hz\n", params.noise_drift_bandwidth_hz);
    printf("  Settle threshold:  %.1f nm\n", params.settling_threshold_nm);
    printf("\n");

    double targets[] = {0.01, -0.005, 0.015, 0.0, 0.01};
    int num_steps = 5;
    double max_error_nm = 0.0;
    double min_error_nm = 0.0;
    double max_noise_nm = 0.0;
    double settle_times[5] = {0};
    bool all_passed = true;
    int total_steps = 0;

    for (int s = 0; s < num_steps; s++) {
        printf("\n--- Step %d: Target = %+.3f mm ---\n", s + 1, targets[s] * 1000.0);

        if (s == 0) {
            dynamics.reset();
        } else {
            dynamics.set_target_position(targets[s]);
        }

        bool achieved = false;
        double settle_time = 0.0;
        double settle_error = 0.0;
        double step_start_time = dynamics.get_sim_time();

        double post_settle_rms = 0.0;
        int post_settle_samples = 0;
        double post_settle_max = 0.0;

        for (int i = 0; i < 50000; i++) {
            dynamics.step();
            total_steps++;

            double t_ms = dynamics.get_sim_time() * 1000.0;
            double pos_mm = dynamics.get_position() * 1000.0;
            double err_nm = dynamics.get_position_error_nm();
            double noise_nm = dynamics.get_current_noise_nm();
            int phase = static_cast<int>(dynamics.get_phase());
            double vel = dynamics.get_velocity();
            uint32_t nan_rec = dynamics.get_nan_recovery_count();

            csv << std::fixed << std::setprecision(4) << t_ms << ","
                << std::setprecision(8) << pos_mm << ","
                << std::setprecision(4) << err_nm << ","
                << std::setprecision(6) << noise_nm << ","
                << phase << ","
                << std::setprecision(8) << vel << ","
                << nan_rec << "\n";

            if (err_nm > max_error_nm) max_error_nm = err_nm;
            if (err_nm < min_error_nm) min_error_nm = err_nm;
            if (std::abs(noise_nm) > max_noise_nm) max_noise_nm = std::abs(noise_nm);

            if (dynamics.is_settled() && !achieved) {
                achieved = true;
                settle_time = dynamics.get_settling_time() * 1000.0;
                settle_error = err_nm;
                settle_times[s] = settle_time;
                printf("  [OK] Settled at t_rel=%.3f ms, true error=%.4f nm\n",
                       settle_time, err_nm);
            }

            if (dynamics.is_settled()) {
                post_settle_rms += err_nm * err_nm;
                post_settle_samples++;
                if (std::abs(err_nm) > post_settle_max)
                    post_settle_max = std::abs(err_nm);
            }

            if (i % 10000 == 0) {
                printf("  t=%8.2fms  pos=%+9.5fmm  err=%+10.4fnm  noise=%+8.5fnm  phase=%-8s\n",
                       t_ms, pos_mm, err_nm, noise_nm,
                       phase < 5 ? phase_names[phase] : "???");
            }

            if (dynamics.has_nan()) {
                printf("  [WARN] NaN detected & recovered! Count: %u\n", dynamics.get_nan_recovery_count());
            }
        }

        if (!achieved) {
            all_passed = false;
            printf("  [FAIL] Not settled within 50000 steps (5.0s)\n");
        }

        double rms = post_settle_samples > 0 ? std::sqrt(post_settle_rms / post_settle_samples) : 0.0;
        printf("  Post-settle: RMS=%.4f nm, Peak=%.4f nm, Samples=%d\n",
               rms, post_settle_max, post_settle_samples);

        if (post_settle_max > params.settling_threshold_nm) {
            all_passed = false;
            printf("  [FAIL] Post-settle peak error exceeds threshold!\n");
        }
    }

    printf("\n==================== SUMMARY ====================\n");
    for (int s = 0; s < num_steps; s++) {
        printf("  Step %d: target=%+.3fmm  settle_time=%.3f ms\n",
               s + 1, targets[s] * 1000.0, settle_times[s]);
    }
    printf("  Total simulation steps:  %d\n", total_steps);
    printf("  True position error range: [%.4f nm, %.4f nm]\n", min_error_nm, max_error_nm);
    printf("  Peak interferometer noise: %.4f nm\n", max_noise_nm);
    printf("  NaN recovery events:       %u\n", dynamics.get_nan_recovery_count());
    printf("  Integrator steps:          %llu\n",
           (unsigned long long)dynamics.get_step_counter());

    if (all_passed && !dynamics.has_nan()) {
        printf("\n  [PASS] ALL 5 STEPS STABLE UNDER FFU TURBULENCE NOISE!\n");
        printf("  [PASS] Sub-nanometer stability maintained (|e| < %.1f nm)\n",
               params.settling_threshold_nm);
        printf("  [PASS] Controller rejects acoustic + refractive index disturbances\n");
    } else {
        printf("\n  [FAIL] Noise robustness verification failed!\n");
    }

    csv.close();
    printf("\nData exported to settling_data_noise.csv\n");

    return (all_passed && !dynamics.has_nan()) ? 0 : 1;
}
