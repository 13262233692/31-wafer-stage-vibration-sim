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
    params.simulation_dt = 1e-5;
    params.settling_threshold_nm = 0.1;
    params.max_force = 50000.0;

    dynamics.initialize(params);

    std::ofstream csv("settling_data.csv");
    csv << "time_ms,position_mm,error_nm,phase,velocity_ms\n";

    int total_steps = 300000;
    int report_interval = 100;

    printf("=== Wafer Stage Vibration Simulation - Standalone Verification ===\n");
    printf("Parameters:\n");
    printf("  Mass:              %.1f kg\n", params.mass);
    printf("  Lorentz K_f:       %.1f N/A\n", params.lorentz_force_constant);
    printf("  Spring K_s:        %.2e N/m\n", params.spring_stiffness);
    printf("  Passive damping:   %.1f N.s/m\n", params.passive_damping);
    printf("  Active stiffness:  %.2e N/m\n", params.active_stiffness);
    printf("  Active damping:    %.1f N.s/m\n", params.active_damping);
    printf("  Active integral:   %.2e N/(m.s)\n", params.active_integral_gain);
    printf("  Target position:   %.3f mm\n", params.target_position * 1000.0);
    printf("  Acceleration:      %.1fG (%.2f m/s^2)\n", params.accel_command / wafer_stage::StageDynamics::GRAVITY, params.accel_command);
    printf("  Sim frequency:     %.0f Hz (dt=%.1e s)\n", 1.0/params.simulation_dt, params.simulation_dt);
    printf("  Settle threshold:  %.1f nm\n", params.settling_threshold_nm);
    printf("  Max force:         %.0f N\n", params.max_force);
    printf("\n");

    const char *phase_names[] = {"IDLE", "ACCEL", "DECEL", "SETTLE", "SETTLED"};

    for (int i = 0; i < total_steps; i++) {
        dynamics.step();

        if (i % report_interval == 0) {
            double t_ms = dynamics.get_sim_time() * 1000.0;
            double pos_mm = dynamics.get_position() * 1000.0;
            double err_nm = dynamics.get_position_error_nm();
            int phase = static_cast<int>(dynamics.get_phase());
            double vel = dynamics.get_velocity();

            csv << std::fixed << std::setprecision(4) << t_ms << ","
                << std::setprecision(8) << pos_mm << ","
                << std::setprecision(4) << err_nm << ","
                << phase << ","
                << std::setprecision(8) << vel << "\n";

            if (i % 5000 == 0) {
                printf("t=%7.2fms  pos=%9.5fmm  err=%+12.4fnm  v=%+9.5fm/s  phase=%s\n",
                       t_ms, pos_mm, err_nm, vel,
                       phase < 5 ? phase_names[phase] : "???");
            }
        }

        if (dynamics.is_settled()) {
            double t_ms = dynamics.get_sim_time() * 1000.0;
            double pos_mm = dynamics.get_position() * 1000.0;
            double err_nm = dynamics.get_position_error_nm();
            double vel = dynamics.get_velocity();

            printf("\n=== SETTLING ACHIEVED ===\n");
            printf("  Settling time:   %.3f ms\n", dynamics.get_settling_time() * 1000.0);
            printf("  Final position:  %.8f mm\n", pos_mm);
            printf("  Final error:     %.4f nm\n", err_nm);
            printf("  Final velocity:  %.8f m/s\n", vel);
            printf("  Target:          %.5f mm\n", params.target_position * 1000.0);

            if (std::abs(err_nm) < params.settling_threshold_nm) {
                printf("\n  [PASS] SUB-NANOMETER STABILITY VERIFIED (|error| < %.1f nm)\n", params.settling_threshold_nm);
            } else {
                printf("\n  [FAIL] |error| = %.4f nm >= %.1f nm\n", std::abs(err_nm), params.settling_threshold_nm);
            }

            for (int j = 0; j < 10000; j++) {
                dynamics.step();
                if (j % 100 == 0) {
                    t_ms = dynamics.get_sim_time() * 1000.0;
                    pos_mm = dynamics.get_position() * 1000.0;
                    err_nm = dynamics.get_position_error_nm();
                    vel = dynamics.get_velocity();
                    csv << std::fixed << std::setprecision(4) << t_ms << ","
                        << std::setprecision(8) << pos_mm << ","
                        << std::setprecision(4) << err_nm << ","
                        << 4 << ","
                        << std::setprecision(8) << vel << "\n";
                }
            }

            printf("  Post-settle stability (%.1f ms after):\n", dynamics.get_sim_time() * 1000.0 - dynamics.get_settling_time() * 1000.0);
            printf("    Current error: %.4f nm\n", dynamics.get_position_error_nm());
            break;
        }
    }

    if (!dynamics.is_settled()) {
        printf("\n  [FAIL] SETTLING NOT ACHIEVED within simulation time\n");
        printf("  Final error: %.4f nm\n", dynamics.get_position_error_nm());
    }

    csv.close();
    printf("\nData exported to settling_data.csv\n");

    return dynamics.is_settled() ? 0 : 1;
}
