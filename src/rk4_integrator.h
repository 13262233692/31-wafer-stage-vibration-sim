#pragma once

#include <cmath>
#include <functional>

namespace wafer_stage {

struct StageState {
    double x;
    double v;

    StageState() : x(0.0), v(0.0) {}
    StageState(double px, double pv) : x(px), v(pv) {}

    StageState operator+(const StageState &other) const {
        return StageState(x + other.x, v + other.v);
    }

    StageState operator-(const StageState &other) const {
        return StageState(x - other.x, v - other.v);
    }

    StageState operator*(double scalar) const {
        return StageState(x * scalar, v * scalar);
    }

    StageState operator/(double scalar) const {
        return StageState(x / scalar, v / scalar);
    }

    friend StageState operator*(double scalar, const StageState &s) {
        return s * scalar;
    }
};

struct StageDerivative {
    double dx;
    double dv;

    StageDerivative() : dx(0.0), dv(0.0) {}
    StageDerivative(double pdx, double pdv) : dx(pdx), dv(pdv) {}

    StageDerivative operator+(const StageDerivative &other) const {
        return StageDerivative(dx + other.dx, dv + other.dv);
    }

    StageDerivative operator*(double scalar) const {
        return StageDerivative(dx * scalar, dv * scalar);
    }

    friend StageDerivative operator*(double scalar, const StageDerivative &d) {
        return d * scalar;
    }
};

class RK4Integrator {
public:
    using DynamicsFunc = std::function<StageDerivative(const StageState &, double)>;

    static StageState step(const StageState &state, double t, double dt, const DynamicsFunc &dynamics) {
        StageDerivative k1 = dynamics(state, t);

        StageState s2 = state + (dt * 0.5) * StageState(k1.dx, k1.dv);
        StageDerivative k2 = dynamics(s2, t + 0.5 * dt);

        StageState s3 = state + (dt * 0.5) * StageState(k2.dx, k2.dv);
        StageDerivative k3 = dynamics(s3, t + 0.5 * dt);

        StageState s4 = state + dt * StageState(k3.dx, k3.dv);
        StageDerivative k4 = dynamics(s4, t + dt);

        double dx = (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx) / 6.0;
        double dv = (k1.dv + 2.0 * k2.dv + 2.0 * k3.dv + k4.dv) / 6.0;

        StageState result;
        result.x = state.x + dt * dx;
        result.v = state.v + dt * dv;
        return result;
    }

    static StageState integrate(StageState state, double t_start, double t_end, double dt, const DynamicsFunc &dynamics) {
        double t = t_start;
        while (t < t_end) {
            double actual_dt = std::min(dt, t_end - t);
            state = step(state, t, actual_dt, dynamics);
            t += actual_dt;
        }
        return state;
    }
};

}
