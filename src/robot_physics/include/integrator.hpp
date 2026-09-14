#ifndef ENGINE_HPP
#define ENGINE_HPP

//#pragma once
#include <functional>

namespace modular_robot {
template<typename State, typename Derivative, typename Function>
class RK4Integrator {
public:
    /**
     * @brief Performs one RK4 step.
     * @param y The current state vector.
     * @param dt The time step.
     * @param f The ODE function.
     * @return The integrated state vector.
     */

    static State integrate(
        const State& y,
        double dt,
        const Function& func) {

        typename State::Vector13d y_vec = y.toVector();

        auto k1 = func(y_vec, 0);
        auto k2 = func(y_vec + 0.5 * dt * k1, 0.5 * dt);
        auto k3 = func(y_vec + 0.5 * dt * k2, 0.5 * dt);
        auto k4 = func(y_vec + dt * k3, dt);

        typename State::Vector13d next_y_vec = y_vec + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        
        State next_state;
        next_state.fromVector(next_y_vec);
        
        next_state.normalize();

        return next_state;

        }

    };

};

#endif