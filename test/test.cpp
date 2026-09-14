#include <iostream>
#include <cassert>
#include "state.hpp"
#include "integrator.hpp"
#include "engine.hpp"

using namespace modular_robot;

int main() {
    // 1. Initialize State
    RobotState initial_state;
    initial_state.position << 0.0, 0.0, 0.0;
    initial_state.orientation = Eigen::Quaterniond::Identity();
    initial_state.lin_vel << 1.0, 0.0, 0.0; // Moving at 1m/s in X
    initial_state.ang_vel << 0.0, 0.0, 0.1; // Rotating slightly

    PhysicsCore core;

    // 2. Define derivative function
    auto robot_physics = [&](const modular_robot::RobotState::Vector13d& v, double /*t*/) {
        RobotState state;
        state.fromVector(v);
        
        modular_robot::RobotState::Vector13d derivative = modular_robot::RobotState::Vector13d::Zero();
        
        derivative.segment(0, 3) = state.lin_vel;

        Eigen::Quaterniond q_dot = core.computeQuaternionDot(state);
        derivative(3) = q_dot.w();
        derivative(4) = q_dot.x();
        derivative(5) = q_dot.y();
        derivative(6) = q_dot.z();

        derivative.segment(7, 3) = core.computeLinearAcceleration(state);
        derivative.segment(10, 3) = core.computeAngAcceleration(state);
        
        return derivative;
    }; // Added semicolon

    // 3. Run Integrator
    double dt = 0.1;
    RobotState current_state = initial_state;
    
    std::cout << "Starting Simulation..." << std::endl;

    for(int i = 0; i < 10; ++i) {
        current_state = modular_robot::RK4Integrator<modular_robot::RobotState, modular_robot::RobotState::Vector13d, decltype(robot_physics)>::integrate(
            current_state, 
            dt, 
            robot_physics);
    }

    // 4. Verification
    std::cout << "--- Results after 1.0s ---" << std::endl;
    std::cout << "Position:    " << current_state.position.transpose() << std::endl;
    std::cout << "Orientation (w,x,y,z): " 
              << current_state.orientation.w() << " " 
              << current_state.orientation.x() << " " 
              << current_state.orientation.y() << " " 
              << current_state.orientation.z() << std::endl;
    
    return 0;
}