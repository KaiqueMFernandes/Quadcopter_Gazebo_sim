#ifndef PHYSICS_ENGINE_HPP
#define PHYSICS_ENGINE_HPP

#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include "state.hpp"

namespace modular_robot {

class PhysicsCore {
public:
    // Physical properties for a typical quadcopter
    Eigen::Vector3d gravity{0.0, 0.0, -9.81};
    double mass = 1.0; // kg
    double arm_length = 0.25; // m (distance from center to motor)
    double motor_thrust_coeff = 1.91e-6; // N/(rad/s)^2 (example value)
    double motor_torque_coeff = 2.6e-7; // Nm/(rad/s)^2 (example value)
    Eigen::Matrix3d inertia_tensor = []{
        Eigen::Matrix3d I = Eigen::Matrix3d::Zero();
        I(0,0) = 0.02; // Ixx
        I(1,1) = 0.02; // Iyy
        I(2,2) = 0.04; // Izz
        return I;
    }();

    // External forces and moments
    Eigen::Vector3d ext_force{0.0, 0.0, 0.0};
    Eigen::Vector3d ext_moment{0.0, 0.0, 0.0};

    // World-frame viscous drag: F = -linear_drag_coeff * v. With no drag, a leveled
    // quad has no horizontal thrust component, so linear velocity never decays.
    double linear_drag_coeff = 1.2; // N·s/m (tune for how quickly coasting stops)

    Eigen::Vector3d computeLinearAcceleration(const RobotState& state) {
        // ext_force is in body frame, rotate to world frame
        Eigen::Vector3d force_world = state.orientation * ext_force;
        Eigen::Vector3d acc_vec = (force_world / mass) + gravity;
        if (linear_drag_coeff > 0.0) {
            acc_vec -= (linear_drag_coeff / mass) * state.lin_vel;
        }
        return acc_vec;
    }

    Eigen::Vector3d computeAngAcceleration(const RobotState& state) {
        Eigen::Vector3d omega = state.ang_vel;
        Eigen::Vector3d gyro = omega.cross(inertia_tensor * omega);
        Eigen::Vector3d ang_acc = inertia_tensor.inverse() * (ext_moment - gyro);
        return ang_acc;
    }

    Eigen::Quaterniond computeQuaternionDot(const RobotState& state) {
        Eigen::Quaterniond q = state.orientation;
        Eigen::Vector3d w = state.ang_vel;
        Eigen::Quaterniond wq(0, w.x(), w.y(), w.z());
        Eigen::Quaterniond q_dot = q * wq;
        q_dot.coeffs() *= 0.5;
        return q_dot;
    }
};
}

#endif