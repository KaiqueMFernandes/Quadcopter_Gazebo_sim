#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP

#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace modular_robot {
/**
 * @brief Represents the 13-dimensional state of the robot in 3D space.
 * State vector: [px, py, pz, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]
 */

struct RobotState {

    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d lin_vel = Eigen::Vector3d::Zero(); // world frame
    Eigen::Vector3d ang_vel = Eigen::Vector3d::Zero(); // body frame

    void normalize() {
        orientation.normalize();
    }
    
    // Helper to serialize for the integrator
    using Vector13d = Eigen::Matrix<double, 13, 1>;
    
    Vector13d toVector() const {
        Vector13d v;
        v << position, orientation.w(), orientation.x(), orientation.y(), orientation.z(), lin_vel, ang_vel;
        return v;
    }

    void resetState(){
        position = Eigen::Vector3d::Zero();
        orientation = Eigen::Quaterniond::Identity();
        lin_vel = Eigen::Vector3d::Zero(); // world frame
        ang_vel = Eigen::Vector3d::Zero(); // body frame 

        toVector();
    }

    void fromVector(const Vector13d& v) {
        position    = v.segment(0, 3); 
        
        // Manual mapping for the quaternion (w, x, y, z)
        orientation = Eigen::Quaterniond(v(3), v(4), v(5), v(6)).normalized();
        
        lin_vel     = v.segment(7, 3);
        ang_vel     = v.segment(10, 3);
    } 
};
}

#endif