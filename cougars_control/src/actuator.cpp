#include <Eigen/Dense>
#include <iostream>
#include <cmath>

class Fin {
public:
    Fin(double a, double CL, double x, double c = 0, double angle = 0, double rho = 1026)
        : area(a), CL(CL), angle_rad(angle * M_PI / 180.0), rho(rho),
          u_actual_fin(0.0), T_delta(0.1), deltaMax(15 * M_PI / 180.0) {
        
        double y = std::cos(angle_rad) * c;
        double z = std::sin(angle_rad) * c;
        R << x, y, z;
    }

    double velocity_in_rotated_plane(const Eigen::Vector3d& nu) const {
        double vx = nu[0], vy = nu[1], vz = nu[2];
        double vy_rot = std::sqrt(std::pow(vy * std::sin(angle_rad), 2) + std::pow(vz * std::cos(angle_rad), 2));
        return std::sqrt(vx * vx + vy_rot * vy_rot);
    }

    Eigen::VectorXd tau(const Eigen::VectorXd& nu) {
        double ur = velocity_in_rotated_plane(nu.head<3>());
        double f = 0.5 * rho * area * CL * u_actual_fin * ur * ur;
        double fy = std::sin(angle_rad) * f;
        double fz = -std::cos(angle_rad) * f;

        Eigen::Vector3d F(0, fy, fz);
        Eigen::Vector3d torque = R.cross(F);

        Eigen::VectorXd result(6);
        result << F, torque;
        return result;
    }

    double actuate(double sampleTime, double command) {
        double delta_dot = (command - u_actual_fin) / T_delta;
        u_actual_fin += sampleTime * delta_dot;

        if (std::abs(u_actual_fin) >= deltaMax) {
            u_actual_fin = std::copysign(deltaMax, u_actual_fin);
        }

        return u_actual_fin;
    }

    double calculate_deflection(const Eigen::Vector3d& desired_torque, const Eigen::VectorXd& nu) {
        double ur = velocity_in_rotated_plane(nu.head<3>());
        double force = 0;

        if (desired_torque[1] != 0 && desired_torque[2] == 0) {
            force = -desired_torque[1] / (R[0] * -std::cos(angle_rad));
        } else if (desired_torque[2] != 0 && desired_torque[1] == 0) {
            force = desired_torque[2] / (R[0] * std::sin(angle_rad));
        } else if (desired_torque[1] == 0 && desired_torque[2] == 0) {
            force = 0;
        } else {
            std::cerr << "Error: Only one of Ty or Tz should be non-zero" << std::endl;
            return 0;
        }

        double den = 0.5 * rho * area * CL * ur * ur;
        double required_deflection = (den == 0) ? force / den : 0;

        return std::clamp(required_deflection, -deltaMax, deltaMax);
    }

private:
    double area;
    double CL;
    double angle_rad;
    double rho;
    double u_actual_fin;
    double T_delta;
    double deltaMax;
    Eigen::Vector3d R;
};
