#include "RobotModel.hpp"

RobotModel::RobotModel()
    : radius(0.15)
    , rx(0.12), ry(0.12), rz(0.3) {}

RobotModel::RobotModel(double radius, double rx, double ry, double rz)
    : radius(radius), rx(rx), ry(ry), rz(rz) {}

bool RobotModel::collides(const Eigen::Vector3d& qi, const Eigen::Vector3d& qj) const {
    Eigen::Vector3d d = qi - qj;
    // İki özdeş elipsoidin çarpışma kontrolü, Minkowski toplamı konsepti kullanılarak yapılır.
    // İki elipsoid, merkezlerini birleştiren vektör (d), yarı eksenleri orijinal
    // elipsoidin iki katı olan (2*rx, 2*ry, 2*rz) daha büyük bir elipsoidin
    // içine düşerse çarpışır.
    // standart elipsiot denklemi: x^2/a^2 + y^2/b^2 + z^2/c^2 = 1
    double val = (d.x() * d.x()) / (4 * rx * rx) +
                 (d.y() * d.y()) / (4 * ry * ry) +
                 (d.z() * d.z()) / (4 * rz * rz);
    return val < 1.0;
}