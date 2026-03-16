#pragma once
#include <Eigen/Dense>


// Makale Eq.(3): RR(q) = { Ex + q : ||x||₂ ≤ 1 }
// E = diag(rx, ry, rz)
// Quadrotor için: rx = ry << rz  (downwash modeli)
struct RobotModel {
    // Ortam çakışması için robot hacmi (çevre engellere karşı)
    double radius;       // tek yarıçap (küre varsayımı — makale Sec. IV-A)

    // Robot-robot çakışması için ellipsoid yarıçapları (downwash)
    double rx, ry, rz;  // rx == ry << rz

    // ── Varsayılan: Crazyflie nano-quadrotor benzeri ──────────────────
    RobotModel()
        : radius(0.15)
        , rx(0.15), ry(0.15), rz(0.3) {}

    RobotModel(double radius, double rx, double ry, double rz)
        : radius(radius), rx(rx), ry(ry), rz(rz) {}

    // ── İki robot çakışıyor mu? (ellipsoid-ellipsoid) ─────────────────
    // Basit yöntem: normalize edilmiş mesafe < 1 ise çakışma var
    bool collides(const Eigen::Vector3d& qi,
                  const Eigen::Vector3d& qj) const
    {
        Eigen::Vector3d d = qi - qj;
        double val = (d.x()*d.x())/(rx*rx) +
                     (d.y()*d.y())/(ry*ry) +
                     (d.z()*d.z())/(rz*rz);
        return val < 1.0;   // < 1 → iç içe geçmiş
    }
};