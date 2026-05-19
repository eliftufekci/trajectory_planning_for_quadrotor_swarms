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
    RobotModel();
    RobotModel(double radius, double rx, double ry, double rz);

    // ── İki robot çakışıyor mu? (ellipsoid-ellipsoid) ─────────────────
    // Basit yöntem: normalize edilmiş mesafe < 1 ise çakışma var
    bool collides(const Eigen::Vector3d& qi, const Eigen::Vector3d& qj) const;
};