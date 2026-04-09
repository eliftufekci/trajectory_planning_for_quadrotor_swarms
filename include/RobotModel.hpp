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
        , rx(0.12), ry(0.12), rz(0.3) {}

    RobotModel(double radius, double rx, double ry, double rz)
        : radius(radius), rx(rx), ry(ry), rz(rz) {}

    // ── İki robot çakışıyor mu? (ellipsoid-ellipsoid) ─────────────────
    // Basit yöntem: normalize edilmiş mesafe < 1 ise çakışma var
    bool collides(const Eigen::Vector3d& qi,
                  const Eigen::Vector3d& qj) const
    {
        Eigen::Vector3d d = qi - qj;
        // İki özdeş elipsoidin çarpışma kontrolü, Minkowski toplamı konsepti kullanılarak yapılır.
        // İki elipsoid, merkezlerini birleştiren vektör (d), yarı eksenleri orijinal
        // elipsoidin iki katı olan (2*rx, 2*ry, 2*rz) daha büyük bir elipsoidin
        // içine düşerse çarpışır.
        // Bu durumun matematiksel ifadesi:
        // (d.x^2 / (4*rx^2)) + (d.y^2 / (4*ry^2)) + (d.z^2 / (4*rz^2)) < 1
        // standart elipsiot denklemi
        // x^2/a^2 + y^2/b^2 + z^2/c^2 = 1
        double val = (d.x()*d.x())/(4*rx*rx) +
                     (d.y()*d.y())/(4*ry*ry) +
                     (d.z()*d.z())/(4*rz*rz);
        return val < 1.0;
    }
};