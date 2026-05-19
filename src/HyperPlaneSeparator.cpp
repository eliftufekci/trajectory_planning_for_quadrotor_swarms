#include "HyperPlaneSeparator.hpp"
#include <iostream>
#include <algorithm>
#include <limits>
#include <Eigen/Sparse>
#include <OsqpEigen/Solver.hpp>

HyperPlaneSeparator::HyperPlaneSeparator(const Graph& graph, const RobotModel& robotModel, const SubdividedSchedule& subdividedSchedule, const Environment& environment)
    : graph(graph), subdividedSchedule(subdividedSchedule), robotModel(robotModel), environment(environment) {}

// Tüm schedule için tüm (i,j,k) hyperplane'lerini hesapla
SafePolyhedron HyperPlaneSeparator::compute(){
    const auto& positions = subdividedSchedule.positions;
    std::vector<std::vector<std::vector<HyperPlane>>> planes(positions.size()); // planes[agent][timestep_k]
    for(size_t i = 0; i < positions.size(); ++i) {
        planes[i].resize(subdividedSchedule.K);
    }

    // Her `(i, j, k)` üçlüsü için bir `(n, d)` çifti: robot `i` timestep `k`'da `n^T x >= d` tarafında, robot `j` `n^T x <= d` tarafında kalacak.
    // robot - robot
    for(int k=0; k < subdividedSchedule.K; k++){
        for(int i=0; i < positions.size(); i++){ // robot i
            for(int j=i+1; j< positions.size(); j++){ // robot j                    
                Eigen::MatrixXd segment_i(3, 2);
                segment_i << subdividedSchedule.positions[i][k], subdividedSchedule.positions[i][k+1];
                Eigen::MatrixXd segment_j(3, 2);
                segment_j << subdividedSchedule.positions[j][k], subdividedSchedule.positions[j][k+1];

                HyperPlane plane = computeSVM(segment_i, segment_j, false);
                plane.separated_from_type = 0; // Robot
                plane.separated_from_id = j;
                planes[i][k].push_back(plane);
                // Karşıt düzlemi de diğer ajan için ekle
                planes[j][k].push_back(HyperPlane(-plane.normal_vector, -plane.d, plane.ellipsoid_offset, 0, i));
            }
        }
    }
    // robot - obstacle
    for(int k = 0; k < subdividedSchedule.K; k++){
        for(int i = 0; i < positions.size(); i++){
            Eigen::Vector3d p_start = subdividedSchedule.positions[i][k];
            Eigen::Vector3d p_end   = subdividedSchedule.positions[i][k+1];
            
            // Segment'e yakın obstacle'ları octree'den bul
            auto nearby_obs = findNearbyObstacles(p_start, p_end, 1.0); // 1.0m margin
            
            for(size_t obs_idx : nearby_obs) {
                const auto& obstacle = environment.obstacles[obs_idx];

                Eigen::MatrixXd segment_i(3, 2);
                segment_i << subdividedSchedule.positions[i][k], subdividedSchedule.positions[i][k+1];
                
                Eigen::MatrixXd obstacle_matrix = getObstacleCorners(obstacle.min, obstacle.max);
                
                HyperPlane plane = computeSVM(segment_i, obstacle_matrix, true);
                plane.separated_from_type = 1; // Obstacle
                plane.separated_from_id = obs_idx;
                planes[i][k].push_back(plane);
            }
        }
    }
    // robot - environment boundaries
    for(int k=0; k < subdividedSchedule.K; k++){
        for(int i=0; i < positions.size(); i++){ // robot i
            // The robot must stay INSIDE the world bounds. This is defined by 6 planes.
            // The offset is based on the robot's radius for obstacle collision.
            double offset = robotModel.radius;
            const int obstacle_type = 1;
            const int env_boundary_id = -1;

            // x >= world_min.x  =>  1*x >= world_min.x
            planes[i][k].emplace_back(Eigen::Vector3d(1, 0, 0), environment.world_min.x(), offset, obstacle_type, env_boundary_id);
            // x <= world_max.x  => -1*x >= -world_max.x
            planes[i][k].emplace_back(Eigen::Vector3d(-1, 0, 0), -environment.world_max.x(), offset, obstacle_type, env_boundary_id);
            // y >= world_min.y  =>  1*y >= world_min.y
            planes[i][k].emplace_back(Eigen::Vector3d(0, 1, 0), environment.world_min.y(), offset, obstacle_type, env_boundary_id);
            // y <= world_max.y  => -1*y >= -world_max.y
            planes[i][k].emplace_back(Eigen::Vector3d(0, -1, 0), -environment.world_max.y(), offset, obstacle_type, env_boundary_id);
            // z >= world_min.z  =>  1*z >= world_min.z
            planes[i][k].emplace_back(Eigen::Vector3d(0, 0, 1), environment.world_min.z(), offset, obstacle_type, env_boundary_id);
            // z <= world_max.z  => -1*z >= -world_max.z
            planes[i][k].emplace_back(Eigen::Vector3d(0, 0, -1), -environment.world_max.z(), offset, obstacle_type, env_boundary_id);
        }
    }

    return SafePolyhedron(planes);
}

// iterative refinement için
SafePolyhedron HyperPlaneSeparator::compute(const std::vector<std::vector<std::vector<Eigen::Vector3d>>>& samples) {
    const size_t num_agents = samples.size();
    if (num_agents == 0) {
        return SafePolyhedron({});
    }
    const size_t K = samples[0].size();
    const size_t S = (K > 0 && !samples[0].empty() && !samples[0][0].empty()) ? samples[0][0].size() : 0;

    std::vector<std::vector<std::vector<HyperPlane>>> planes(num_agents);
    for(size_t i = 0; i < num_agents; ++i) {
        planes[i].resize(K);
    }

    // Robot-robot separation from samples
    for(size_t k = 0; k < K; ++k) {
        for(size_t i = 0; i < num_agents; ++i) {
            for(size_t j = i + 1; j < num_agents; ++j) {
                Eigen::MatrixXd points_i(3, S);
                for(size_t s = 0; s < S; ++s) {
                    points_i.col(s) = samples[i][k][s];
                }

                Eigen::MatrixXd points_j(3, S);
                for(size_t s = 0; s < S; ++s) {
                    points_j.col(s) = samples[j][k][s];
                }

                HyperPlane plane = computeSVM(points_i, points_j, false);
                plane.separated_from_type = 0; // Robot
                plane.separated_from_id = j;
                planes[i][k].push_back(plane);
                
                planes[j][k].push_back(HyperPlane(-plane.normal_vector, -plane.d, plane.ellipsoid_offset, 0, i));
            }
        }
    }

    // Robot-obstacle separation from samples
    for(size_t k = 0; k < K; ++k) {
        for(size_t i = 0; i < num_agents; ++i) {
            if (S == 0) continue; // Örnek nokta yoksa bu adımı atla.

            // Yörünge parçasını (sample'ları) içeren sınırlayıcı kutuyu (AABB) hesapla.
            Eigen::Vector3d samples_min = samples[i][k][0];
            Eigen::Vector3d samples_max = samples[i][k][0];
            for(size_t s = 1; s < S; ++s) {
                samples_min = samples_min.cwiseMin(samples[i][k][s]);
                samples_max = samples_max.cwiseMax(samples[i][k][s]);
            }

            // Octree kullanarak bu kutuya yakın olan engelleri verimli bir şekilde bul.
            auto nearby_obs = findNearbyObstacles(samples_min, samples_max, 1.0); // 1.0m margin

            for(size_t obs_idx : nearby_obs) {
                const auto& obstacle = environment.obstacles.at(obs_idx);

                Eigen::MatrixXd points_i(3, S);
                for(size_t s = 0; s < S; ++s) {
                    points_i.col(s) = samples[i][k][s];
                }

                Eigen::MatrixXd obstacle_matrix = getObstacleCorners(obstacle.min, obstacle.max);

                HyperPlane plane = computeSVM(points_i, obstacle_matrix, true);
                plane.separated_from_type = 1; // Obstacle
                plane.separated_from_id = obs_idx;
                planes[i][k].push_back(plane);
            }
        }
    }

    // robot - environment boundaries
    for(size_t k = 0; k < K; ++k) {
        for(size_t i = 0; i < num_agents; ++i) {
            // The robot must stay INSIDE the world bounds. This is defined by 6 planes.
            // We don't use SVM here; the planes are fixed and don't depend on samples.
            // The offset is based on the robot's radius for obstacle collision.
            double offset = robotModel.radius;
            const int obstacle_type = 1;
            const int env_boundary_id = -1;

            // x >= world_min.x  =>  1*x >= world_min.x
            planes[i][k].emplace_back(Eigen::Vector3d(1, 0, 0), environment.world_min.x(), offset, obstacle_type, env_boundary_id);
            // x <= world_max.x  => -1*x >= -world_max.x
            planes[i][k].emplace_back(Eigen::Vector3d(-1, 0, 0), -environment.world_max.x(), offset, obstacle_type, env_boundary_id);
            // y >= world_min.y  =>  1*y >= world_min.y
            planes[i][k].emplace_back(Eigen::Vector3d(0, 1, 0), environment.world_min.y(), offset, obstacle_type, env_boundary_id);
            // y <= world_max.y  => -1*y >= -world_max.y
            planes[i][k].emplace_back(Eigen::Vector3d(0, -1, 0), -environment.world_max.y(), offset, obstacle_type, env_boundary_id);
            // z >= world_min.z  =>  1*z >= world_min.z
            planes[i][k].emplace_back(Eigen::Vector3d(0, 0, 1), environment.world_min.z(), offset, obstacle_type, env_boundary_id);
            // z <= world_max.z  => -1*z >= -world_max.z
            planes[i][k].emplace_back(Eigen::Vector3d(0, 0, -1), -environment.world_max.z(), offset, obstacle_type, env_boundary_id);
        }
    }

    return SafePolyhedron(planes);
}

// İki nokta kümesini (burada iki segmentin uç noktaları) ayıran
// maksimum marjlı hiperdüzlemi bulan SVM (QP) çözücü.    
HyperPlane HyperPlaneSeparator::computeSVM(const Eigen::MatrixXd& points_i, const Eigen::MatrixXd& points_j, bool is_obstacle){
    const int m_i = points_i.cols();
    const int m_j = points_j.cols();
    const int n_vars = 4; // [nx, ny, nz, d]
    const int n_constraints = m_i + m_j;

    // --- 1. Hessian Matrisi (P): minimize 0.5 * x'Px ---
    Eigen::SparseMatrix<double> P(n_vars, n_vars);
    if(is_obstacle){
        // Engel durumunda, robotun yarıçapını kullanarak marjı maksimize et
        P.insert(0, 0) = robotModel.radius * robotModel.radius;
        P.insert(1, 1) = robotModel.radius * robotModel.radius;
        P.insert(2, 2) = robotModel.radius * robotModel.radius;
    } else {
        // Robot-robot durumunda, elipsoid yarıçaplarını kullan
        P.insert(0, 0) = robotModel.rx * robotModel.rx;
        P.insert(1, 1) = robotModel.ry * robotModel.ry;
        P.insert(2, 2) = robotModel.rz * robotModel.rz;
    }
    P.insert(3, 3) = 1e-6;

    // --- 2. Gradyan Vektörü (q): q'x terimi yok ---
    Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);

    // --- 3. Kısıt Matrisi (A): l <= Ax <= u ---
    Eigen::MatrixXd A_dense(n_constraints, n_vars);
    // points_i için kısıtlar: n'.p_i - d >= 1
    for (int i = 0; i < m_i; ++i) {
        A_dense.row(i) << points_i.col(i).transpose(), -1;
    }
    // points_j için kısıtlar: n'.p_j - d <= -1
    for (int j = 0; j < m_j; ++j) {
        A_dense.row(m_i + j) << points_j.col(j).transpose(), -1;
    }
    Eigen::SparseMatrix<double> A = A_dense.sparseView();

    // --- 4. Kısıt Sınırları (l ve u) ---
    Eigen::VectorXd l(n_constraints), u(n_constraints);
    const double inf = std::numeric_limits<double>::infinity();
    
    // points_i için: l=1, u=inf
    l.head(m_i).fill(1.0);
    u.head(m_i).fill(inf);

    // points_j için: l=-inf, u=-1
    l.tail(m_j).fill(-inf);
    u.tail(m_j).fill(-1.0);

    // --- 5. Solver'ı kur ve çöz ---
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);

    if (!solver.data()->setHessianMatrix(P) || !solver.data()->setGradient(q) ||
        !solver.data()->setLinearConstraintsMatrix(A) ||
        !solver.data()->setLowerBound(l) || !solver.data()->setUpperBound(u)) {
        throw std::runtime_error("SVM QP setup failed");
    }
                            
    if (!solver.initSolver()) throw std::runtime_error("SVM QP init failed");
    solver.solveProblem();

    // --- 6. Sonucu işle ---
    Eigen::VectorXd solution = solver.getSolution();
    Eigen::Vector3d normal = solution.head<3>();
    double d_val = solution(3);
    
    double norm_val = normal.norm();
    if (norm_val < 1e-6) { // If SVM failed to find a good normal (e.g., sets are not separable with margin)
        std::cout << "SVM failed to find a good normal";
        Eigen::Vector3d centroid_diff = points_i.rowwise().mean() - points_j.rowwise().mean();
        if (centroid_diff.norm() < 1e-6) { // Centroids are too close or identical
            // Fallback to an arbitrary non-zero normal if centroids are also too close
            normal = Eigen::Vector3d(1.0, 0.0, 0.0);
        } else {
            normal = centroid_diff.normalized();
        }
        d_val = normal.dot((points_i.rowwise().mean() + points_j.rowwise().mean()) * 0.5);
        norm_val = 1.0; // Treat the fallback normal as unit length for consistency
    }

    double ellipsoid_offset_val;
    if (is_obstacle) {
        ellipsoid_offset_val = robotModel.radius; // For obstacle, robot is treated as a sphere
    } else {
        ellipsoid_offset_val = calculate_offset(normal / norm_val); // For robot-robot, use ellipsoid model
    }

    return HyperPlane(normal / norm_val, d_val / norm_val, ellipsoid_offset_val);
}

std::vector<size_t> HyperPlaneSeparator::findNearbyObstacles(const Eigen::Vector3d& p_start, const Eigen::Vector3d& p_end, double margin) {
    // 1. Segment'in bounding box'ını hesapla + margin ekle
    Eigen::Vector3d box_min_v = p_start.cwiseMin(p_end).array() - margin;
    Eigen::Vector3d box_max_v = p_start.cwiseMax(p_end).array() + margin;
    octomap::point3d bbx_min(box_min_v.x(), box_min_v.y(), box_min_v.z());
    octomap::point3d bbx_max(box_max_v.x(), box_max_v.y(), box_max_v.z());

    std::vector<size_t> result;

    // 2. Octree'de bu box ile kesişen occupied node'ları sorgula
    for(auto it = environment.tree->begin_leafs_bbx(bbx_min, bbx_max); it != environment.tree->end_leafs_bbx(); ++it){
        if (!environment.tree->isNodeOccupied(*it)) continue;
        
        Eigen::Vector3d node_center(it.getX(), it.getY(), it.getZ());
        for (size_t obs_idx = 0; obs_idx < environment.obstacles.size(); ++obs_idx) {
            if (environment.obstacles[obs_idx].contains(node_center)) {
                result.push_back(obs_idx);
                // Bir düğüm merkezinin yalnızca bir engele ait olabileceğini
                // varsayarak döngüden çıkıyoruz. Bu, verimliliği artırır.
                break;
            }
        }
    }

    // Duplicate index'leri temizle
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());

    return result;
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d> HyperPlaneSeparator::find_closest_points(const Eigen::MatrixXd& s1, const Eigen::MatrixXd& s2) {
    Eigen::Vector3d p1 = s1.col(0);
    Eigen::Vector3d p2 = s1.col(1);
    Eigen::Vector3d p3 = s2.col(0);
    Eigen::Vector3d p4 = s2.col(1);
    
    Eigen::Vector3d u = p2 - p1;
    Eigen::Vector3d v = p4 - p3;
    Eigen::Vector3d w = p1 - p3;

    double a = u.squaredNorm();
    double b = u.dot(v);
    double c = v.squaredNorm();
    double d = u.dot(w);
    double e = v.dot(w);

    double denom = a*c - b*b; // paralel mi
    double s = 0.0, t = 0.0;

    if (a < 1e-8 && c < 1e-8) {
        s = 0.0;
        t = 0.0;
    } else if (a < 1e-8) {
        s = 0.0;
        t = std::clamp(e / c, 0.0, 1.0);
    } else if (c < 1e-8) {
        t = 0.0;
        s = std::clamp(-d / a, 0.0, 1.0);
    } else {
        if (denom > 1e-8) {
            s = std::clamp((b*e - c*d) / denom, 0.0, 1.0);
        } else {
            s = 0.0; // Paralellerse s'i sabitleyip t'yi buluyoruz
        }

        t = (b*s + e) / c;

        if (t < 0.0) {
            t = 0.0;
            s = std::clamp(-d / a, 0.0, 1.0);
        } else if (t > 1.0) {
            t = 1.0;
            s = std::clamp((b - d) / a, 0.0, 1.0);
        }
    }

    Eigen::Vector3d point1 = p1 + s*u;
    Eigen::Vector3d point2 = p3 + t*v;

    return {point1, point2};
}

bool HyperPlaneSeparator::sweptVolumesCollide(const Eigen::MatrixXd& segment_i, const Eigen::MatrixXd& segment_j) {
    auto [p_i, p_j] = find_closest_points(segment_i, segment_j);
    return robotModel.collides(p_i, p_j);
}

Eigen::MatrixXd HyperPlaneSeparator::getObstacleCorners(Eigen::Vector3d min, Eigen::Vector3d max){
    Eigen::MatrixXd corners(3, 8);
    
    corners.col(0) << min.x(), min.y(), min.z();
    corners.col(1) << max.x(), min.y(), min.z();
    corners.col(2) << min.x(), max.y(), min.z();
    corners.col(3) << max.x(), max.y(), min.z();
    
    corners.col(4) << min.x(), min.y(), max.z();
    corners.col(5) << max.x(), min.y(), max.z();
    corners.col(6) << min.x(), max.y(), max.z();
    corners.col(7) << max.x(), max.y(), max.z();
    
    return corners;
}

double HyperPlaneSeparator::calculate_offset(Eigen::Vector3d normal_vector){
    Eigen::Vector3d E_n(robotModel.rx * normal_vector.x(),
                    robotModel.ry * normal_vector.y(),
                    robotModel.rz * normal_vector.z());
    return E_n.norm();
}
