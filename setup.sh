OSQP_EIGEN_DIR="/home/wwweliftufekci/trajectory_planning_for_quadrotor_swarms/libs/osqp-eigen"
echo "Cloning osqp-eigen..."
git clone --depth 1 https://github.com/robotology/osqp-eigen.git "$OSQP_EIGEN_DIR"

LIB_DIR="/home/wwweliftufekci/trajectory_planning_for_quadrotor_swarms/libs/libMultiRobotPlanning"
echo "Cloning libMultiRobotPlanning..."
git clone --depth 1 https://github.com/whoenig/libMultiRobotPlanning.git "$LIB_DIR"

echo "Cleaning up libMultiRobotPlanning..."
rm -rf "$LIB_DIR/benchmark"
rm -rf "$LIB_DIR/doc"
rm -rf "$LIB_DIR/examples"
rm -rf "$LIB_DIR/test"s
rm -rf "$LIB_DIR/.github"
rm -f  "$LIB_DIR/CMakeLists.txt" "$LIB_DIR/.clang-format" "$LIB_DIR/.gitignore"

sudo apt-get update
# libosqp-dev paketi eski Ubuntu dağıtımlarında (örn. 18.04) bulunmayabilir.
# Bu yüzden OSQP'yi kaynaktan derlemek daha güvenilir bir yöntemdir.
sudo apt-get install -y liboctomap-dev libeigen3-dev libyaml-cpp-dev libcdd-dev libfcl-dev libompl-dev libboost-all-dev

# --- OSQP'yi kaynaktan derle ve kur ---
(
    echo "Building and installing OSQP from source..."
    cd /tmp
    git clone --recursive https://github.com/osqp/osqp.git
    cd osqp
    mkdir build && cd build
    cmake -G "Unix Makefiles" -DBUILD_SHARED_LIBS=ON ..
    make -j$(nproc)
    sudo make install
)


(
    START=$(date)
    ./env_test  "/home/wwweliftufekci/trajectory_planning_for_quadrotor_swarms/config/environment8.yaml"
    END=$(date)
    sec1=$(date -d "$START" +%s)
    sec2=$(date -d "$END" +%s)
    diff_seconds=$((sec2 - sec1))
    echo "Runtime: $diff_seconds seconds"
)