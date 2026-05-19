cmake_minimum_required(VERSION 3.10)
project(QuadrotorSwarmPlanner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON) 
set(CMAKE_BUILD_TYPE Debug) # Hata ayıklama için 'Release' yerine 'Debug' kullanın

# IDE'lerin (IntelliSense/clangd) kütüphaneleri tanıması için compile_commands.json üret:
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── 1. Paketleri Bul (Sistemde yüklü olmaları gerekir) ────────────────
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system filesystem)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)
find_package(osqp REQUIRED)

# osqp-eigen'ın kendi içindeki hata ayıklama çıktılarını etkinleştir
add_compile_definitions(OSQP_EIGEN_DEBUG_OUTPUT)

# osqp-eigen'i bir alt proje olarak ekle. Bu, kütüphaneyi doğru şekilde
# yapılandırır ve OsqpEigen::OsqpEigen hedefini oluşturur.
add_subdirectory(libs/osqp-eigen)

# ── 2. Executable (Çalıştırılabilir Dosya) ───────────────────────────
# src altındaki tüm .cpp dosyalarını otomatik olarak SOURCE_FILES değişkenine atar
file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

add_executable(env_test
    ${SOURCE_FILES}
)

# ── 3. Include Dizinleri ──────────────────────────────────────────────
# Modern CMake'te 'target_include_directories' kullanmak daha profesyoneldir.
target_include_directories(env_test PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${OMPL_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/libs/libMultiRobotPlanning/include
)

# ── 4. Kütüphaneleri Bağla (Linking) ──────────────────────────────────
# Not: fcl için bazen ${FCL_LIBRARIES} bazen fcl::fcl kullanılır. 
# En garanti yol değişkenleri kullanmaktır.
target_link_libraries(env_test
    ${OCTOMAP_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${FCL_LIBRARIES}
    yaml-cpp
    Eigen3::Eigen
    osqp::osqp # osqp-eigen buna zaten bağımlı, ama burada kalması zarar vermez.
    OsqpEigen::OsqpEigen # osqp-eigen kütüphanesini doğru şekilde bağla.
)

# ── 5. Build Sonrası Bilgi (Opsiyonel - Kontrol için) ──────────────────
message(STATUS "Eigen3 : ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "OctoMap : ${OCTOMAP_LIBRARIES}")
message(STATUS "OMPL : ${OMPL_INCLUDE_DIRS}")
message(STATUS "FCL : ${FCL_LIBRARIES}")
message(STATUS "OSQP : ${OSQP_LIBRARIES}")
