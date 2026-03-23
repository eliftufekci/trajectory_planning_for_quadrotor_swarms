cmake_minimum_required(VERSION 3.10)
project(QuadrotorSwarmPlanner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# IDE'lerin (IntelliSense/clangd) kütüphaneleri tanıması için compile_commands.json üret:
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── 1. Paketleri Bul (Sistemde yüklü olmaları gerekir) ────────────────
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system filesystem)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)

# ── 2. Executable (Çalıştırılabilir Dosya) ───────────────────────────
add_executable(env_test
    src/main.cpp
)

# ── 3. Include Dizinleri ──────────────────────────────────────────────
# Modern CMake'te 'target_include_directories' kullanmak daha profesyoneldir.
target_include_directories(env_test PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${OMPL_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
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
)

# ── 5. Build Sonrası Bilgi (Opsiyonel - Kontrol için) ──────────────────
message(STATUS "Eigen3 : ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "OctoMap : ${OCTOMAP_LIBRARIES}")
message(STATUS "OMPL : ${OMPL_INCLUDE_DIRS}")
message(STATUS "FCL : ${FCL_LIBRARIES}")
