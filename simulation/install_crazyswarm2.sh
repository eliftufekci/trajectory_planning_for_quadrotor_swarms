#!/bin/bash

# Bu betik, Google Cloud Shell'de ROS 2 Jazzy üzerine Crazyswarm2'yi BINARY olarak kurar.
# Bu betiği çalıştırmadan önce ROS 2 Jazzy'nin kurulu ve ortam değişkenlerinin ayarlı olduğundan emin olun.
# Yani, 'install_ros2_jazzy.sh' betiğini çalıştırmış ve 'source ~/.bashrc' yapmış olmalısınız.

set -e

echo "0. ROS 2 ortamı kontrol ediliyor..."
if [ -z "$ROS_DISTRO" ]; then
    echo "HATA: ROS 2 ortamı ayarlanmamış. Lütfen önce ROS 2 Jazzy kurulumunu tamamlayın ve 'source ~/.bashrc' komutunu çalıştırın."
    exit 1
fi

if [ "$ROS_DISTRO" != "jazzy" ]; then
    echo "UYARI: ROS_DISTRO 'jazzy' değil, '$ROS_DISTRO' olarak ayarlı. Crazyswarm2 Jazzy için tasarlanmıştır."
fi

WORKSPACE_DIR="/home/wwweliftufekci/ros2_ws"

echo "1. Crazyswarm2 Binary paketleri ve bağımlılıklar yükleniyor..."
sudo apt update
sudo apt install -y ros-jazzy-crazyflie* ros-jazzy-motion-capture-tracking ros-jazzy-tf-transformations

echo "2. Python bağımlılıkları yükleniyor..."
# Python bağımlılıkları
pip3 install --user "setuptools<80" rowan nicegui==1.4.2 cflib transforms3d

echo "Kurulum tamamlandı! Binary paketler /opt/ros/jazzy altına yüklendi."
echo "Denemek için ros2 pkg list | grep crazyflie"
