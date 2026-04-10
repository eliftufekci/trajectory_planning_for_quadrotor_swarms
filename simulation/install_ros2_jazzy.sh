#!/bin/bash

# Bu betik Ubuntu 24.04 (Noble) üzerine ROS 2 Jazzy kurar.
# Not: Humble 24.04'te resmi olarak desteklenmediği için Jazzy önerilir.

set -e

echo "0. Tüm çakışan ROS kaynakları temizleniyor..."
# Sadece ros2.list değil, içinde 'ros' geçen tüm kaynak listelerini temizle
sudo rm -f /etc/apt/sources.list.d/*ros*.list
sudo rm -f /etc/apt/sources.list.d/*ros*.sources
sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg

# Ana sources.list içinde kalmış olabilecek hatalı satırları temizle
sudo sed -i '/packages.ros.org/d' /etc/apt/sources.list

echo "1. Locale ayarları yapılıyor..."
sudo apt update && sudo apt install locales gnupg2 -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "2. Ubuntu Universe repository etkinleştiriliyor..."
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe -y

echo "3. ROS 2 GPG anahtarı ekleniyor..."
sudo apt update
# Anahtarı indirip dearmor ederek kaydediyoruz (en güvenli yöntem)
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "4. Repository listesine ekleniyor..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "5. ROS 2 Jazzy paketleri kuruluyor..."
sudo apt update
sudo apt upgrade -y
# Cloud Shell için 'ros-base' (GUI'siz) sürümü daha performanslıdır.
sudo apt install ros-jazzy-ros-base -y
sudo apt install python3-colcon-common-extensions -y

echo "6. Ortam değişkenleri ayarlanıyor..."
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

echo "Kurulum tamamlandı! Değişikliklerin etkili olması için 'source ~/.bashrc' çalıştırın."
echo "Test etmek için: ros2 pkg list"