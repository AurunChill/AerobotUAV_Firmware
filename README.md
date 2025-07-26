# Aerobot Gazebo Simulation

Симуляция дрона с использованием ROS2 Kilted, Gazebo Ionic и PX4 Autopilot.

## Требования

### Системные требования
- **ОС**: Ubuntu 24.04 LTS
- **RAM**: минимум 12GB (рекомендуется 16GB)
- **Диск**: минимум 30GB свободного места
- **CPU**: 4+ ядра

### Версии программного обеспечения
- **ROS2**: Kilted
- **Gazebo**: Ionic
- **PX4-Autopilot**: 1.15.4
- **QGroundControl**: latest stable
- **Python**: 3.10+

## Установка

### 1. Установка ROS2 Kilted

Убедитесь, что ваши локали поддерживают UTF-8:
```bash
locale  # должен показать UTF-8
sudo apt update && sudo apt upgrade
sudo apt install software-properties-common
sudo add-apt-repository universe

# Установка ros-apt-source
sudo apt install curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Установка ROS2 Kilted
sudo apt update && sudo apt upgrade
sudo apt install ros-kilted-desktop
```

Настройка окружения:
```bash
echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

Проверка установки ROS2:
```bash
ros2 --version
# Должно вывести: ros2 <версия>
```

```bash
# Запуск демо-ноды talker/listener
# Терминал 1:
ros2 run demo_nodes_cpp talker
```

```bash
# Терминал 2:
ros2 run demo_nodes_cpp listener
# Должны увидеть сообщения "Hello World"
```

<img width="1627" height="578" alt="image" src="https://github.com/user-attachments/assets/97e5179d-a928-4765-bfb2-7cd65707de5e" />


### 2. Установка Gazebo Ionic

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-ionic
```

Проверка установки Gazebo:
```bash
gz sim --version
# Должно вывести версию Gazebo Sim
```

```bash
# Запуск пустого мира
gz sim empty.sdf
# Должно открыться окно Gazebo с пустым миром
```
<img width="1194" height="1034" alt="image" src="https://github.com/user-attachments/assets/6c6a428e-3382-4219-897c-b896abfbfdb9" />


### 3. Установка ros-gz bridge

```bash
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt install ros-kilted-ros-gz
```

Проверка установки:

```bash
# Терминал 1: Запустите Gazebo с демо-миром
gz sim -v4 -r shapes.sdf

# Терминал 2: Запустите bridge
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock

# Терминал 3: Проверьте топик
ros2 topic echo /clock
# Должны видеть временные метки
```
<img width="1786" height="1044" alt="image" src="https://github.com/user-attachments/assets/88826cb9-714a-4daf-a7d3-5a5e258d9963" />


### 4. Установка MAVROS

```bash
sudo apt install ros-kilted-mavros ros-kilted-mavros-extras
```

```bash
# Запустите MAVROS (даже без подключения к FCU)
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@localhost:14580

# В другом терминале проверьте топики
ros2 topic list | grep mavros
# Должен показать список топиков MAVROS
```
<img width="1629" height="567" alt="image" src="https://github.com/user-attachments/assets/2fa1e5d7-ea0a-483f-a211-9b605d5cc70b" />


### 5. Установка инструментов сборки
```bash
sudo apt install python3-colcon-common-extensions
```

```bash
colcon version-check
# Должно показать версии colcon
```

### 6. Установка QGroundControl
```bash
sudo usermod -aG dialout "$(id -un)"
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y

# Скачивание QGroundControl
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage
chmod +x QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage
```
<img width="1377" height="1043" alt="image" src="https://github.com/user-attachments/assets/b4640c20-8f84-40da-8a22-2fe1b0c91a90" />


### 7. Установка PX4 Autopilot
```bash
sudo apt install git
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools
```
**Обязательно перегружаем компьютер!**

Проверка установки:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Отдельно запустите QGroundControl
```
<img width="1839" height="1046" alt="image" src="https://github.com/user-attachments/assets/748e4415-e4ff-460b-b017-da5e99a815a6" />


### 8. Установка Micro-XRCE-DDS-Agent (опционально, но желательно)
```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

```bash
# Запуск
MicroXRCEAgent udp4 -p 8888

# Вывод должен быть примерно такой
[1753517454.939337] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 8888
[1753517454.939492] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
```

### 9. Настройка PX4 для кастомной модели дрона

Необходимо добавить конфигурацию для нашей модели дрона:

```bash
cd ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
touch 4022_gz_x500_mono_cam_forward_down_drone
sudo nano 4022_gz_x500_mono_cam_forward_down_drone
```

Вставьте следующее содержимое:
```bash
PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_mono_cam_forward_down_drone}
. ${R}etc/init.d-posix/airframes/4001_gz_x500
param set COM_OF_LOSS_T 20
```

Теперь отредактируйте CMakeLists.txt:
```bash
cd ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
sudo nano CMakeLists.txt
```

Добавьте:
```bash
4022_gz_x500_mono_cam_forward_down_drone
```

<img width="822" height="577" alt="image" src="https://github.com/user-attachments/assets/a0150d66-4057-4265-ac5d-8c66895738c2" />


### 10. Настройка переменных окружения
```bash
echo "export GZ_SIM_RESOURCE_PATH=\$HOME/Firmware2/src/aerobot_gz_sim/worlds:\$HOME/Firmware2/src/aerobot_gz_sim/models:\${GZ_SIM_RESOURCE_PATH}" >> ~/.bashrc
source ~/.bashrc
```

### 11. Запуск
Запустите QGroundControl

Далее:
```bash
cd ~
git clone https://github.com/AurunChill/Firmware2
cd ~/Firmware2
chmod +x launch.sh
./launch.sh
```

<img width="1835" height="1003" alt="image" src="https://github.com/user-attachments/assets/21d6e560-a61a-4b0e-a32f-4e1fe640f5c5" />

Некоторое время после запуска в QGroundControl будет надпись "Not Ready". Подождите пару секунд
