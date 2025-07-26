#!/bin/bash

# Выполнение предварительных команд
colcon build --symlink-install
unset GTK_PATH
source install/setup.bash

# Массив с launch файлами
launch_files=(
    "aerobot_first_test_mavros_without_car.launch.py"
    "aerobot_first_test_mavros.launch.py"
    "aerobot_first_test_without_car.launch.py"
    "aerobot_first_test.launch.py"
    "aerobot_second_test_mavros.launch.py"
    "aerobot_second_test.launch.py"
    "aerobot_third_test_mavros.launch.py"
    "aerobot_third_test.launch.py"
)

# Вывод списка файлов
echo "Доступные launch файлы:"
for i in "${!launch_files[@]}"; do
    printf "%2d - %s\n" "$((i+1))" "${launch_files[$i]}"
done

# Запрос ввода от пользователя
echo ""
echo "Какой файл запустить?"
read -p "Ввод: " choice

# Удаление пробелов из ввода
choice=$(echo "$choice" | tr -d ' ')

# Проверка корректности ввода
if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le "${#launch_files[@]}" ]; then
    selected_file="${launch_files[$((choice-1))]}"
    echo "Запускаю: $selected_file"
    ros2 launch aerobot_gz_sim "$selected_file"
else
    echo "Неверный выбор. Пожалуйста, введите число от 1 до ${#launch_files[@]}"
    exit 1
fi