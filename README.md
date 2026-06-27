
## Установка и запуск
Используйте образ clover VM с их официального репозитория (https://github.com/CopterExpress/clover_vm)

Перейдите в папку /home/clover/Documents/ и клонируйте в него репозиторий:
```bash
cd /home/clover/Documents/
git clone https://github.com/Izek1234/Kval_arhip.git
```

Перейдите в папку проекта в VSCode и запустите скрипт инициализации:
```bash
bash start_solar.sh
```
Этот скрипт генерирует мир и создает среду.
Запустить код для инспекции:
```bash
python3 /home/clover/Documents/Kval_arhip/scripts/solar_inspector.py
```

## Топики находятся по путям:
http://localhost:8080/stream_viewer?topic=/solar
http://localhost:8080/stream_viewer?topic=/solar/mask

## Структура проекта

| Файл | Назначение |
|---|---|
| `start_solar.sh` | Инициализация: копирование launch, генерация мира, запуск симулятора и нод |
| `simulator.launch` | ROS launch-файл для симуляции дрона Клевер в Gazebo |
| `scripts/gen_solar_farm.py` | Генерация случайного мира с нефтяным трубопроводом и врезками |
| `scripts/solar_inspector.py` | Нода автономного полёта: навигация вдоль трубы, обнаружение врезок через камеру, публикация координат |
