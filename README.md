
## Требования

- **Unity 6000.0.23f1** (или та, что у тебя — уточни в `ProjectSettings/ProjectVersion.txt`)
- Git

## Как запустить

1. Установи ту же версию Unity через Unity Hub (Installs → Install Editor).
2. Склонируй репозиторий:
```bash
   git clone https://github.com/DrPepper18/YandexCamp_Robotics.git
```
3. В Unity Hub → *Add → Add project from disk* → выбери склонированную папку.
4. Кликни на проект и подожди первый импорт (несколько минут).
5. Открой сцену `Assets/Scenes/SampleScene.unity`.
6. Жми Play.

## Управление

- **WASD / стрелки** — движение робота
- **Пробел** — схватить / отпустить мяч клешнёй

## Структура

- `Assets/Scripts/TrackController.cs` — движение
- `Assets/Scripts/VirtualSensors.cs` — датчики
- `Assets/Scripts/GripperController.cs` — клешня
![alt text](<Screenshot 2026-07-20 121015.png>)