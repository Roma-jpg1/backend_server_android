# Backend Server Android

C++17 сервер для приема JSON-измерений с Android, записи их в JSONL/PostgreSQL и просмотра положения на карте через SDL2 + OpenGL + ImGui/ImPlot.

## Сборка

Зависимости: `cmake`, C++17-компилятор, `SDL2`, `GLEW`, `OpenGL`, `libpng`, `libzmq`, `cppzmq`, `libpq`. `nlohmann_json` берется из `third_party/json`, если не установлен в системе.

```bash
cmake -S . -B build
cmake --build build
```

## Запуск

GUI + сервер ZeroMQ:

```bash
./build/main
```

PostgreSQL включается опционально через `--db "host=... dbname=... user=... password=..."` или переменную `PGCONNINFO`.

Импорт измерений из JSON в PostgreSQL:

```bash
./build/main --db "host=localhost dbname=postgres user=postgres password=postgres" --import-json-to-db src/data.json
```

Файлы с форматом `cells[]` тоже поддерживаются: каждая запись из массива `cells` разворачивается в отдельную точку тепловой карты с собственными `earfcn`, `rsrp`, `rsrq`, `rssi`.

```bash
./build/main --db "host=localhost dbname=postgres user=postgres password=postgres" --import-json-to-db /Users/roman/Downloads/merged_locations.json
./build/main --db "host=localhost dbname=postgres user=postgres password=postgres"
```

## Тепловая карта

Тепловая карта считается методом IDW: для каждого пикселя берутся экспериментальные значения в радиусе 10-40 метров, вес точки равен обратному квадрату расстояния. Поддержаны критерии `RSRP`, `RSRQ`, `RSSI`, `Altitude`; если в данных есть числовой `mEarfcn`/`earfcn`, изображения генерируются отдельно для каждого EARFCN. Если задан `--db` или `PGCONNINFO`, точки для тепловой карты читаются из таблицы `user_equipment`.

В GUI heatmap-слой включен автоматически: при запуске и при смене zoom/области карты считаются видимые heatmap-тайлы и накладываются поверх OpenStreetMap. В окне `Heatmap` можно выбрать критерий, EARFCN, радиус IDW, радиус отображения, степень IDW и прозрачность.

- global image: сохраняет `build/heatmap_<criterion>.png`;
- per-tile image: сохраняет heatmap-тайлы рядом с OSM в `build/<zoom>/<x>/<y>_<criterion>.png` или `build/<zoom>/<x>/<y>_<criterion>_earfcn_<value>.png`.

OSM-тайлы кэшируются в формате задания: `build/<zoom>/<x>/<y>.png`. Автоматические тайлы оверлея heatmap хранятся отдельно в `build/heatmap/v3/<metric>/<earfcn>/idw_<radius>_draw_<radius>/<zoom>/<x>/<y>.png`, чтобы не затирать PNG-подложку OpenStreetMap.

RSRP раскрашивается по шкале: отличный сигнал красный, хороший оранжевый, средний голубой, слабый темно-синий, ниже `-110 dBm` не закрашивается.
