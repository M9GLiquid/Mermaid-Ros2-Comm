# ROS2 API - Användningsguide

Detta är den refaktoriserade versionen av ROS2-position API:et.

## Filer

- `ros2-api.py` - Huvudfilen med API:et
- `ros2.py` - Originalfilen (behålls för debugging)
- `test_ros2_api.py` - Komplett testprogram med egen publisher
- `demo_ros2_api.py` - Enkel demo som visar grundläggande användning

## Snabbstart

### Testprogram (rekommenderas för första gången)

Detta program skapar sin egen ROS2 publisher och testar allt automatiskt:

```bash
python3 test_ros2_api.py
```

Detta kommer:
- Skapa en test publisher som skickar meddelanden
- Testa grundläggande funktionalitet
- Testa certainty-filtrering
- Testa callbacks
- Testa outlier-detection
- Testa context manager

### Enkel demo

Om du redan har en ROS2 publisher som skickar på topic `robotPositions`:

```bash
python3 demo_ros2_api.py
```

## Användning i egen kod

### Grundläggande användning

```python
from ros2_api import RobotPositionAPI

# Skapa API-instans
api = RobotPositionAPI(
    topic='robotPositions',
    msg_type='string',  # eller 'float32multiarray'
    min_certainty=0.25,
    max_speed=500.0
)

# Starta
api.start()

# Hämta position för specifik spiral
position = api.getPosition(5)
if position:
    print(f"Spiral 5: ({position.row}, {position.col})")

# Stoppa
api.stop()
```

### Med context manager (rekommenderas)

```python
from ros2_api import RobotPositionAPI

with RobotPositionAPI(topic='robotPositions', min_certainty=0.25) as api:
    position = api.getPosition(5)
    if position:
        print(f"Spiral 5: ({position.row}, {position.col})")
    # API stoppas automatiskt när vi går ut
```

### Med callback

```python
from ros2_api import RobotPositionAPI, SpiralRow

def on_position_update(robots: list[SpiralRow]):
    for robot in robots:
        print(f"Uppdaterad: Spiral {robot.id} på ({robot.row}, {robot.col})")

api = RobotPositionAPI(topic='robotPositions', min_certainty=0.25)
api.start()
api.setPositionCallback(on_position_update)

# Vänta på uppdateringar...
import time
time.sleep(10)

api.stop()
```

## Parametrar

### RobotPositionAPI

- `topic` (str): ROS2 topic att lyssna på (default: `'robotPositions'`)
- `msg_type` (str): Meddelandetyp (`'string'` eller `'float32multiarray'`, default: `'string'`)
- `min_certainty` (float): Minsta certainty för att acceptera position (0.0-1.0, default: `0.25`)
- `max_speed` (float): Max hastighet för outlier detection i enheter per sekund (default: `500.0`)
- `tick_hz` (float): Frekvens för timer (används för framtida kollisionsdetektering, default: `5.0`)

## Filtrering

API:et filtrerar automatiskt:

1. **Certainty-filter**: Positioner med `certainty < min_certainty` ignoreras
2. **Outlier-detection**: Positioner som är för långt från föregående position (baserat på `max_speed`) ignoreras

## Meddelandeformat

API:et stödjer flera meddelandeformat:

### JSON String (msg_type='string')

**Lista med dicts:**
```json
[
  {"id": 0, "row": 100.0, "col": 200.0, "angle": 0.0, "certainty": 0.8},
  {"id": 1, "row": 150.0, "col": 250.0, "angle": 1.57, "certainty": 0.9}
]
```

**Flat list:**
```json
[100.0, 200.0, 0.0, 0, 0.8, 150.0, 250.0, 1.57, 1, 0.9]
```

**Single dict:**
```json
{"id": 0, "row": 100.0, "col": 200.0, "angle": 0.0, "certainty": 0.8}
```

### Float32MultiArray (msg_type='float32multiarray')

Format: `[row, col, angle, id, cert, row, col, angle, id, cert, ...]`

## Felsökning

### Inga positioner hittas

1. Kontrollera att ROS2 publisher körs
2. Kontrollera att topic-namnet stämmer
3. Kontrollera att meddelandeformatet stämmer
4. Kontrollera att certainty är tillräckligt hög (`min_certainty`)

### Callbacks anropas inte

1. Kontrollera att positioner faktiskt kommer in (använd `getPosition()`)
2. Kontrollera att positioner inte filtreras bort (certainty, outlier)
3. Se till att callback är satt efter `start()` anropas

### Outliers filtreras bort

1. Öka `max_speed` om robotar faktiskt kan röra sig snabbt
2. Kontrollera att meddelanden kommer regelbundet (10-100ms)

## Arkitektur

API:et är uppdelat i separata komponenter:

- **MessageParser**: Parsar ROS2-meddelanden till SpiralRow-objekt
- **PositionFilter**: Filtrerar positioner (certainty, outlier)
- **PositionStore**: Thread-safe storage av positioner
- **MatrixCollisionNode**: ROS2-nod som orchestrerar allt
- **RobotPositionAPI**: Enkelt API för användare

## Framtida funktionalitet

- Kollisionsdetektering kommer läggas till senare
- Logging till fil kan läggas till vid behov
