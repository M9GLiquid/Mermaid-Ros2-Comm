# Använda RobotPositionAPI med Riktig GPS Server

## Förutsättningar

1. **GPS-servern måste köras** och skicka meddelanden på ett ROS2-topic
2. **ROS2 måste vara aktiverat**: `source /opt/ros/jazzy/setup.bash`
3. **Kontrollera topic-namn och typ** innan du kör

## Steg 1: Kontrollera GPS-servern

### Lista tillgängliga topics:
```bash
ros2 topic list
```

### Se meddelanden på topic:
```bash
ros2 topic echo /robotPositions
```

### Se topic-typ och format:
```bash
ros2 topic info /robotPositions
ros2 topic type /robotPositions
```

Detta visar om meddelandet är:
- `std_msgs/msg/String` → använd `msg_type='string'`
- `std_msgs/msg/Float32MultiArray` → använd `msg_type='float32multiarray'`

## Steg 2: Kör testprogrammet

### Grundläggande test:
```bash
cd Ros2/api
python3 test_real_gps.py
# Välj alternativ 1
```

### Kontinuerlig övervakning:
```bash
cd Ros2/api
python3 test_real_gps.py
# Välj alternativ 2
```

## Steg 3: Använd i egen kod

### Exempel 1: Enkel användning
```python
from ros2_api import RobotPositionAPI

# Skapa API-instans med riktig topic
api = RobotPositionAPI(
    topic='robotPositions',  # Ändra till ditt topic-namn
    msg_type='string',  # eller 'float32multiarray'
    min_certainty=0.25,
    max_speed=500.0
)

api.start()

# Vänta lite för att få meddelanden
import time
time.sleep(2)

# Hämta positioner
for spiral_id in range(10):
    position = api.getPosition(spiral_id)
    if position:
        print(f"Spiral {spiral_id}: ({position.row}, {position.col})")

api.stop()
```

### Exempel 2: Med callback
```python
from ros2_api import RobotPositionAPI, SpiralRow

def on_update(robots: list[SpiralRow]):
    for robot in robots:
        print(f"Uppdaterad: Spiral {robot.id} på ({robot.row}, {robot.col})")

api = RobotPositionAPI(topic='robotPositions', min_certainty=0.25)
api.start()
api.setPositionCallback(on_update)

# Vänta på uppdateringar
import time
time.sleep(30)  # eller kör tills du vill stoppa

api.stop()
```

### Exempel 3: Kontinuerlig övervakning
```python
from ros2_api import RobotPositionAPI
import time

with RobotPositionAPI(topic='robotPositions', min_certainty=0.25) as api:
    while True:
        print("\n--- Positioner ---")
        for spiral_id in range(10):
            position = api.getPosition(spiral_id)
            if position:
                print(f"Spiral {spiral_id}: ({position.row:.2f}, {position.col:.2f})")
        time.sleep(2)  # Uppdatera var 2:e sekund
```

## Konfiguration

### Topic-namn
Om GPS-servern använder ett annat topic-namn, ändra:
```python
api = RobotPositionAPI(topic='ditt_topic_namn')
```

### Meddelandeformat
Om GPS-servern använder Float32MultiArray istället för String:
```python
api = RobotPositionAPI(topic='robotPositions', msg_type='float32multiarray')
```

### Filtrering
Justera filtrering baserat på dina behov:
```python
api = RobotPositionAPI(
    topic='robotPositions',
    min_certainty=0.5,  # Högre threshold = striktare filtrering
    max_speed=1000.0    # Högre = mer tolerans för outliers
)
```

## Felsökning

### Problem: Inga positioner hittas

1. **Kontrollera att GPS-servern körs:**
   ```bash
   ros2 topic echo /robotPositions
   ```
   Om detta visar meddelanden fungerar servern.

2. **Kontrollera topic-namn:**
   - Se vilka topics som finns: `ros2 topic list`
   - Använd exakt samma namn (inklusive `/` i början om det finns)

3. **Kontrollera meddelandeformat:**
   - Se topic-typ: `ros2 topic type /robotPositions`
   - Använd rätt `msg_type` i API:et

4. **Kontrollera filtrering:**
   - Sänk `min_certainty` om positioner filtreras bort
   - Öka `max_speed` om outliers filtreras bort

### Problem: Callbacks anropas inte

1. Kontrollera att positioner faktiskt kommer in (använd `getPosition()`)
2. Kontrollera att positioner inte filtreras bort
3. Se till att callback är satt efter `start()` anropas

### Problem: Fel meddelandeformat

Om du ser fel som "cannot parse" eller liknande:
1. Kontrollera meddelandeformatet med `ros2 topic echo /robotPositions`
2. Ändra `msg_type` till rätt typ
3. Kontrollera att meddelandet matchar förväntat format (se dokumentation för GPS-servern)

## Meddelandeformat som stöds

### String (JSON):
- Lista med dicts: `[{"id": 0, "row": 100.0, "col": 200.0, ...}, ...]`
- Flat list: `[100.0, 200.0, 0.0, 0, 0.8, ...]`
- Single dict: `{"id": 0, "row": 100.0, "col": 200.0, ...}`

### Float32MultiArray:
- Format: `[row, col, angle, id, cert, row, col, angle, id, cert, ...]`

## Tips

- Använd `ros2 topic echo` för att se exakt vad GPS-servern skickar
- Starta med högre `min_certainty` och sänk om inga positioner kommer igenom
- Använd callback för realtidsuppdateringar
- Använd `getPosition()` för polling-baserad användning
