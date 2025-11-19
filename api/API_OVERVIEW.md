# API Översikt - Vad behövs för att köra API:et

## 📁 Nödvändiga filer

### Huvudfiler (måste finnas)

1. **`ros2-api.py`** - Huvudfilen med all API-logik
   - Innehåller alla klasser och funktioner
   - Måste finnas i samma mapp eller i Python path

2. **`ros2.py`** - Originalfilen (måste finnas i parent directory)
   - Innehåller `SpiralRow` dataclass som importeras
   - Måste finnas i `Ros2/` mappen (en nivå upp från `api/`)

### Test/Demo filer (valfria, för testning)

- `test_ros2_api.py` - Testprogram med simulerad publisher
- `test_real_gps.py` - Testprogram för riktig GPS-server
- `demo_ros2_api.py` - Enkel demo

## 🔧 Systemkrav

### ROS2 (måste vara installerat)
- ROS2 Jazzy (eller annan distribution)
- `rclpy` Python-paket
- `std_msgs` Python-paket

### Python dependencies
- `numpy` (för matematiska operationer)
- Standard library: `json`, `re`, `time`, `math`, `threading`, `dataclasses`

## 📦 Klasser och funktioner i ros2-api.py

### 1. MessageParser (Intern klass)
**Ansvar:** Parsar ROS2-meddelanden till SpiralRow-objekt

**Metoder:**
- `parse_string(msg: String) -> List[SpiralRow]`
  - Parsar JSON-sträng meddelanden
  - Stödjer: lista med dicts, flat list, single dict, regex fallback
  
- `parse_array(msg: Float32MultiArray) -> List[SpiralRow]`
  - Parsar Float32MultiArray meddelanden
  - Format: [row, col, angle, id, cert, ...]

**Används av:** MatrixCollisionNode (internt)

---

### 2. PositionFilter (Intern klass)
**Ansvar:** Filtrerar positioner baserat på certainty och outlier detection

**Metoder:**
- `should_accept(row: SpiralRow, min_certainty: float) -> bool`
  - Kollar om certainty är tillräckligt hög
  - Returnerar True om `row.certainty >= min_certainty`

- `is_outlier(row: SpiralRow, prev_row: Optional[SpiralRow], prev_timestamp: Optional[float], max_speed: float) -> bool`
  - Kollar om position är för långt från föregående (outlier)
  - Beräknar avstånd och jämför med max_speed × tid
  - Returnerar True om det är en outlier

**Används av:** MatrixCollisionNode (internt)

---

### 3. PositionStore (Intern klass)
**Ansvar:** Thread-safe storage av robotpositioner

**Metoder:**
- `get(spiral_id: int) -> Optional[SpiralRow]`
  - Hämta position för specifik spiral (thread-safe)
  - Returnerar None om spiralen inte finns

- `get_all() -> List[SpiralRow]`
  - Hämta alla positioner (thread-safe, kopierar data)

- `set(row: SpiralRow)`
  - Uppdatera position (thread-safe)
  - Sparar automatiskt föregående position för outlier detection

- `get_prev(spiral_id: int) -> Tuple[Optional[SpiralRow], Optional[float]]`
  - Hämta föregående position och timestamp
  - Används för outlier detection

**Används av:** MatrixCollisionNode (internt)

---

### 4. MatrixCollisionNode (Intern klass)
**Ansvar:** ROS2-nod som tar emot meddelanden och orchestrerar allt

**Metoder:**
- `__init__(topic, msg_type, min_certainty, max_speed, tick_hz)`
  - Initierar ROS2-nod
  - Skapar subscription till topic
  - Skapar timer för regelbundna uppdateringar

- `cb_string(msg: String)` / `cb_array(msg: Float32MultiArray)`
  - Callbacks för ROS2-meddelanden
  - Parsar meddelanden och anropar `_upsert()`

- `_upsert(row: SpiralRow)`
  - Uppdaterar position med filtrering
  - Använder PositionFilter och PositionStore
  - Anropar callback om position accepteras

- `get_position(spiral_id: int) -> Optional[SpiralRow]`
  - Hämta position (delegerar till PositionStore)

- `get_all_positions() -> List[SpiralRow]`
  - Hämta alla positioner (delegerar till PositionStore)

- `set_position_callback(callback: Optional[Callable])`
  - Sätt callback för position updates

**Används av:** RobotPositionAPI (internt)

---

### 5. NodeHandle (Dataclass)
**Ansvar:** Hanterar ROS2-nod och executor för cleanup

**Metoder:**
- `stop()`
  - Stoppar executor och noden
  - Stänger ner ROS2 korrekt

**Används av:** start_async() och RobotPositionAPI (internt)

---

### 6. start_async() (Funktion)
**Ansvar:** Starta ROS2-noden asynkront i bakgrunden

**Parametrar:**
- `topic: str` - ROS2 topic att lyssna på (default: 'robotPositions')
- `msg_type: str` - 'string' eller 'float32multiarray' (default: 'string')
- `min_cert: float` - Minsta certainty (default: 0.25)
- `max_speed: float` - Max hastighet för outlier detection (default: 500.0)
- `tick_hz: float` - Timer frekvens (default: 5.0)
- `multithread: bool` - Använd multithreaded executor (default: True)

**Returnerar:** NodeHandle

**Används av:** RobotPositionAPI.start() (internt)

---

### 7. RobotPositionAPI (Huvudklass - ANVÄND DENNA!)
**Ansvar:** Enkelt API för att hämta robotpositioner

**Metoder:**

#### `__init__(topic, msg_type, min_certainty, max_speed, tick_hz)`
Initierar API:et (startar INTE automatiskt)

**Parametrar:**
- `topic: str` - ROS2 topic (default: 'robotPositions')
- `msg_type: str` - 'string' eller 'float32multiarray' (default: 'string')
- `min_certainty: float` - Minsta certainty (default: 0.25)
- `max_speed: float` - Max hastighet för outlier detection (default: 500.0)
- `tick_hz: float` - Timer frekvens (default: 5.0)

#### `start()`
Startar att lyssna på robotpositioner från GPS Server
- Måste anropas innan `getPosition()` kan användas
- Kan anropas flera gånger (ignorerar om redan startad)

#### `stop()`
Stoppar att lyssna på positioner
- Stänger ner ROS2-noden korrekt
- Måste anropas när du är klar

#### `getPosition(spiralID: int) -> Optional[SpiralRow]`
Hämta position för en specifik spiral

**Parametrar:**
- `spiralID: int` - ID för spiralen (0-9 eller annat ID)

**Returnerar:**
- `SpiralRow` om spiralen finns och har giltig position
- `None` om spiralen inte finns eller filtreras bort

**Exempel:**
```python
position = api.getPosition(5)
if position:
    print(f"Spiral 5: ({position.row}, {position.col})")
```

#### `setPositionCallback(callback: Optional[Callable[[List[SpiralRow]], None]])`
Sätt callback som anropas när positioner uppdateras

**Parametrar:**
- `callback: Callable` - Funktion som tar lista med SpiralRow som argument
  - Anropas i ROS2-tråden, så håll den snabb!
  - Kan vara None för att ta bort callback

**Exempel:**
```python
def on_update(robots):
    for robot in robots:
        print(f"Uppdaterad: Spiral {robot.id}")

api.setPositionCallback(on_update)
```

#### `__enter__()` / `__exit__()`
Context manager support för automatisk start/stop

**Exempel:**
```python
with RobotPositionAPI() as api:
    position = api.getPosition(5)
    # API stoppas automatiskt när vi går ut
```

---

## 📊 Data-typer

### SpiralRow (Dataclass från ros2.py)
Representerar en robotposition

**Fält:**
- `id: int` - Robotens/spiralens ID
- `row: float` - X-koordinat (row)
- `col: float` - Y-koordinat (col)
- `angle: float` - Vinkel/riktning
- `certainty: float` - Konfidensgrad (0.0-1.0)

**Exempel:**
```python
position = api.getPosition(5)
if position:
    print(f"ID: {position.id}")
    print(f"Position: ({position.row}, {position.col})")
    print(f"Vinkel: {position.angle}")
    print(f"Certainty: {position.certainty}")
```

---

## 🔄 Flöde när API:et används

1. **Användare skapar API:**
   ```python
   api = RobotPositionAPI(topic='robotPositions')
   ```

2. **Användare startar API:**
   ```python
   api.start()
   ```
   - Skapar ROS2-nod
   - Skapar subscription till topic
   - Startar executor i bakgrundstråd

3. **GPS-servern skickar meddelande:**
   - Meddelande kommer till `cb_string()` eller `cb_array()`
   - MessageParser parsar meddelandet till SpiralRow-objekt
   - För varje SpiralRow anropas `_upsert()`

4. **Filtrering sker:**
   - PositionFilter kollar certainty
   - PositionFilter kollar om det är outlier
   - Om accepterad → sparas i PositionStore
   - Callback anropas om satt

5. **Användare hämtar positioner:**
   ```python
   position = api.getPosition(5)
   ```
   - Hämtar från PositionStore (thread-safe)

6. **Användare stoppar API:**
   ```python
   api.stop()
   ```
   - Stänger ner ROS2-nod och executor

---

## ✅ Minimal användning (allt du behöver)

```python
import sys
import os
import importlib.util

# Importera API (pga filnamn med bindestreck)
ros2_api_path = "Ros2/api/ros2-api.py"
spec = importlib.util.spec_from_file_location("ros2_api", ros2_api_path)
ros2_api = importlib.util.module_from_spec(spec)
sys.modules["ros2_api"] = ros2_api
spec.loader.exec_module(ros2_api)

RobotPositionAPI = ros2_api.RobotPositionAPI

# Använd API:et
with RobotPositionAPI(topic='robotPositions') as api:
    position = api.getPosition(5)
    if position:
        print(f"Spiral 5: ({position.row}, {position.col})")
```

---

## 📝 Sammanfattning

**För att köra API:et behöver du:**

1. ✅ `ros2-api.py` (huvudfilen)
2. ✅ `ros2.py` (i parent directory, för SpiralRow)
3. ✅ ROS2 installerat (rclpy, std_msgs)
4. ✅ GPS-servern körs och skickar meddelanden

**Viktigaste funktionerna för användare:**

- `RobotPositionAPI()` - Skapa API-instans
- `start()` - Starta att lyssna
- `getPosition(spiralID)` - Hämta position
- `setPositionCallback(callback)` - Sätt callback
- `stop()` - Stoppa

**Allt annat är internt och behöver inte användas direkt!**
