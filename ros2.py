from __future__ import annotations
import argparse
import json
import re
import time
import math
import threading
import queue
from dataclasses import dataclass
from typing import Dict, List, Tuple, Union, Optional, Callable

import numpy as np

import rclpy
from rclpy import executors
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


@dataclass
class SpiralRow:
    id: int
    row: float
    col: float
    angle: float
    certainty: float


def rows_to_coords(rows: List[SpiralRow]) -> np.ndarray:
    if not rows:
        return np.zeros((0, 2), dtype=np.float32)
    return np.array([[r.row, r.col] for r in rows], dtype=np.float32)


def pairwise_distances(coords: np.ndarray) -> np.ndarray:
    if coords.size == 0:
        return np.zeros((0, 0), dtype=np.float32)
    diff = coords[:, None, :] - coords[None, :, :]
    return np.linalg.norm(diff, axis=2)


def pairwise_distances_squared(coords: np.ndarray) -> np.ndarray:
    if coords.size == 0:
        return np.zeros((0, 0), dtype=np.float32)
    diff = coords[:, None, :] - coords[None, :, :]
    return np.einsum('ijk,ijk->ij', diff, diff, dtype=np.float32)


def detect_collisions(rows: List[SpiralRow], threshold: float, prev_map: Dict[int, SpiralRow] = None,
                      min_certainty: float = 0.0, return_suppressed: bool = False,
                      direction_mode: str = 'approaching',
                      active_pairs: set[Tuple[int, int]] | None = None,
                      hyst_factor: float = 1.05
                      ) -> Union[List[Tuple[int, int, float]], Tuple[List[Tuple[int, int, float]], List[Tuple[int, int, float, float, float]]]]:
    ids = [r.id for r in rows]
    certs = [r.certainty for r in rows]
    coords = rows_to_coords(rows)
    d2 = pairwise_distances_squared(coords)
    n = len(rows)
    collisions: List[Tuple[int, int, float]] = []
    suppressed: List[Tuple[int, int, float, float, float]] = []
    dir_vecs: List[Union[np.ndarray, None]] = [None] * n
    if prev_map is not None:
        for idx, r in enumerate(rows):
            prev = prev_map.get(r.id)
            if prev is not None:
                dx = r.row - prev.row
                dy = r.col - prev.col
                norm = math.hypot(dx, dy)
                dir_vecs[idx] = None if norm == 0 else np.array([dx / norm, dy / norm], dtype=np.float32)
    for i in range(n):
        for j in range(i + 1, n):
            dist2 = float(d2[i, j])
            a_id, b_id = ids[i], ids[j]
            pair = (min(a_id, b_id), max(a_id, b_id))
            if direction_mode == 'approaching':
                vi, vj = dir_vecs[i], dir_vecs[j]
                if vi is not None and vj is not None:
                    r_vec = coords[j] - coords[i]
                    v_rel = vj - vi
                    approaching = float(np.dot(r_vec, v_rel)) < 0
            else:
                approaching = True

            thr2 = threshold * threshold
            thr_hyst2 = (threshold * hyst_factor) * (threshold * hyst_factor)
            sticky_inside = (direction_mode == 'sticky'
                             and active_pairs is not None
                             and pair in active_pairs
                             and dist2 <= thr_hyst2)

            if dist2 < thr2 or sticky_inside:
                low_cert = (certs[i] < min_certainty) or (certs[j] < min_certainty)
                approaching = True
                if direction_mode == 'approaching':
                    vi = dir_vecs[i]
                    vj = dir_vecs[j]
                    if vi is not None and vj is not None:
                        r_vec = coords[j] - coords[i]
                        v_rel = vj - vi
                        approaching = float(np.dot(r_vec, v_rel)) < 0
                else:
                    approaching = True
                if approaching:
                    dist = math.sqrt(dist2)
                    if low_cert:
                        if return_suppressed:
                            suppressed.append((a_id, b_id, dist, certs[i], certs[j]))
                    else:
                        collisions.append((a_id, b_id, dist))
    return (collisions, suppressed) if return_suppressed else collisions


def pretty_distances(rows: List[SpiralRow]) -> str:
    if not rows:
        return "<no robots>"
    ids = [r.id for r in rows]
    coords = rows_to_coords(rows)
    d = pairwise_distances(coords)
    n = len(rows)
    header = f"Present IDs ({len(ids)}): {sorted(ids)}"
    if n < 2:
        return header + "\n<only one robot>"
    pairs: List[Tuple[float, int, int]] = []
    for i in range(n):
        for j in range(i + 1, n):
            pairs.append((float(d[i, j]), ids[i], ids[j]))
    pairs.sort(key=lambda x: x[0])
    lines = [header, "Pairwise distances (sorted ascending):"]
    for dist, a, b in pairs:
        lines.append(f"  ({a}, {b}): {dist:.3f}")
    return "\n".join(lines)


def pretty_positions(rows: List[SpiralRow]) -> str:
    found: Dict[int, SpiralRow] = {r.id: r for r in rows if r.id >= 0}
    lines = ["Positions (ID, row, col, cert):"]
    for ident in range(10):
        r = found.get(ident)
        if r is None:
            lines.append(f"  {ident}: missing")
        else:
            lines.append(f"  {ident}: ({r.row:.3f}, {r.col:.3f})  cert={r.certainty:.2f}")
    return "\n".join(lines)


def pretty_table(rows: List[SpiralRow], header_time: bool = True) -> str:
    ts = time.strftime('%Y-%m-%d %H:%M:%S') if header_time else ''
    head = f"Time: {ts}" if header_time else "Positions and distances"
    sections = [head, pretty_positions(rows), pretty_distances(rows)]
    return "\n".join(sections)


def _as_spiral_rows_from_list(vals: List[float]) -> List[SpiralRow]:
    n = len(vals)
    rows: List[SpiralRow] = []
    if n % 5 != 0:
        return rows
    for i in range(0, n, 5):
        row, col, angle, rid, cert = (
            float(vals[i]),
            float(vals[i + 1]),
            float(vals[i + 2]),
            int(vals[i + 3]),
            float(vals[i + 4]),
        )
        if int(row) == -1 or rid == -1:
            continue
        rows.append(SpiralRow(rid, row, col, angle, cert))
    return rows


class AsyncFileAppender:
    def __init__(self, path: str):
        self.path = path
        self.q: "queue.Queue[str | None]" = queue.Queue()
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def _run(self):
        while True:
            data = self.q.get()
            if data is None:
                break
            with open(self.path, 'a', encoding='utf-8') as f:
                f.write(data)

    def write(self, data: str):
        self.q.put(data)

    def close(self):
        self.q.put(None)
        self.t.join(timeout=1.0)


class MatrixCollisionNode(Node):
    def __init__(self, topic: str, msg_type: str, threshold: float, tick_hz: float, stale_sec: float,
                 min_certainty: float = 0.4, direction_mode: str = 'approaching', hyst_factor: float = 1.05,
                 history_file: str | None = None, max_speed: float = 500.0):
        super().__init__('matrix_gps_ros2')
        self.threshold = threshold
        self.tick_interval = 1.0 / max(1e-3, tick_hz)
        self.stale_sec = stale_sec
        self.min_certainty = min_certainty
        self.direction_mode = direction_mode
        self.hyst_factor = hyst_factor
        self.max_speed = max_speed  # Max hastighet för outlier detection (enheter per sekund)
        
        # Thread-säkerhet
        self._lock = threading.RLock()
        
        # Robotpositioner
        self.rows: Dict[int, SpiralRow] = {}
        self.last_seen: Dict[int, float] = {}
        self.prev_rows: Dict[int, SpiralRow] = {}  # Föregående giltiga positioner för outlier detection
        self.prev_timestamps: Dict[int, float] = {}  # Timestamp för föregående positioner
        
        # Kollisionsdetektering (för framtida användning)
        self.active_collisions: set[Tuple[int, int]] = set()
        
        # Logging
        self.history_file = history_file
        self._dropped_rows: List[SpiralRow] = []
        self._history = AsyncFileAppender(history_file) if history_file else None
        
        # Callback för position updates
        self._position_callback: Optional[Callable[[List[SpiralRow]], None]] = None
        
        # ROS2 subscription och timer
        if msg_type == 'string':
            self.sub = self.create_subscription(String, topic, self.cb_string, 10)
        else:
            self.sub = self.create_subscription(Float32MultiArray, topic, self.cb_array, 10)
        self.timer = self.create_timer(self.tick_interval, self.on_tick)

    def upsert(self, row: SpiralRow):
        """Uppdatera robotposition med filtrering på certainty och outlier detection"""
        callback_to_call = None
        updated_robot = None
        
        with self._lock:
            # Filtrera på certainty
            if row.certainty < self.min_certainty:
                if self.history_file:
                    self._dropped_rows.append(row)
                return
            
            # Outlier detection: kolla om ny position är för långt från föregående
            prev_valid = self.prev_rows.get(row.id)
            if prev_valid is not None:
                # Beräkna avstånd till föregående giltiga position
                dx = row.row - prev_valid.row
                dy = row.col - prev_valid.col
                distance = math.hypot(dx, dy)
                
                # Beräkna tid sedan senaste uppdatering
                prev_time = self.prev_timestamps.get(row.id, time.time())
                current_time = time.time()
                delta_time = max(0.001, current_time - prev_time)  # Minst 1ms för att undvika division med 0
                
                # Max avstånd baserat på max hastighet
                max_distance = self.max_speed * delta_time
                
                # Om avståndet är för stort, ignorera (outlier)
                if distance > max_distance:
                    if self.history_file:
                        self._dropped_rows.append(row)
                    return
            
            # Spara föregående position om den finns
            prev = self.rows.get(row.id)
            if prev is not None:
                self.prev_rows[row.id] = prev
                self.prev_timestamps[row.id] = self.last_seen.get(row.id, time.time())
            
            # Uppdatera position
            self.rows[row.id] = row
            self.last_seen[row.id] = time.time()
            
            # Spara callback och uppdaterad robot för anrop utanför låset
            callback_to_call = self._position_callback
            updated_robot = row
        
        # Anropa callback utanför låset (callback körs i ROS2-tråden, håll den snabb!)
        if callback_to_call is not None and updated_robot is not None:
            try:
                callback_to_call([updated_robot])
            except Exception:
                # Ignorera fel i callback för att inte krascha ROS2-noden
                pass

    def cb_string(self, msg: String):
        s = msg.data
        try:
            obj = json.loads(s)
            if isinstance(obj, list):
                if obj and isinstance(obj[0], (list, tuple)):
                    flat: List[float] = []
                    for r in obj:
                        flat.extend([float(v) for v in r])
                    for row in _as_spiral_rows_from_list(flat):
                        self.upsert(row)
                elif obj and isinstance(obj[0], dict):
                    for r in obj:
                        row = float(r.get('row', r.get('r', r.get('Row', -1))))
                        col = float(r.get('col', r.get('c', r.get('Col', -1))))
                        angle = float(r.get('angle', r.get('Angle', 0.0)))
                        rid = int(r.get('id', r.get('ID', r.get('identity', -1))))
                        cert = float(r.get('certainty', r.get('conf', r.get('Conf', 1.0))))
                        if int(row) != -1 and rid != -1:
                            self.upsert(SpiralRow(rid, row, col, angle, cert))
                else:
                    vals = [float(v) for v in obj]
                    for row in _as_spiral_rows_from_list(vals):
                        self.upsert(row)
            elif isinstance(obj, dict):
                row = float(obj.get('row', obj.get('Row', -1)))
                col = float(obj.get('col', obj.get('Col', -1)))
                angle = float(obj.get('angle', obj.get('Angle', 0.0)))
                rid = int(obj.get('id', obj.get('ID', obj.get('identity', -1))))
                cert = float(obj.get('certainty', obj.get('Cert', 1.0)))
                if int(row) != -1 and rid != -1:
                    self.upsert(SpiralRow(rid, row, col, angle, cert))
            else:
                nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", s)
                vals = [float(v) for v in nums]
                for row in _as_spiral_rows_from_list(vals):
                    self.upsert(row)
        except Exception:
            try:
                nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", s)
                vals = [float(v) for v in nums]
                for row in _as_spiral_rows_from_list(vals):
                    self.upsert(row)
            except Exception:
                pass

    def cb_array(self, msg: Float32MultiArray):
        try:
            vals = list(msg.data)
            for row in _as_spiral_rows_from_list(vals):
                self.upsert(row)
        except Exception:
            pass

    def _append_history(self, rows: List[SpiralRow], collisions: List[Tuple[int, int, float]]):
        if not self._history:
            self._dropped_rows.clear()
            return
        parts: List[str] = []
        parts.append(pretty_table(rows, header_time=True))
        parts.append("\n")
        if self._dropped_rows:
            parts.append(f"Dropped (cert < {self.min_certainty:.2f}):\n")
            for r in self._dropped_rows:
                parts.append(f"  {r.id}: ({r.row:.3f}, {r.col:.3f}) angle={r.angle:.3f} cert={r.certainty:.2f}\n")
        else:
            parts.append("Dropped (cert < min): none\n")
        if collisions:
            parts.append("Collisions:\n")
            for a, b, d in collisions:
                parts.append(f"  {a} <-> {b} dist={d:.3f} (< {self.threshold})\n")
        else:
            parts.append("Collisions: none\n")
        parts.append("----------\n")
        snapshot = "".join(parts)
        self._history.write(snapshot)
        self._dropped_rows.clear()

    def on_tick(self):
        """Körs regelbundet av timern - kan användas för kollisionsdetektering senare"""
        # För nu är denna tom eftersom vi fokuserar på positioner
        # Här kan kollisionsdetektering läggas till senare
        pass
    
    def set_position_callback(self, callback: Optional[Callable[[List[SpiralRow]], None]]):
        """Sätt callback som anropas när positioner uppdateras"""
        with self._lock:
            self._position_callback = callback
    
    def get_position(self, spiral_id: int) -> Optional[SpiralRow]:
        """Hämta position för en specifik spiral (thread-safe)"""
        with self._lock:
            return self.rows.get(spiral_id)
    
    def get_all_positions(self) -> List[SpiralRow]:
        """Hämta alla aktiva robotpositioner (thread-safe, kopierar data)"""
        with self._lock:
            return list(self.rows.values())
    
    def destroy_node(self):
        try:
            if getattr(self, "_history", None):
                self._history.close()
        finally:
            super().destroy_node()


@dataclass
class NodeHandle:
    node: MatrixCollisionNode
    executor: executors.Executor
    thread: threading.Thread
    def stop(self):
        try:
            self.executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def start_async(topic: str = 'robotPositions', 
                msg_type: str = 'string',
                threshold: float = 150.0, 
                tick_hz: float = 5.0, 
                stale_sec: float = 5.0,
                min_cert: float = 0.25, 
                direction_mode: str = 'approaching',
                hyst_factor: float = 1.05, 
                history_file: str | None = './gps_history.log',
                multithread: bool = True,
                max_speed: float = 500.0) -> NodeHandle:
    """Starta ROS2-noden asynkront i bakgrunden"""
    if not rclpy.ok():
        rclpy.init()
    node = MatrixCollisionNode(topic, msg_type, threshold, tick_hz, stale_sec,
                               min_cert, direction_mode, hyst_factor, 
                               history_file=history_file, max_speed=max_speed)
    executor = executors.MultiThreadedExecutor() if multithread else executors.SingleThreadedExecutor()
    executor.add_node(node)
    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()
    return NodeHandle(node=node, executor=executor, thread=t)



# def main():
#     ap = argparse.ArgumentParser()
#     ap.add_argument('--topic', type=str,            default='robotPositions')
#     ap.add_argument('--msg-type', type=str,         default='string', choices=['string', 'float32multiarray'])
#     ap.add_argument('--threshold', type=float,      default=150.0)
#     ap.add_argument('--tick-hz', type=float,        default=5.0)
#     ap.add_argument('--stale-sec', type=float,      default=5.0)
#     ap.add_argument('--min-cert', type=float,       default=0.25)
#     ap.add_argument('--direction-mode', type=str,   default='approaching', choices=['approaching', 'any', 'sticky'])
#     ap.add_argument('--hyst-factor', type=float,    default=1.05)
#     ap.add_argument('--history-file', type=str,     default='./gps_history.log')
#     args = ap.parse_args()
#     rclpy.init()
#     node = MatrixCollisionNode(
#         args.topic, args.msg_type, args.threshold, args.tick_hz, args.stale_sec,
#         args.min_cert, args.direction_mode, args.hyst_factor, history_file=args.history_file
#     )
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


class RobotPositionAPI:
    """
    Enkelt API för att hämta robotpositioner från GPS Server via ROS2.
    
    Användning:
        api = RobotPositionAPI(topic='robotPositions', min_certainty=0.25, max_speed=500.0)
        api.start()
        
        # Hämta position för specifik spiral
        position = api.getPosition(5)
        
        # Sätt callback för uppdateringar
        api.setPositionCallback(lambda robots: print(f"Uppdaterade: {len(robots)}"))
        
        api.stop()
    """
    
    def __init__(self, 
                 topic: str = 'robotPositions',
                 msg_type: str = 'string',
                 min_certainty: float = 0.25,
                 max_speed: float = 500.0,
                 tick_hz: float = 5.0):
        """
        Initiera API:et för robotpositioner.
        
        Args:
            topic: ROS2 topic att lyssna på
            msg_type: Meddelandetyp ('string' eller 'float32multiarray')
            min_certainty: Minsta certainty för att acceptera position (0.0-1.0)
            max_speed: Max hastighet för outlier detection (enheter per sekund)
            tick_hz: Frekvens för timer (används för kollisionsdetektering senare)
        """
        self.topic = topic
        self.msg_type = msg_type
        self.min_certainty = min_certainty
        self.max_speed = max_speed
        self.tick_hz = tick_hz
        
        # ROS2-noden och executor
        self._node_handle: Optional[NodeHandle] = None
        self._node: Optional[MatrixCollisionNode] = None
    
    def start(self):
        """Starta att lyssna på robotpositioner från GPS Server"""
        if self._node_handle is not None:
            return  # Redan startad
        
        self._node_handle = start_async(
            topic=self.topic,
            msg_type=self.msg_type,
            min_cert=self.min_certainty,
            max_speed=self.max_speed,
            tick_hz=self.tick_hz,
            stale_sec=999999.0,  # Ingen stale-hantering behövs
            threshold=150.0,  # Används inte för nu (kollision kommer senare)
            direction_mode='approaching',
            hyst_factor=1.05,
            history_file=None,  # Ingen logging för nu
            multithread=True
        )
        self._node = self._node_handle.node
    
    def stop(self):
        """Stoppa att lyssna på positioner"""
        if self._node_handle:
            self._node_handle.stop()
            self._node_handle = None
            self._node = None
    
    def getPosition(self, spiralID: int) -> Optional[SpiralRow]:
        """
        Hämta position för en specifik spiral.
        
        Args:
            spiralID: ID för spiralen att hämta
        
        Returns:
            SpiralRow om spiralen finns och har giltig position, annars None
            Positioner med certainty < min_certainty eller outliers returneras inte
        """
        if self._node is None:
            return None
        return self._node.get_position(spiralID)
    
    def setPositionCallback(self, callback: Optional[Callable[[List[SpiralRow]], None]]):
        """
        Sätt callback som anropas när positioner uppdateras.
        
        Args:
            callback: Funktion som tar en lista med SpiralRow som argument
                     Anropas i ROS2-tråden, så håll den snabb!
        """
        if self._node is None:
            raise RuntimeError("API måste startas först (anropa start())")
        self._node.set_position_callback(callback)
    
    def __enter__(self):
        """Context manager support - startar automatiskt"""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager support - stoppar automatiskt"""
        self.stop()
