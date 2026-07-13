#!/usr/bin/env python3
import argparse
import json
import math
import signal
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path


TOPIC_VEHICLE_STATUS_FMT = "v2x/vehicle/{vehicle_id}/status"
MAX_CONFLICT_ZONES = 4
PASSED_CONFLICT_ZONE_THRESHOLD_CM = 10.0

BNO055_CHIP_ID_REG = 0x00
BNO055_PAGE_ID_REG = 0x07
BNO055_OPR_MODE_REG = 0x3D
BNO055_PWR_MODE_REG = 0x3E
BNO055_EULER_H_LSB = 0x1A

BNO055_CHIP_ID = 0xA0
OPERATION_MODE_CONFIG = 0x00
OPERATION_MODE_NDOF = 0x0C


def now_ms():
    return int(time.time() * 1000)


def normalize_angle_deg(angle):
    return angle % 360.0


def angle_delta_deg(current, reference):
    return (current - reference + 180.0) % 360.0 - 180.0


def apply_heading_deadband_x100(heading_x100):
    heading_x100 %= 36000

    if heading_x100 <= 500 or heading_x100 >= 35500:
        return 0
    if 8500 <= heading_x100 <= 9500:
        return 9000
    if 17500 <= heading_x100 <= 18500:
        return 18000
    if 26500 <= heading_x100 <= 27500:
        return 27000
    return heading_x100


def point_on_segment(px, py, a, b):
    eps = 1e-9
    cross = (px - a[0]) * (b[1] - a[1]) - (py - a[1]) * (b[0] - a[0])
    if abs(cross) > eps:
        return False

    dot = (px - a[0]) * (b[0] - a[0]) + (py - a[1]) * (b[1] - a[1])
    if dot < -eps:
        return False

    length_sq = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
    return dot - length_sq <= eps


def point_in_polygon(x, y, polygon):
    if len(polygon) < 3:
        return False

    for i in range(len(polygon)):
        if point_on_segment(x, y, polygon[i], polygon[(i + 1) % len(polygon)]):
            return True

    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


@dataclass
class Lanelet:
    lanelet_id: str
    polygon: list
    maneuver: str = "straight"
    unprotected_left: bool = False
    traffic_light_id: str = ""
    conflict_zone_ids: list = field(default_factory=list)
    movement_conflicts: dict = field(default_factory=dict)


@dataclass
class VehicleMapState:
    conflict_zone_ids: list = field(default_factory=list)


class IntersectionMap:
    def __init__(self, xml_path):
        self.xml_path = Path(xml_path)
        self.lanelets = []
        self.intersection_center = []
        self.conflict_zones = {}
        self.load()

    def load(self):
        root = ET.parse(self.xml_path).getroot()

        points = {}
        for point in root.findall("./points/point"):
            points[point.attrib["id"]] = (
                float(point.attrib["x"]),
                float(point.attrib["y"]),
            )

        for area in root.findall("./areas/area"):
            area_points = [points[ref.attrib["id"]] for ref in area.findall("point_ref")]
            area_type = area.attrib.get("type")
            if area_type == "intersection_center":
                self.intersection_center = area_points
            elif area_type == "conflict_zone":
                self.conflict_zones[area.attrib["id"]] = area_points

        lane_by_id = {}
        for node in root.findall("./lanelets/lanelet"):
            lane = Lanelet(
                lanelet_id=node.attrib["id"],
                polygon=[points[ref.attrib["id"]] for ref in node.findall("area_point_ref")],
                maneuver=node.attrib.get("maneuver", "straight"),
                unprotected_left=node.attrib.get("unprotected_left", "false") == "true",
                traffic_light_id=node.attrib.get("traffic_light_ref", ""),
            )
            self.lanelets.append(lane)
            lane_by_id[lane.lanelet_id] = lane

        for reg in root.findall("./regulatory_elements/regulatory_element"):
            if reg.attrib.get("type") == "traffic_light":
                tl_ref = reg.find("traffic_light_ref")
                if tl_ref is None:
                    continue
                for applies_to in reg.findall("applies_to"):
                    lane = lane_by_id.get(applies_to.attrib.get("lanelet_id", ""))
                    if lane:
                        lane.traffic_light_id = tl_ref.attrib.get("id", lane.traffic_light_id)

            if reg.attrib.get("type") != "conflict_zone":
                continue

            cz_ref = reg.find("conflict_zone_ref")
            if cz_ref is None:
                continue
            cz_id = cz_ref.attrib.get("id", "")
            if not cz_id:
                continue

            participants = reg.findall("participant")
            if participants:
                for participant in participants:
                    lane = lane_by_id.get(participant.attrib.get("lanelet_id", ""))
                    movement = participant.attrib.get("movement", "")
                    if lane and movement:
                        lane.movement_conflicts.setdefault(movement, [])
                        if cz_id not in lane.movement_conflicts[movement]:
                            lane.movement_conflicts[movement].append(cz_id)
                        if cz_id not in lane.conflict_zone_ids:
                            lane.conflict_zone_ids.append(cz_id)
                continue

            for applies_to in reg.findall("applies_to"):
                lane = lane_by_id.get(applies_to.attrib.get("lanelet_id", ""))
                if lane and cz_id not in lane.conflict_zone_ids:
                    lane.conflict_zone_ids.append(cz_id)

    def query(self, x_cm, y_cm):
        if self.intersection_center and point_in_polygon(x_cm, y_cm, self.intersection_center):
            return None, True

        for lane in self.lanelets:
            if point_in_polygon(x_cm, y_cm, lane.polygon):
                return lane, False

        return None, False

    def conflict_zone_center(self, conflict_zone_id):
        polygon = self.conflict_zones.get(conflict_zone_id)
        if not polygon:
            return None

        center_x = sum(point[0] for point in polygon) / len(polygon)
        center_y = sum(point[1] for point in polygon) / len(polygon)
        return center_x, center_y


class BNO055:
    def __init__(self, bus_num=1, addr=0x28):
        try:
            import smbus2
        except ImportError as exc:
            raise RuntimeError("smbus2 is required when BNO055 is enabled") from exc

        self.bus = smbus2.SMBus(bus_num)
        self.addr = addr

    def write8(self, reg, value):
        self.bus.write_byte_data(self.addr, reg, value)

    def read8(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def init(self):
        time.sleep(0.7)
        chip_id = self.read8(BNO055_CHIP_ID_REG)
        if chip_id != BNO055_CHIP_ID:
            raise RuntimeError("BNO055 chip id mismatch. Check wiring and I2C address.")

        self.write8(BNO055_PAGE_ID_REG, 0x00)
        time.sleep(0.01)
        self.write8(BNO055_OPR_MODE_REG, OPERATION_MODE_CONFIG)
        time.sleep(0.03)
        self.write8(BNO055_PWR_MODE_REG, 0x00)
        time.sleep(0.02)
        self.write8(BNO055_OPR_MODE_REG, OPERATION_MODE_NDOF)
        time.sleep(0.6)

    def read_heading_x100(self):
        for _ in range(3):
            try:
                lsb, msb = self.bus.read_i2c_block_data(self.addr, BNO055_EULER_H_LSB, 2)
                raw = (msb << 8) | lsb
                if raw & 0x8000:
                    raw -= 0x10000
                return int((raw * 100) / 16) % 36000
            except OSError:
                time.sleep(0.05)
        return None


class Position:
    def __init__(self, start_x_cm=0.0, start_y_cm=0.0):
        self.x_m = start_x_cm / 100.0
        self.y_m = start_y_cm / 100.0
        self.total_distance_m = 0.0
        self.last_time = time.monotonic()

    def update(self, speed_mps, heading_x100):
        current_time = time.monotonic()
        dt = min(current_time - self.last_time, 0.5)
        self.last_time = current_time

        if speed_mps < 0.02:
            return

        heading_rad = math.radians(heading_x100 / 100.0)
        distance_m = speed_mps * dt
        self.total_distance_m += distance_m
        self.x_m += distance_m * math.sin(heading_rad)
        self.y_m += distance_m * math.cos(heading_rad)

    @property
    def x_cm(self):
        return self.x_m * 100.0

    @property
    def y_cm(self):
        return self.y_m * 100.0


def maneuver_to_turn_state(maneuver, unprotected_left=False):
    if maneuver == "straight_right":
        return "right_turn"
    if maneuver == "straight_left":
        return "unprotected_left" if unprotected_left else "left_turn"
    return "straight"


def conflict_movement(turn_state):
    if turn_state == "right_turn":
        return "right_turn"
    if turn_state in ("left_turn", "unprotected_left"):
        return "left_turn"
    return "straight"


def conflict_zone_is_behind_vehicle(intersection_map, conflict_zone_id, x_cm, y_cm, heading_deg):
    center = intersection_map.conflict_zone_center(conflict_zone_id)
    if center is None:
        return False

    heading_rad = math.radians(heading_deg)
    dir_x = math.sin(heading_rad)
    dir_y = math.cos(heading_rad)
    to_cz_x = center[0] - x_cm
    to_cz_y = center[1] - y_cm
    projection = to_cz_x * dir_x + to_cz_y * dir_y
    return projection < -PASSED_CONFLICT_ZONE_THRESHOLD_CM


def filter_existing_conflict_zones_by_projection(intersection_map, previous_conflict_zone_ids, x_cm, y_cm, heading_deg):
    kept = []
    for zone_id in previous_conflict_zone_ids:
        if not conflict_zone_is_behind_vehicle(intersection_map, zone_id, x_cm, y_cm, heading_deg):
            kept.append(zone_id)
    return kept[:MAX_CONFLICT_ZONES]


def conflict_zones_from_lane(lane, turn_state):
    if not lane:
        return []

    movement = conflict_movement(turn_state)
    return list(lane.movement_conflicts.get(movement, []))[:MAX_CONFLICT_ZONES]


def update_conflict_zones(intersection_map, vehicle_state, lane, turn_state, x_cm, y_cm, heading_deg):
    if lane:
        vehicle_state.conflict_zone_ids = conflict_zones_from_lane(lane, turn_state)
        return

    vehicle_state.conflict_zone_ids = filter_existing_conflict_zones_by_projection(
        intersection_map,
        vehicle_state.conflict_zone_ids,
        x_cm,
        y_cm,
        heading_deg,
    )


def build_payload(vehicle_id, pos, speed_mps, heading_x100, lane, turn_state, vehicle_state):
    lanelet_id = ""
    linked_tl_id = ""

    if lane:
        lanelet_id = lane.lanelet_id
        linked_tl_id = lane.traffic_light_id

    return {
        "vehicle_id": vehicle_id,
        "x": max(0, min(65535, int(round(pos.x_cm)))),
        "y": max(0, min(65535, int(round(pos.y_cm)))),
        "speed": max(0, min(255, int(round(speed_mps * 100.0)))),
        "heading": max(0, min(65535, int(round(heading_x100 / 100.0)))),
        "lanelet_id": lanelet_id,
        "turn_state": turn_state,
        "conflict_zone_ids": vehicle_state.conflict_zone_ids,
        "conflict_zone_count": len(vehicle_state.conflict_zone_ids),
        "linked_tl_id": linked_tl_id,
        "timestamp_ms": now_ms(),
    }


def make_mqtt_client(vehicle_id, host, port, keepalive):
    try:
        import paho.mqtt.client as mqtt
    except ImportError as exc:
        raise RuntimeError("paho-mqtt is required. Install it with: pip install paho-mqtt") from exc

    status_topic = TOPIC_VEHICLE_STATUS_FMT.format(vehicle_id=vehicle_id)
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"subcar-{vehicle_id}")
    except AttributeError:
        client = mqtt.Client(client_id=f"subcar-{vehicle_id}")

    client.will_set(status_topic, payload=b"", qos=0, retain=True)
    client.connect(host, port, keepalive)
    client.loop_start()
    return client, status_topic


class KeyboardDriveController:
    def __init__(self, drive_key="w", quit_key="q", timeout_s=0.4):
        self.drive_key = drive_key.lower()
        self.quit_key = quit_key.lower()
        self.timeout_s = timeout_s
        self.last_drive_time = 0.0
        self.quit_requested = False
        self.enabled = sys.stdin.isatty()
        self.fd = None
        self.old_terminal_settings = None

    def start(self):
        if not self.enabled:
            print("[KEY] stdin is not a terminal. Speed will stay 0 unless keyboard input is available.")
            return

        import termios
        import tty

        self.fd = sys.stdin.fileno()
        self.old_terminal_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

    def poll(self):
        if not self.enabled:
            return

        import select

        while select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1).lower()
            if key == self.drive_key:
                self.last_drive_time = time.monotonic()
            elif key == self.quit_key:
                self.quit_requested = True

    def speed(self, drive_speed_mps):
        self.poll()
        if time.monotonic() - self.last_drive_time <= self.timeout_s:
            return drive_speed_mps
        return 0.0

    def stop(self):
        if not self.enabled or self.old_terminal_settings is None:
            return

        import termios

        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_terminal_settings)


def parse_args():
    base_dir = Path(__file__).resolve().parents[1]
    default_map = base_dir / "linux" / "v2x_mqtt" / "map" / "intersection_lanelet_v1.xml"

    parser = argparse.ArgumentParser(description="SubCar sensor + MQTT publisher")
    parser.add_argument("--vehicle-id", type=int, default=2)
    parser.add_argument("--mqtt-host", default="localhost")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--map", default=str(default_map))
    parser.add_argument("--start-x-cm", type=float, default=0.0)
    parser.add_argument("--start-y-cm", type=float, default=0.0)
    parser.add_argument("--speed-mps", type=float, default=0.2)
    parser.add_argument("--drive-key", default="w")
    parser.add_argument("--quit-key", default="q")
    parser.add_argument("--key-timeout-s", type=float, default=0.4)
    parser.add_argument("--initial-heading-deg", type=float, default=0.0)
    parser.add_argument("--period-s", type=float, default=0.1)
    parser.add_argument("--i2c-bus", type=int, default=1)
    parser.add_argument("--bno055-addr", type=lambda value: int(value, 0), default=0x28)
    parser.add_argument("--no-bno055", action="store_true")
    parser.add_argument(
        "--turn-state",
        choices=["straight", "right_turn", "left_turn", "unprotected_left"],
        default="straight",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if not 1 <= args.vehicle_id <= 63:
        raise ValueError("vehicle-id must be between 1 and 63")

    intersection_map = IntersectionMap(args.map)
    pos = Position(args.start_x_cm, args.start_y_cm)
    vehicle_state = VehicleMapState()
    client, topic = make_mqtt_client(args.vehicle_id, args.mqtt_host, args.mqtt_port, 30)

    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    imu = None
    initial_raw_heading_x100 = 0
    last_heading_x100 = int(args.initial_heading_deg * 100)

    if args.no_bno055:
        print("[BNO055] disabled. Fixed heading mode.")
    else:
        imu = BNO055(args.i2c_bus, args.bno055_addr)
        imu.init()
        print("Place the subcar at the start position and align the heading.")
        input(f"Press Enter to map the current heading to {args.initial_heading_deg:.1f} deg...")
        initial_raw_heading_x100 = imu.read_heading_x100()
        if initial_raw_heading_x100 is None:
            raise RuntimeError("Failed to read initial heading")

    keyboard = KeyboardDriveController(args.drive_key, args.quit_key, args.key_timeout_s)

    print(f"[MQTT] publishing to {topic} at {args.period_s:.3f}s period")
    print(f"[KEY] hold '{args.drive_key}' to publish speed={args.speed_mps:.2f} m/s, release to publish speed=0")
    print(f"[KEY] press '{args.quit_key}' to quit")
    keyboard.start()

    try:
        while running:
            current_speed_mps = keyboard.speed(args.speed_mps)
            if keyboard.quit_requested:
                running = False
                break

            if imu:
                raw_heading_x100 = imu.read_heading_x100()
                if raw_heading_x100 is not None:
                    delta = angle_delta_deg(raw_heading_x100 / 100.0, initial_raw_heading_x100 / 100.0)
                    mapped = normalize_angle_deg(args.initial_heading_deg + delta)
                    last_heading_x100 = apply_heading_deadband_x100(int(mapped * 100))
            else:
                last_heading_x100 = apply_heading_deadband_x100(int(args.initial_heading_deg * 100))

            pos.update(current_speed_mps, last_heading_x100)
            lane, in_center = intersection_map.query(pos.x_cm, pos.y_cm)
            heading_deg = last_heading_x100 / 100.0
            update_conflict_zones(
                intersection_map,
                vehicle_state,
                lane,
                args.turn_state,
                pos.x_cm,
                pos.y_cm,
                heading_deg,
            )
            payload = build_payload(
                args.vehicle_id,
                pos,
                current_speed_mps,
                last_heading_x100,
                lane,
                args.turn_state,
                vehicle_state,
            )

            client.publish(topic, json.dumps(payload, separators=(",", ":")), qos=0, retain=False)
            print(
                f"x={payload['x']:4d} y={payload['y']:4d} "
                f"speed={payload['speed']:3d} "
                f"heading={payload['heading']:3d} lane={payload['lanelet_id'] or '-':>19} "
                f"turn={payload['turn_state']:<16} cz={payload['conflict_zone_ids']}"
            )
            time.sleep(args.period_s)
    finally:
        keyboard.stop()
        client.publish(topic, payload=b"", qos=0, retain=True)
        client.loop_stop()
        client.disconnect()
        print("[STOP] subcar publisher stopped")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)
