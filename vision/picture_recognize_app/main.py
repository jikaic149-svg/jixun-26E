"""Hex25E 第一问：巡检识别、MCU 串口通信和掉电后结果显示。"""

import os
import time as pytime

from maix import app, camera, display, image, nn, pinmap, time, uart

import vision_config as config
from geometry_center import find_geometry_center


COLORS = [image.Color.from_rgb(*rgb) for rgb in config.CLASS_COLORS_RGB]
WHITE = image.Color.from_rgb(255, 255, 255)
GREEN = image.Color.from_rgb(40, 255, 80)
RED = image.Color.from_rgb(255, 40, 40)
YELLOW = image.Color.from_rgb(255, 220, 0)
CYAN = image.Color.from_rgb(0, 220, 255)
BLACK = image.Color.from_rgb(0, 0, 0)


def now_seconds():
    try:
        return pytime.monotonic()
    except Exception:
        try:
            return time.ticks_ms() / 1000.0
        except Exception:
            return pytime.time()


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_packet(packet_type, sequence, payload):
    payload_bytes = payload.encode("ascii")
    prefix = "@1|%s|%d|%d|" % (
        packet_type,
        sequence & 0xFFFF,
        len(payload_bytes),
    )
    covered = prefix.encode("ascii") + payload_bytes + b"|"
    return covered + ("%04X\n" % crc16_ccitt_false(covered)).encode("ascii")


def parse_packet(line):
    try:
        if line.endswith(b"\r"):
            line = line[:-1]
        parts = line.split(b"|")
        if len(parts) != 6 or parts[0] != b"@1":
            return None
        covered = b"|".join(parts[:5]) + b"|"
        if crc16_ccitt_false(covered) != int(parts[5], 16):
            return None
        packet_type = parts[1].decode("ascii")
        sequence = int(parts[2])
        payload_length = int(parts[3])
        if payload_length != len(parts[4]):
            return None
        return packet_type, sequence, parts[4].decode("ascii")
    except Exception:
        return None


class InspectionHistory:
    def __init__(self):
        self.records = []
        self.elapsed_seconds = 0.0
        self.complete = False
        self.load()

    def clear_for_new_run(self):
        self.records = []
        self.elapsed_seconds = 0.0
        self.complete = False
        self.save()

    def upsert(self, location, shape, class_id, confidence):
        record = (location, shape, class_id, confidence)
        for index, old in enumerate(self.records):
            if old[0] == location:
                self.records[index] = record
                self.save()
                return
        if len(self.records) < config.MAX_RECORDS:
            self.records.append(record)
            self.records.sort(key=lambda item: item[0])
            self.save()

    def finish(self, elapsed_seconds):
        self.elapsed_seconds = max(0.0, float(elapsed_seconds))
        self.complete = True
        self.save()

    def save(self):
        lines = [
            "version=1",
            "complete=%d" % (1 if self.complete else 0),
            "inspection_time=%.2f" % self.elapsed_seconds,
        ]
        for location, shape, class_id, confidence in self.records:
            lines.append(
                "record=%s,%s,%d,%d"
                % (location, shape, class_id, confidence)
            )
        try:
            with open(config.HISTORY_TEMP_FILE, "w", encoding="utf-8") as file:
                file.write("\n".join(lines) + "\n")
                file.flush()
                os.fsync(file.fileno())
            try:
                os.replace(config.HISTORY_TEMP_FILE, config.HISTORY_FILE)
            except AttributeError:
                if os.path.exists(config.HISTORY_FILE):
                    os.remove(config.HISTORY_FILE)
                os.rename(config.HISTORY_TEMP_FILE, config.HISTORY_FILE)
        except Exception as error:
            print("history save failed:", error)

    def load(self):
        self.records = []
        self.elapsed_seconds = 0.0
        self.complete = False
        if not os.path.exists(config.HISTORY_FILE):
            return
        try:
            with open(config.HISTORY_FILE, "r", encoding="utf-8") as file:
                for raw_line in file:
                    line = raw_line.strip()
                    if line.startswith("complete="):
                        self.complete = line.split("=", 1)[1] == "1"
                    elif line.startswith("inspection_time="):
                        self.elapsed_seconds = float(line.split("=", 1)[1])
                    elif line.startswith("record="):
                        fields = line.split("=", 1)[1].split(",")
                        if len(fields) == 4:
                            location = fields[0]
                            shape = fields[1]
                            class_id = int(fields[2])
                            confidence = int(fields[3])
                            if (
                                len(location) == 1
                                and "A" <= location <= "N"
                                and shape in ("SQUARE", "CIRCLE")
                                and 0 <= class_id < len(config.CLASS_NAMES)
                            ):
                                self.records.append(
                                    (location, shape, class_id, confidence)
                                )
            self.records.sort(key=lambda item: item[0])
        except Exception as error:
            print("history load failed:", error)
            self.records = []


class VisionLink:
    def __init__(self):
        self.port = None
        self.rx = bytearray()
        self.next_sequence = 0
        self.last_open_attempt_ms = -config.UART_REOPEN_MS
        self.handled = []
        self.open()

    def open(self):
        self.last_open_attempt_ms = int(now_seconds() * 1000)
        try:
            pinmap.set_pin_function(config.UART_TX_PIN, "UART0_TX")
            pinmap.set_pin_function(config.UART_RX_PIN, "UART0_RX")
            self.port = uart.UART(config.UART_DEVICE, config.UART_BAUD_RATE)
            self.rx = bytearray()
            print("UART ready:", config.UART_DEVICE, config.UART_TX_PIN, config.UART_RX_PIN)
        except Exception as error:
            print("UART open failed:", error)
            self.port = None

    def recover_if_due(self):
        if self.port is not None:
            return
        now_ms = int(now_seconds() * 1000)
        if now_ms - self.last_open_attempt_ms >= config.UART_REOPEN_MS:
            self.open()

    def write(self, data):
        if self.port is None:
            return False
        try:
            self.port.write(data)
            return True
        except Exception as error:
            print("UART write failed:", error)
            self.port = None
            return False

    def send_ack(self, sequence, sender_session):
        self.write(encode_packet("ACK", sequence, sender_session))

    def send_detection(self, session, context_id, class_id, shape, confidence, cx, cy):
        payload = "%s,%d,%d,%s,%d,%d,%d" % (
            session,
            context_id,
            class_id,
            shape,
            confidence,
            max(0, int(cx)),
            max(0, int(cy)),
        )
        packet = encode_packet("DETECTION", self.next_sequence, payload)
        if self.write(packet):
            self.next_sequence = (self.next_sequence + 1) & 0xFFFF

    def poll(self):
        self.recover_if_due()
        if self.port is None:
            return []
        try:
            received = self.port.read()
            if received:
                self.rx.extend(received)
        except Exception as error:
            print("UART read failed:", error)
            self.port = None
            return []
        if len(self.rx) > config.UART_RX_LIMIT:
            newline = self.rx.rfind(b"\n")
            self.rx = self.rx[newline + 1 :] if newline >= 0 else bytearray()

        packets = []
        while True:
            newline = self.rx.find(b"\n")
            if newline < 0:
                break
            line = bytes(self.rx[:newline])
            del self.rx[: newline + 1]
            packet = parse_packet(line)
            if packet is not None:
                packets.append(packet)
        return packets

    def is_duplicate(self, sender_session, sequence):
        key = (sender_session, sequence)
        if key in self.handled:
            return True
        self.handled.append(key)
        if len(self.handled) > 16:
            self.handled.pop(0)
        return False


def select_objects(objects):
    if config.TARGET_CLASS_ID < 0:
        candidates = list(objects)
    else:
        candidates = [
            obj for obj in objects if obj.class_id == config.TARGET_CLASS_ID
        ]
    if not candidates:
        return []
    if config.PROCESS_ALL_GEOMETRY_TARGETS:
        return candidates
    return [max(candidates, key=lambda obj: obj.score)]


def make_display_frame(frame):
    if not config.BINARY_DEBUG_VIEW:
        return frame
    try:
        preview = frame.copy()
        converted = preview.binary(config.DARK_BORDER_THRESHOLDS)
        return converted if converted is not None else preview
    except Exception:
        return frame


def shape_name(geometry):
    if geometry is None:
        return None
    if geometry["shape_id"] == 1:
        return "SQUARE"
    if geometry["shape_id"] == 2:
        return "CIRCLE"
    return None


class InspectionApp:
    def __init__(self):
        self.detector = nn.YOLO11(model=config.MODEL_PATH, dual_buff=config.DUAL_BUFF)
        self.cam = camera.Camera(
            self.detector.input_width(),
            self.detector.input_height(),
            self.detector.input_format(),
        )
        self.disp = display.Display()
        self.link = VisionLink()
        self.history = InspectionHistory()
        self.running = False
        self.started_at = 0.0
        self.location = None
        self.context_id = 0
        self.remote_session = None
        self.vision_session = "%08X" % (int(now_seconds() * 1000) & 0xFFFFFFFF or 1)
        self.candidate = None
        self.candidate_frames = 0
        self.recorded_locations = set(item[0] for item in self.history.records)

    def handle_packet(self, packet_type, sequence, payload):
        fields = payload.split(",")
        if not fields or len(fields[0]) != 8:
            return
        try:
            int(fields[0], 16)
        except ValueError:
            return
        sender_session = fields[0]
        if packet_type not in ("START", "END", "CONTEXT"):
            return
        self.link.send_ack(sequence, sender_session)

        if packet_type == "START":
            if self.running and self.link.is_duplicate(sender_session, sequence):
                return
            self.link.handled = [(sender_session, sequence)]
            self.remote_session = sender_session
            self.running = True
            self.started_at = now_seconds()
            self.location = None
            self.candidate = None
            self.candidate_frames = 0
            self.recorded_locations = set()
            self.history.clear_for_new_run()
            print("inspection start")
            return

        if self.link.is_duplicate(sender_session, sequence):
            return
        if sender_session != self.remote_session:
            return
        if packet_type == "END":
            if self.running:
                self.running = False
                self.history.finish(now_seconds() - self.started_at)
                print("inspection end: %.2fs" % self.history.elapsed_seconds)
            return
        if packet_type == "CONTEXT" and len(fields) == 2:
            try:
                context_id = int(fields[1])
            except ValueError:
                return
            if self.running and 0 <= context_id < 14:
                self.context_id = context_id
                self.location = chr(ord("A") + context_id)
                self.candidate = None
                self.candidate_frames = 0

    def process_uart(self):
        for packet in self.link.poll():
            self.handle_packet(*packet)

    def accept_detection(self, obj, geometry, frame_cx):
        shape = shape_name(geometry)
        if not self.running or self.location is None or shape is None:
            return
        if self.location in self.recorded_locations:
            return
        if abs(geometry["cx"] - frame_cx) > config.DETECTION_CENTER_TOLERANCE_PX:
            return
        candidate = (self.location, shape, int(obj.class_id))
        if candidate == self.candidate:
            self.candidate_frames += 1
        else:
            self.candidate = candidate
            self.candidate_frames = 1
        if self.candidate_frames < config.DETECTION_STABLE_FRAMES:
            return

        confidence = max(0, min(1000, int(round(obj.score * 1000.0))))
        self.history.upsert(self.location, shape, int(obj.class_id), confidence)
        self.recorded_locations.add(self.location)
        self.link.send_detection(
            self.vision_session,
            self.context_id,
            int(obj.class_id),
            shape,
            confidence,
            geometry["cx"],
            geometry["cy"],
        )
        print("record:", self.location, shape, config.CLASS_NAMES[obj.class_id])

    def draw_history(self, frame):
        frame.draw_rect(0, 0, frame.width(), frame.height(), color=BLACK, thickness=-1)
        frame.draw_string(8, 6, "上次巡检", color=WHITE)
        if not self.history.records:
            frame.draw_string(8, 42, "暂无巡检记录", color=YELLOW)
        for index, (location, shape, class_id, _confidence) in enumerate(self.history.records[:14]):
            column = index // 7
            row = index % 7
            x = 8 + column * 156
            y = 34 + row * 24
            shape_cn = "方形" if shape == "SQUARE" else "圆形"
            class_cn = config.CLASS_NAMES_CN[class_id]
            frame.draw_string(x, y, "%s+%s+%s" % (location, shape_cn, class_cn), color=GREEN)
        if self.history.complete:
            frame.draw_string(8, 214, "检+%.1fs" % self.history.elapsed_seconds, color=CYAN)
        else:
            frame.draw_string(8, 214, "等待巡检开始", color=YELLOW)
        if self.link.port is None:
            frame.draw_string(216, 214, "串口故障", color=RED)

    def draw_live(self, frame, objects, selected):
        for obj in objects:
            frame.draw_string(
                obj.x,
                max(0, obj.y - 14),
                self.detector.labels[obj.class_id],
                color=COLORS[obj.class_id % len(COLORS)],
            )
        for obj, geometry, _roi in selected:
            if geometry is not None:
                frame.draw_cross(geometry["cx"], geometry["cy"], color=RED, size=8, thickness=2)
        location = self.location if self.location is not None else "-"
        frame.draw_rect(0, 0, frame.width(), 24, color=BLACK, thickness=-1)
        frame.draw_string(
            2,
            2,
            "巡检 %s  记录:%d  T:%.1fs"
            % (location, len(self.history.records), now_seconds() - self.started_at),
            color=GREEN,
        )

    def run(self):
        frame_cx = self.detector.input_width() // 2
        print("model:", config.MODEL_PATH)
        print("labels:", self.detector.labels)
        while not app.need_exit():
            time.fps_start()
            self.process_uart()
            frame = self.cam.read()
            if frame is None:
                continue
            if not self.running:
                self.draw_history(frame)
                self.disp.show(frame)
                continue

            objects = self.detector.detect(frame, conf_th=config.CONF_TH, iou_th=config.IOU_TH)
            selected = []
            for obj in select_objects(objects):
                geometry, roi = find_geometry_center(frame, obj)
                selected.append((obj, geometry, roi))
                self.accept_detection(obj, geometry, frame_cx)
            display_frame = make_display_frame(frame)
            self.draw_live(display_frame, objects, selected)
            self.disp.show(display_frame)


def main():
    InspectionApp().run()


if __name__ == "__main__":
    main()
