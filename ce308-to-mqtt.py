#!/usr/bin/env python3
"""
CE308 -> MQTT -> Home Assistant
Периодический опрос счетчика через CE901 с публикацией в MQTT.
Читает: энергию 1.8.0/1.8.1/1.8.2, напряжение по фазам,
дату последнего изменения конфигурации.
Управление периодом опроса — через MQTT-команды.
"""
import struct
import serial
import time
import json
import os
import signal
import sys
import threading
from datetime import datetime, timedelta, timezone

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("Установите paho-mqtt: pip3 install paho-mqtt")
    sys.exit(1)

# ===================== НАСТРОЙКИ (из env vars / .env) =====================
COM_PORT = os.getenv("COM_PORT", "/dev/ttyUSB0")
BAUDRATE = int(os.getenv("BAUDRATE", "115200"))
TIMEOUT = int(os.getenv("TIMEOUT", "3"))
PASSWORD = os.getenv("PASSWORD", "12345678").encode()
POLL_INTERVAL = int(os.getenv("POLL_INTERVAL", "300"))  # 5 минут по умолчанию

# MQTT настройки
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USER = os.getenv("MQTT_USER", "")
MQTT_PASS = os.getenv("MQTT_PASS", "")
MQTT_TOPIC = os.getenv("MQTT_TOPIC", "energomera/ce308")
MQTT_CLIENT_ID = os.getenv("MQTT_CLIENT_ID", "ce308_poller")

# HA Auto Discovery
HA_DEVICE_ID = os.getenv("HA_DEVICE_ID", "ce308_energy_meter")
HA_DEVICE_NAME = os.getenv("HA_DEVICE_NAME", "Счетчик CE308")

# ===================== DLMS/HDLC =====================
SERVER = bytes([0x00, 0x02, 0x00, 0x21])
CLIENT = 0x41


def parse_obis_list(env_name, default):
    """Переопределение через env: "класс:A.B.C.D.E.F[,класс:A.B.C.D.E.F]"."""
    raw = os.getenv(env_name)
    if not raw:
        return default
    out = []
    for item in raw.split(","):
        cls, code = item.strip().split(":")
        out.append((int(cls), tuple(int(x) for x in code.split("."))))
    return out


# Энергия (class 3, register)
REGS = {
    "1.8.0": (1, 0, 1, 8, 0, 255),
    "1.8.1": (1, 0, 1, 8, 1, 255),
    "1.8.2": (1, 0, 1, 8, 2, 255)
}
# Напряжение по фазам (class 3, register; стандартные коды IEC 62056-6-1)
VOLTAGE_REGS = {
    "L1": (1, 0, 32, 7, 0, 255),
    "L2": (1, 0, 52, 7, 0, 255),
    "L3": (1, 0, 72, 7, 0, 255),
}
# Дата последнего изменения конфигурации. Можно задать OBIS_CONFIG_DATE="1:0.0.96.2.1.255"
CONFIG_DATE_CANDIDATES = parse_obis_list("OBIS_CONFIG_DATE", [
    (1, (0, 0, 96, 2, 1, 255)),
])

UNITS = {27: "Вт", 28: "ВА", 29: "вар", 30: "Вт·ч", 31: "ВА·ч", 32: "вар·ч",
         33: "А", 35: "В", 44: "Гц"}

SNRM = bytes.fromhex("8180140502040006020400070400000001080400000001")
AARQ = bytes.fromhex(
    "E6E600" "6036A109060760857405080101" "8A0207808B0760857405080201"
    "AC0A8008" + PASSWORD.hex() + "BE10040E01000000065F1F0400FFFFFF0800"
)


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc ^ 0xFFFF


def build_frame(ctrl: int, info: bytes = b"") -> bytes:
    length = 2 + len(SERVER) + 1 + 1 + (2 + len(info) if info else 0) + 2
    body = bytes([0xA0 | (length >> 8), length & 0xFF]) + SERVER + bytes([CLIENT, ctrl])
    if info:
        body += struct.pack("<H", crc16(body)) + info
    return b"\x7e" + body + struct.pack("<H", crc16(body)) + b"\x7e"


def check_frame(f: bytes) -> bool:
    return len(f) >= 9 and crc16(f[1:-3]) == struct.unpack("<H", f[-3:-1])[0]


def frame_info(f: bytes) -> bytes:
    pos = 3
    for _ in range(2):
        while not (f[pos] & 1):
            pos += 1
        pos += 1
    pos += 1
    end = len(f) - 3
    return f[pos + 2:end] if end > pos + 2 else b""


def get_request(cls: int, obis: tuple, attr: int) -> bytes:
    return b"\xe6\xe6\x00\xc0\x01\x40" + struct.pack(">H", cls) + bytes(obis) + bytes([attr, 0])


def decode_data(b: bytes, p: int):
    t = b[p]; p += 1
    if t == 0:                                    # null-data
        return None, p
    if t in (1, 2):                               # array / structure
        n = b[p]; p += 1; out = []
        for _ in range(n):
            v, p = decode_data(b, p); out.append(v)
        return out, p
    fmt = {5: ">i", 6: ">I", 15: ">b", 16: ">h", 17: ">B", 18: ">H",
           20: ">q", 21: ">Q", 22: ">B", 23: ">f", 24: ">d"}
    if t in fmt:
        n = struct.calcsize(fmt[t])
        return struct.unpack(fmt[t], b[p:p + n])[0], p + n
    if t in (9, 10, 12):                          # octet-string / visible-string / utf8
        n = b[p]; p += 1
        return bytes(b[p:p + n]), p + n
    if t == 25:                                   # date-time (12 байт)
        return bytes(b[p:p + 12]), p + 12
    if t == 26:                                   # date (5 байт)
        return bytes(b[p:p + 5]), p + 5
    raise ValueError(f"тип DLMS 0x{t:02X} не поддержан")


def decode_get_response(info: bytes):
    if info[:6] != b"\xe6\xe7\x00\xc4\x01\x40":
        raise ValueError("не GET-response: " + info.hex(" "))
    if info[6] != 0:
        raise ValueError(f"ошибка доступа, код {info[7]}")
    return decode_data(info, 7)[0]


def decode_datetime(v):
    """COSEM date-time (12 байт) или date (5 байт) -> ISO-строка с часовым поясом."""
    if not isinstance(v, (bytes, bytearray)) or len(v) not in (5, 12):
        return None
    year, month, day = struct.unpack(">HBB", bytes(v[:4]))
    hour = minute = sec = 0
    dev = -32768
    if len(v) == 12:
        hour, minute, sec = v[5], v[6], v[7]
        dev = struct.unpack(">h", bytes(v[9:11]))[0]
    if year == 0xFFFF or not 1 <= month <= 12 or not 1 <= day <= 31:
        return None
    hour, minute, sec = (x if x < 60 else 0 for x in (hour, minute, sec))
    try:
        if dev == -32768:                         # отклонение не задано -> локальное время
            dt = datetime(year, month, day, hour, minute, sec).astimezone()
        else:                                     # deviation = локальное - UTC, в минутах
            dt = datetime(year, month, day, hour, minute, sec,
                          tzinfo=timezone(timedelta(minutes=-dev)))
    except ValueError:
        return None
    return dt.isoformat(timespec="seconds")


_scaler_cache = {}      # (obis) -> (scaler, unit): множитель не меняется, читаем один раз


def read_meter(port: str = COM_PORT) -> dict:
    """Один сеанс связи: энергия (обязательно) + напряжение и дата изменения
    конфигурации (необязательно, ошибки не прерывают опрос)."""
    ser = serial.Serial(port, BAUDRATE, serial.EIGHTBITS, serial.PARITY_NONE,
                        serial.STOPBITS_ONE, timeout=TIMEOUT)
    ns = 0

    def xchg(frame: bytes) -> bytes:
        ser.reset_input_buffer()
        ser.write(frame)
        buf = b""
        while True:
            ch = ser.read(1)
            if not ch:
                break
            buf += ch
            if len(buf) >= 3 and buf[0] == 0x7E:
                if len(buf) >= (((buf[1] & 7) << 8) | buf[2]) + 2:
                    break
        if not check_frame(buf):
            raise IOError("нет или повреждён ответ: " + buf.hex(" "))
        return buf

    def i_frame(apdu: bytes) -> bytes:
        nonlocal ns
        ctrl = ((ns & 7) << 5) | 0x10 | ((ns & 7) << 1)
        ns += 1
        return build_frame(ctrl, apdu)

    def get(cls, obis, attr):
        return decode_get_response(frame_info(xchg(i_frame(get_request(cls, obis, attr)))))

    def get_register(obis):
        if obis not in _scaler_cache:
            sc = get(3, obis, 3)
            _scaler_cache[obis] = (sc[0], sc[1])
        raw = get(3, obis, 2)
        scaler, unit = _scaler_cache[obis]
        return raw * 10 ** scaler, UNITS.get(unit, str(unit))

    def optional(label, fn):
        """Ошибка объекта (нет в счётчике и т.п.) не должна ломать весь опрос."""
        try:
            return fn()
        except (ValueError, IndexError, struct.error) as e:
            print(f"  ⚠️ {label}: {e}")
            return None

    def first_of(candidates, attr, decoder, label):
        for cls, obis in candidates:
            v = optional(f"{label} {'.'.join(map(str, obis))}", lambda: get(cls, obis, attr))
            if v is not None:
                v = decoder(v)
                if v:
                    return v
        return None

    result = {"energy": {}, "voltage": {}, "config_date": None}
    try:
        xchg(build_frame(0x93, SNRM))
        resp = xchg(i_frame(AARQ))
        if b"\xa2\x03\x02\x01\x00" not in resp:
            raise IOError("авторизация отклонена: " + resp.hex(" "))

        # --- обязательная часть: энергия (ошибка = ошибка опроса)
        for name, obis in REGS.items():
            result["energy"][name] = get_register(obis)

        # --- необязательная часть
        try:
            for phase, obis in VOLTAGE_REGS.items():
                v = optional(f"напряжение {phase}", lambda: get_register(obis))
                if v is not None:
                    result["voltage"][phase] = v[0]
            result["config_date"] = first_of(CONFIG_DATE_CANDIDATES, 2, decode_datetime,
                                             "дата изменения конфигурации")
        except OSError as e:                      # потеряна связь — отдаём то, что успели
            print(f"  ⚠️ связь потеряна при чтении доп. параметров: {e}")
    finally:
        try:
            xchg(build_frame(0x53))
        except Exception:
            pass
        ser.close()
    return result


# ===================== MQTT =====================
running = True
poll_interval = max(10, min(3600, POLL_INTERVAL))   # меняется командой из MQTT/HA
poll_event = threading.Event()        # «опросить сейчас» / завершение
connected = threading.Event()

T_AVAIL = f"{MQTT_TOPIC}/availability"
T_STATUS = f"{MQTT_TOPIC}/status"
T_LAST = f"{MQTT_TOPIC}/last_update"
T_CONFIG_DATE = f"{MQTT_TOPIC}/config_change"
T_INTERVAL = f"{MQTT_TOPIC}/status/poll_interval"
T_SET_INTERVAL = f"{MQTT_TOPIC}/set/poll_interval"
T_SET_NOW = f"{MQTT_TOPIC}/set/poll_now"
DISCOVERY_PREFIX = "homeassistant"


def now_iso() -> str:
    """ISO-время с часовым поясом (обязательно для device_class timestamp в HA)."""
    return datetime.now().astimezone().isoformat(timespec="seconds")


def signal_handler(sig, frame):
    global running
    print("\nПолучен сигнал завершения, остановка...")
    running = False
    poll_event.set()


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def publish_interval_state(client):
    client.publish(T_INTERVAL, json.dumps({"interval": poll_interval}), retain=True)


def publish_discovery(client: mqtt.Client):
    """MQTT Auto Discovery: энергия, напряжение по фазам,
    дата изменения конфигурации, период опроса (number), кнопка «опросить сейчас»,
    статус и время последнего обновления."""
    device = {
        "identifiers": [HA_DEVICE_ID],
        "name": HA_DEVICE_NAME,
        "manufacturer": "Энергомера",
        "model": "CE308",
        "sw_version": "DLMS/COSEM",
    }
    common = {"device": device, "availability_topic": T_AVAIL}

    def energy(name, obis):
        return ("sensor", {
            "name": name,
            "unique_id": f"{HA_DEVICE_ID}_{obis.replace('.', '_')}",
            "state_topic": f"{MQTT_TOPIC}/{obis}",
            "value_template": "{{ value_json.value | float }}",
            # ВАЖНО: для device_class energy HA принимает только "kWh", а не "кВт·ч"
            "unit_of_measurement": "kWh",
            "device_class": "energy",
            "state_class": "total_increasing",
            "suggested_display_precision": 2,
        })

    def voltage(phase):
        return ("sensor", {
            "name": f"Напряжение {phase}",
            "unique_id": f"{HA_DEVICE_ID}_voltage_{phase.lower()}",
            "state_topic": f"{MQTT_TOPIC}/voltage/{phase}",
            "value_template": "{{ value_json.value | float }}",
            "unit_of_measurement": "V",
            "device_class": "voltage",
            "state_class": "measurement",
            "suggested_display_precision": 1,
        })

    entities = [
        energy("Активная энергия (Всего)", "1.8.0"),
        energy("Активная энергия (Тариф 1)", "1.8.1"),
        energy("Активная энергия (Тариф 2)", "1.8.2"),
        voltage("L1"),
        voltage("L2"),
        voltage("L3"),
        ("sensor", {
            "name": "Изменение конфигурации",
            "unique_id": f"{HA_DEVICE_ID}_config_change",
            "state_topic": T_CONFIG_DATE,
            "value_template": "{{ value_json.value }}",
            "device_class": "timestamp",
            "icon": "mdi:cog-clock",
            "entity_category": "diagnostic",
        }),
        ("number", {
            "name": "Период опроса",
            "unique_id": f"{HA_DEVICE_ID}_poll_interval_set",
            "command_topic": T_SET_INTERVAL,
            "state_topic": T_INTERVAL,
            "value_template": "{{ value_json.interval | int }}",
            "min": 10, "max": 3600, "step": 10,
            "mode": "box",
            "unit_of_measurement": "сек",
            "icon": "mdi:timer-cog",
            "retain": True,              # значение переживёт перезапуск скрипта
            "entity_category": "config",
        }),
        ("button", {
            "name": "Опросить сейчас",
            "unique_id": f"{HA_DEVICE_ID}_poll_now",
            "command_topic": T_SET_NOW,
            "payload_press": "1",
            "icon": "mdi:refresh",
        }),
        ("sensor", {
            "name": "Статус опроса",
            "unique_id": f"{HA_DEVICE_ID}_status",
            "state_topic": T_STATUS,
            "value_template": "{{ value_json.status }}",
            "json_attributes_topic": T_STATUS,
            "icon": "mdi:check-network",
            "entity_category": "diagnostic",
        }),
        ("sensor", {
            "name": "Последнее обновление",
            "unique_id": f"{HA_DEVICE_ID}_last_update",
            "state_topic": T_LAST,
            "device_class": "timestamp",
            "entity_category": "diagnostic",
        }),
    ]
    for component, cfg in entities:
        topic = f"{DISCOVERY_PREFIX}/{component}/{HA_DEVICE_ID}/{cfg['unique_id']}/config"
        client.publish(topic, json.dumps({**common, **cfg}), retain=True)
        print(f"  discovery [{component}]: {cfg['name']}")

    # удалить устаревшие сенсоры («Период опроса» теперь number, серийный номер не опрашивается)
    for old in ("poll_interval", "serial"):
        client.publish(f"{DISCOVERY_PREFIX}/sensor/{HA_DEVICE_ID}/{HA_DEVICE_ID}_{old}/config",
                       "", retain=True)


def on_connect(client, userdata, flags, reason_code, properties=None):
    failed = getattr(reason_code, "is_failure", None)
    if failed is None:
        failed = reason_code != 0
    if failed:
        print(f"❌ Ошибка подключения к MQTT: {reason_code}")
        return
    print(f"✅ Подключено к MQTT брокеру: {MQTT_BROKER}:{MQTT_PORT}")
    publish_discovery(client)
    publish_interval_state(client)
    client.publish(T_AVAIL, "online", retain=True)
    client.subscribe([(T_SET_INTERVAL, 0), (T_SET_NOW, 0)])
    print(f"  Подписка на команды: {MQTT_TOPIC}/set/#")
    connected.set()


def on_message(client, userdata, msg):
    """MQTT-команды: set/poll_interval (сек, 10..3600) и set/poll_now (1)."""
    global poll_interval
    payload = msg.payload.decode(errors="ignore").strip()
    print(f"\n📥 Команда: {msg.topic} = {payload}")

    if msg.topic == T_SET_INTERVAL:
        try:
            value = int(float(payload))
        except ValueError:
            print("  ❌ Ожидалось число")
            return
        if not 10 <= value <= 3600:
            print("  ❌ Значение вне диапазона (10-3600 сек)")
            publish_interval_state(client)      # вернуть в HA актуальное значение
            return
        poll_interval = value
        print(f"  ✅ Период опроса: {value} сек")
        publish_interval_state(client)
    elif msg.topic == T_SET_NOW:
        if payload.lower() in ("1", "true", "yes", "on", "press"):
            print("  ✅ Немедленный опрос")
            poll_event.set()


def poll_and_publish(client: mqtt.Client):
    try:
        print(f"\n[{datetime.now():%Y-%m-%d %H:%M:%S}] Опрос счетчика...")
        data = read_meter()
        ts = now_iso()

        for obis, (value, unit) in data["energy"].items():
            kwh = value / 1000.0                  # значение со счётчика — в Вт·ч
            payload = {"value": round(kwh, 5), "unit": "kWh",
                       "raw_value": value, "raw_unit": unit, "timestamp": ts}
            client.publish(f"{MQTT_TOPIC}/{obis}", json.dumps(payload), retain=True)
            print(f"  {obis}: {kwh:.5f} кВт·ч -> MQTT")

        for phase, volts in data["voltage"].items():
            payload = {"value": round(volts, 2), "unit": "V", "timestamp": ts}
            client.publish(f"{MQTT_TOPIC}/voltage/{phase}", json.dumps(payload), retain=True)
            print(f"  U {phase}: {volts:.2f} В -> MQTT")

        if data["config_date"]:
            client.publish(T_CONFIG_DATE, json.dumps({"value": data["config_date"]}), retain=True)
            print(f"  Изменение конфигурации: {data['config_date']}")

        client.publish(T_STATUS, json.dumps(
            {"status": "ok", "timestamp": ts, "port": COM_PORT,
             "poll_interval": poll_interval}), retain=True)
        client.publish(T_LAST, ts, retain=True)
        print(f"  Следующий опрос через {poll_interval} сек...")
    except Exception as e:
        print(f"  ❌ Ошибка опроса: {e}")
        client.publish(T_STATUS, json.dumps(
            {"status": "error", "error": str(e), "timestamp": now_iso(),
             "port": COM_PORT, "poll_interval": poll_interval}), retain=True)


def main():
    print("=" * 70)
    print("CE308 -> MQTT -> Home Assistant")
    print("=" * 70)
    print(f"COM-порт: {COM_PORT}")
    print(f"MQTT брокер: {MQTT_BROKER}:{MQTT_PORT}")
    print(f"Интервал опроса: {poll_interval} сек (по умолчанию)")
    print("=" * 70)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=MQTT_CLIENT_ID)
    client.on_connect = on_connect
    client.on_message = on_message
    if MQTT_USER:
        client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.will_set(T_AVAIL, "offline", retain=True)

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
    except Exception as e:
        print(f"Ошибка подключения к MQTT: {e}")
        sys.exit(1)

    if connected.wait(10):
        time.sleep(1)       # дать пройти retained-команде периода опроса
    else:
        print("⚠️ MQTT пока не подключен, продолжаю (paho переподключится сам)")

    while running:
        poll_and_publish(client)
        poll_event.clear()
        last = time.monotonic()
        # ждём следующий опрос; изменение периода действует в пределах секунды
        while running and time.monotonic() - last < poll_interval:
            if poll_event.wait(timeout=1.0):
                break

    print("\nОстановка MQTT клиента...")
    client.publish(T_AVAIL, "offline", retain=True).wait_for_publish(2)
    client.loop_stop()
    client.disconnect()
    print("✅ Завершено")


if __name__ == "__main__":
    main()
