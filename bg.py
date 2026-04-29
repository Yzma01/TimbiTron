import argparse
import glob
import os

from brachiograph import BrachioGraph


servo_1_angle_pws1 = [
    [-162, 2470],
    [-144, 2250],
    [-126, 2050],
    [-108, 1860],
    [-90, 1690],
    [-72, 1530],
    [-54, 1350],
    [-36, 1190],
    [-18, 1010],
    [0, 840],
    [18, 640],
]

servo_2_angle_pws2 = [
    [0, 660],
    [18, 840],
    [36, 1030],
    [54, 1180],
    [72, 1340],
    [90, 1490],
    [108, 1640],
    [126, 1830],
    [144, 2000],
    [162, 2200],
    [180, 2410],
]

DEFAULT_LINUX_PORTS = (
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/serial/by-id/*",
)
DEFAULT_WIFI_HOST = "brachiograph-esp32.local"
DEFAULT_WIFI_PORT = 8888


def detect_serial_port():
    env_port = os.environ.get("BGRAPH_SERIAL_PORT")
    if env_port:
        return env_port

    for pattern in DEFAULT_LINUX_PORTS:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]

    return None


def detect_wifi_host():
    return os.environ.get("BGRAPH_ESP32_HOST") or DEFAULT_WIFI_HOST


def build_bg(serial_port=None, virtual=False, esp32_host=None, esp32_port=DEFAULT_WIFI_PORT):
    chosen_port = None if virtual else (serial_port or detect_serial_port())
    chosen_host = None if virtual else esp32_host

    if virtual:
        print("Starting BrachioGraph in virtual mode")
    elif chosen_host:
        print(f"Using ESP32 over WiFi: {chosen_host}:{esp32_port}")
    elif chosen_port:
        print(f"Using serial port: {chosen_port}")
    else:
        print("No Linux serial device found; falling back to virtual mode")

    return BrachioGraph(
        inner_arm=8,
        outer_arm=8,
        bounds=(-8, 4, 8, 13),
        servo_1_angle_pws=servo_1_angle_pws1,
        servo_2_angle_pws=servo_2_angle_pws2,
        pw_down=1200,
        pw_up=1850,
        serial_port=chosen_port,
        esp32_host=chosen_host,
        esp32_port=esp32_port,
        virtual=virtual,
    )


def parse_args():
    parser = argparse.ArgumentParser(description="Start the BrachioGraph backend")
    parser.add_argument(
        "--port",
        default=os.environ.get("BGRAPH_SERIAL_PORT"),
        help="Serial port for the ESP32, e.g. /dev/ttyUSB0 or /dev/ttyACM0",
    )
    parser.add_argument(
        "--virtual",
        action="store_true",
        help="Force virtual mode without trying to open a serial port",
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("BGRAPH_ESP32_HOST"),
        help="ESP32 WiFi host or IP, e.g. 192.168.1.50",
    )
    parser.add_argument(
        "--tcp-port",
        type=int,
        default=int(os.environ.get("BGRAPH_ESP32_PORT", DEFAULT_WIFI_PORT)),
        help="ESP32 WiFi TCP port",
    )
    return parser.parse_args()


def main():
    global bg
    args = parse_args()
    bg = build_bg(
        serial_port=args.port,
        virtual=args.virtual,
        esp32_host=args.host,
        esp32_port=args.tcp_port,
    )
    return bg


bg = None


if __name__ == "__main__":
    bg = main()
