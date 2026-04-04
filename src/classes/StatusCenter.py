import glob
import os
import socket
import threading
import time
from collections import deque
from typing import Any, Deque, Dict, Optional

try:
    import adafruit_ssd1306
    from PIL import Image, ImageDraw, ImageFont

    OLED_LIBS_AVAILABLE = True
except Exception:
    adafruit_ssd1306 = None  # type: ignore
    Image = None  # type: ignore
    ImageDraw = None  # type: ignore
    ImageFont = None  # type: ignore
    OLED_LIBS_AVAILABLE = False


class StatusCenter:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if getattr(self, "_initialized", False):
            return

        self._initialized = True
        self._lock = threading.Lock()

        self._last_emit: Dict[str, float] = {}
        self._recent_messages: Deque[str] = deque(maxlen=10)

        self._metrics: Dict[str, Any] = {}
        self._last_metrics_t = 0.0
        self._last_oled_t = 0.0

        self._oled = None
        self._oled_width = 128
        self._oled_height = 64
        self._oled_font = None
        self._oled_ready = False
        self._oled_attempted = False

        self._hostname = socket.gethostname()
        self._latest_robot: Optional[Any] = None
        self._worker_started = False
        self._worker_interval_s = 0.1

        # Keep health collection alive for instant startup and continuous refresh.
        self.start_background_updates()

    def start_background_updates(self) -> None:
        with self._lock:
            if self._worker_started:
                return
            self._worker_started = True

        thread = threading.Thread(
            target=self._background_worker,
            name="status-center-worker",
            daemon=True,
        )
        thread.start()

    def _background_worker(self) -> None:
        while True:
            with self._lock:
                robot = self._latest_robot
                oled_ready = self._oled_ready
                last_metrics_t = self._last_metrics_t
                last_oled_t = self._last_oled_t

            now = time.monotonic()
            if now - last_metrics_t >= 1.0:
                metrics = self._collect_metrics(robot)
                with self._lock:
                    self._metrics = metrics
                    self._last_metrics_t = now

            if oled_ready and now - last_oled_t >= 0.5:
                self._render_oled(robot)
                with self._lock:
                    self._last_oled_t = now

            time.sleep(self._worker_interval_s)

    def log(
        self,
        topic: str,
        message: str,
        level: str = "INFO",
        cooldown_s: float = 0.0,
        cooldown_key: Optional[str] = None,
        force: bool = False,
    ) -> bool:
        now = time.monotonic()
        key = cooldown_key if cooldown_key is not None else f"{level}|{topic}|{message}"

        with self._lock:
            if not force and cooldown_s > 0.0:
                last = self._last_emit.get(key, 0.0)
                if now - last < cooldown_s:
                    return False
            self._last_emit[key] = now

            stamp = time.strftime("%H:%M:%S")
            print(f"[{stamp}] [{level}] [{topic}] {message}", flush=True)
            compact = f"{topic}:{message}"
            if len(compact) > 48:
                compact = compact[:47] + ">"
            self._recent_messages.append(compact)

        return True

    def attach_oled(
        self,
        mux: Optional[Any] = None,
        channel: int = 5,
        address: int = 0x3D,
        width: int = 128,
        height: int = 64,
    ) -> None:
        if self._oled_ready:
            return
        if self._oled_attempted:
            return

        self._oled_attempted = True
        self._oled_width = width
        self._oled_height = height

        if not OLED_LIBS_AVAILABLE:
            self.log(
                "OLED",
                "OLED libraries not installed; display updates disabled",
                level="WARN",
                force=True,
            )
            return

        addresses: list[int] = []
        for addr in [address, 0x3C, 0x3D]:
            if addr not in addresses:
                addresses.append(addr)

        channels: list[int] = [channel]
        for ch in range(8):
            if ch not in channels:
                channels.append(ch)

        last_error: Optional[Exception] = None
        for ch in channels:
            try:
                if mux is not None:
                    i2c_bus = mux[ch]
                else:
                    import board

                    i2c_bus = board.I2C()
            except Exception as e:
                last_error = e
                continue

            for addr in addresses:
                try:
                    self._oled = adafruit_ssd1306.SSD1306_I2C(  # type: ignore[operator]
                        width, height, i2c_bus, addr=addr
                    )
                    self._oled.fill(0)
                    self._oled.show()
                    self._oled_font = (
                        ImageFont.load_default() if ImageFont is not None else None
                    )
                    self._oled_ready = True
                    self.log(
                        "OLED",
                        f"Display online on mux channel {ch}, addr 0x{addr:02X}",
                        force=True,
                    )
                    self._metrics = self._collect_metrics(None)
                    self._render_oled(None)
                    self.start_background_updates()
                    return
                except Exception as e:
                    last_error = e
                    continue

        if last_error is not None:
            self.log(
                "OLED",
                f"Display init failed after I2C scan: {last_error}",
                level="WARN",
                force=True,
            )
        else:
            self.log(
                "OLED",
                "Display init failed after I2C scan (no device found)",
                level="WARN",
                force=True,
            )

    def update(self, robot: Optional[Any] = None) -> None:
        if robot is not None:
            with self._lock:
                self._latest_robot = robot

    def _collect_metrics(self, robot: Optional[Any]) -> Dict[str, Any]:
        cpu_load = self._read_cpu_load_percent()
        ram_used = self._read_ram_used_percent()
        temp_by_type = self._read_temperatures()
        cpu_temp = self._pick_temperature(temp_by_type, ("cpu", "soc"))
        gpu_temp = self._pick_temperature(temp_by_type, ("gpu",))
        max_temp = max(temp_by_type.values()) if temp_by_type else None
        clock_ratio = self._read_cpu_clock_ratio()
        thermal_state = self._classify_thermal_state(max_temp, clock_ratio, cpu_load)

        metrics: Dict[str, Any] = {
            "hostname": self._hostname,
            "cpu_load_pct": cpu_load,
            "ram_used_pct": ram_used,
            "cpu_temp_c": cpu_temp,
            "gpu_temp_c": gpu_temp,
            "max_temp_c": max_temp,
            "cpu_clk_ratio": clock_ratio,
            "thermal_state": thermal_state,
            "uptime_s": self._read_uptime_seconds(),
        }

        if robot is not None:
            try:
                metrics["yaw"] = float(getattr(robot, "yaw", 0.0))
                metrics["line_size"] = float(getattr(robot, "line_size", 0.0))
                metrics["obs"] = bool(getattr(robot, "obs_detected", False))
                metrics["loop_fps"] = float(getattr(robot, "_latest_fps", 0.0))
                metrics["line_cam_fps"] = float(getattr(robot, "line_cam_fps", 0.0))
                metrics["line_cam_read_ms"] = float(getattr(robot, "line_cam_read_ms", 0.0))
                metrics["line_cam_fail_count"] = int(getattr(robot, "line_cam_fail_count", 0))
                front = list(getattr(robot, "front_distances", []))
                side = list(getattr(robot, "side_distances", []))
                metrics["front_min"] = min(front) if front else None
                metrics["side_min"] = min(side) if side else None
            except Exception:
                pass

        return metrics

    def _render_oled(self, robot: Optional[Any]) -> None:
        if not self._oled_ready or self._oled is None:
            return
        if Image is None or ImageDraw is None:
            return

        with self._lock:
            metrics = dict(self._metrics)

        img = Image.new("1", (self._oled_width, self._oled_height))
        draw = ImageDraw.Draw(img)
        font = self._oled_font

        cpu_load = self._fmt_pct(metrics.get("cpu_load_pct"))
        ram_used = self._fmt_pct(metrics.get("ram_used_pct"))
        max_temp = self._fmt_temp(metrics.get("max_temp_c"))
        clk_ratio = metrics.get("cpu_clk_ratio")
        thermal_state = str(metrics.get("thermal_state", "UNK"))
        heat_state = self._describe_heat(thermal_state)
        speed_text = self._describe_cpu_speed(clk_ratio)
        cam_fps = self._fmt_float(metrics.get("line_cam_fps"), 1)
        loop_fps = self._fmt_float(metrics.get("loop_fps"), 1)
        read_ms = self._fmt_float(metrics.get("line_cam_read_ms"), 0)
        drop_cnt = self._fmt_int(metrics.get("line_cam_fail_count"))
        line_size = self._fmt_float(metrics.get("line_size"), 0)

        line0 = f"{time.strftime('%H:%M:%S')} Run Monitor"
        line1 = f"Heat:{heat_state:<5} Max:{max_temp}"
        line2 = f"CPU use:{cpu_load:<5} RAM:{ram_used}"
        line3 = f"CPU speed:{speed_text}"
        line4 = self._fit_text(f"CamFPS:{cam_fps} Loop:{loop_fps}", 21)
        line5 = self._fit_text(f"Read:{read_ms}ms Drop:{drop_cnt} Ln:{line_size}", 21)

        draw.text((0, 0), line0, fill=255, font=font)
        draw.text((0, 10), line1, fill=255, font=font)
        draw.text((0, 20), line2, fill=255, font=font)
        draw.text((0, 30), line3, fill=255, font=font)
        draw.text((0, 42), line4, fill=255, font=font)
        draw.text((0, 52), line5, fill=255, font=font)

        try:
            self._oled.image(img)
            self._oled.show()
        except Exception as e:
            self._oled_ready = False
            self.log("OLED", f"Display write failed: {e}", level="WARN", force=True)

    def _read_cpu_load_percent(self) -> float:
        try:
            load1, _, _ = os.getloadavg()
            cpu_count = max(1, os.cpu_count() or 1)
            return max(0.0, min(100.0, (load1 / cpu_count) * 100.0))
        except Exception:
            return 0.0

    def _read_ram_used_percent(self) -> float:
        meminfo = self._read_meminfo()
        total = meminfo.get("MemTotal", 0.0)
        avail = meminfo.get("MemAvailable", 0.0)
        if total <= 0:
            return 0.0
        used = max(0.0, total - avail)
        return max(0.0, min(100.0, (used / total) * 100.0))

    def _read_meminfo(self) -> Dict[str, float]:
        data: Dict[str, float] = {}
        try:
            with open("/proc/meminfo", "r", encoding="utf-8") as f:
                for raw in f:
                    parts = raw.split(":")
                    if len(parts) < 2:
                        continue
                    key = parts[0].strip()
                    value = parts[1].strip().split()[0]
                    data[key] = float(value)
        except Exception:
            pass
        return data

    def _read_temperatures(self) -> Dict[str, float]:
        temps: Dict[str, float] = {}
        for zone in glob.glob("/sys/class/thermal/thermal_zone*"):
            try:
                with open(f"{zone}/type", "r", encoding="utf-8") as tf:
                    tname = tf.read().strip().lower()
                with open(f"{zone}/temp", "r", encoding="utf-8") as vf:
                    raw = float(vf.read().strip())
                value = raw / 1000.0 if raw > 1000 else raw
                if value > -100 and value < 200:
                    if tname in temps:
                        temps[tname] = max(temps[tname], value)
                    else:
                        temps[tname] = value
            except Exception:
                continue
        return temps

    def _pick_temperature(self, temp_map: Dict[str, float], keys: tuple[str, ...]) -> Optional[float]:
        for key in keys:
            for name, value in temp_map.items():
                if key in name:
                    return value
        return None

    def _read_cpu_clock_ratio(self) -> Optional[float]:
        candidates_cur = [
            "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq",
            "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_cur_freq",
        ]
        candidates_max = ["/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq"]

        cur = self._read_first_float(candidates_cur)
        maxf = self._read_first_float(candidates_max)

        if cur is None or maxf is None or maxf <= 0:
            return None
        return max(0.0, min(1.0, cur / maxf))

    def _read_first_float(self, paths: list[str]) -> Optional[float]:
        for path in paths:
            try:
                with open(path, "r", encoding="utf-8") as f:
                    return float(f.read().strip())
            except Exception:
                continue
        return None

    def _classify_thermal_state(
        self, max_temp: Optional[float], clock_ratio: Optional[float], cpu_load: float
    ) -> str:
        if max_temp is None:
            return "UNK"
        if max_temp >= 85.0:
            state = "HOT"
        elif max_temp >= 78.0:
            state = "WARM"
        else:
            state = "OK"

        if clock_ratio is not None and clock_ratio < 0.65 and cpu_load > 60.0:
            state += "/THR?"
        return state

    def _read_uptime_seconds(self) -> float:
        try:
            with open("/proc/uptime", "r", encoding="utf-8") as f:
                return float(f.read().split()[0])
        except Exception:
            return 0.0

    def _fmt_temp(self, value: Optional[float]) -> str:
        if value is None:
            return "N/A"
        return f"{value:4.1f}C"

    def _fmt_pct(self, value: Any) -> str:
        try:
            num = float(value)
            return f"{num:.0f}%"
        except Exception:
            return "N/A"

    def _fmt_float(self, value: Any, decimals: int) -> str:
        try:
            num = float(value)
            return f"{num:.{decimals}f}"
        except Exception:
            return "N/A"

    def _fmt_int(self, value: Any) -> str:
        try:
            num = int(value)
            return str(num)
        except Exception:
            return "N/A"

    def _describe_heat(self, thermal_state: str) -> str:
        if thermal_state.startswith("HOT"):
            return "HOT"
        if thermal_state.startswith("WARM"):
            return "WARM"
        if thermal_state.startswith("OK"):
            return "COOL"
        return "UNK"

    def _describe_cpu_speed(self, ratio: Any) -> str:
        if ratio is None:
            return "N/A"
        try:
            pct = int(max(0.0, min(1.0, float(ratio))) * 100.0)
        except Exception:
            return "N/A"

        if pct >= 95:
            level = "FULL"
        elif pct >= 80:
            level = "HIGH"
        elif pct >= 65:
            level = "MID"
        else:
            level = "LOW"
        return f"{pct}% {level}"

    def _fit_text(self, text: str, max_chars: int) -> str:
        if len(text) <= max_chars:
            return text
        return text[: max_chars - 1] + ">"


_status_center: Optional[StatusCenter] = None


def get_status_center() -> StatusCenter:
    global _status_center
    if _status_center is None:
        _status_center = StatusCenter()
    return _status_center
