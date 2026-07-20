# -*- coding: utf-8 -*-
"""
bridge.py — جسر Wave Rover (واجهة تجريد real/sim)
==================================================
sim (الافتراضي الآن — الروبوت لم يصل): يحاكي استجابة حركة واقعية، heading
يتبع أوامر اللف، الموقع يتكامل من السرعة. يُهيّأ موقعه من أول قفل GPS حقيقي.
real (M6): بروتوكول Waveshare JSON عبر UART — لاحقاً.

أوامر الحركة: F/B/L/R/S بقوة 0-100 (نظير البروتوكول النصي في legacy).
مزلاج أمان: بعد ROVER_SAFETY_TIMEOUT_S بلا أمر → توقف (نظير مهلة الروفر).
"""
import math
import time

from pi.config import (
    DRIVE_SPEED_MPS, TURN_RATE_DPS, ROVER_SAFETY_TIMEOUT_S,
    SIM_HOME_LAT, SIM_HOME_LNG,
)

_M_PER_DEG = 111320.0                  # تقريب مستوٍ كافٍ للمسافات القصيرة


class RoverBridge:
    def __init__(self, mode: str = "sim"):
        self.mode = mode
        self.lat = SIM_HOME_LAT
        self.lng = SIM_HOME_LNG
        self.heading = 0.0             # درجة، 0 = شمال، مع عقارب الساعة
        self.speed = 0.0               # m/s فعلية (موجب أمام)
        self._dir = "S"
        self._power = 0
        self._last_cmd_ts = 0.0
        self._from_gps = False

    def set_home_from_gps(self, lat: float, lng: float) -> None:
        """تهيئة موقع الروفر من أول قفل GPS حقيقي (مرة واحدة)."""
        if not self._from_gps and lat and lng:
            self.lat, self.lng = lat, lng
            self._from_gps = True

    def command(self, direction: str, power: int = 70) -> None:
        d = (direction or "S").upper()
        if d not in ("F", "B", "L", "R", "S"):
            return
        self._dir = d
        try:
            self._power = max(0, min(100, int(power)))
        except (TypeError, ValueError):
            self._power = 70
        self._last_cmd_ts = time.time()

    def update(self, dt: float) -> None:
        """يتكامل الحركة عبر dt ثانية (تُستدعى من حلقة السيرفر)."""
        if self.mode != "sim":
            return                     # real: يُملأ من تيليمتري UART في M6
        if dt <= 0 or dt > 2.0:
            return
        # مزلاج الأمان: بلا أمر حديث → توقف
        if time.time() - self._last_cmd_ts > ROVER_SAFETY_TIMEOUT_S:
            self._dir = "S"
        p = self._power / 70.0
        if self._dir == "F":
            self.speed = DRIVE_SPEED_MPS * p
            self._advance(self.speed * dt)
        elif self._dir == "B":
            self.speed = -DRIVE_SPEED_MPS * p
            self._advance(self.speed * dt)
        elif self._dir == "L":
            self.heading = (self.heading - TURN_RATE_DPS * p * dt) % 360.0
            self.speed = 0.0
        elif self._dir == "R":
            self.heading = (self.heading + TURN_RATE_DPS * p * dt) % 360.0
            self.speed = 0.0
        else:
            self.speed = 0.0

    def _advance(self, dist_m: float) -> None:
        hd = math.radians(self.heading)
        self.lat += (dist_m * math.cos(hd)) / _M_PER_DEG
        self.lng += (dist_m * math.sin(hd)) / (_M_PER_DEG * math.cos(math.radians(self.lat)))

    def state(self) -> dict:
        return {
            "mode": self.mode,
            "lat": round(self.lat, 6),
            "lng": round(self.lng, 6),
            "heading": round(self.heading, 1),
            "speed": round(self.speed, 2),
            "dir": self._dir,
            "power": self._power,
        }
