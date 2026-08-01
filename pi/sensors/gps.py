# -*- coding: utf-8 -*-
"""
gps.py — قارئ GPS (منقول من pi/tests/test_gps.py المثبت)
=========================================================
pyserial + pynmea2 على /dev/serial0. القراءة في خيط خلفي (readline حاجب)
يحدّث حالة مشتركة؛ state() لقطة غير حاجبة لحلقة السيرفر.

⚠ **لا يُفتح على منفذ الروفر** (docs/wiring.md §«تعارض UART»): قارئان على
   نفس الـUART يتخاطفان الأسطر — خيط الـGPS هنا يقرأ ردود الروفر ويرميها
   (ليست NMEA) فتضيع على الجسر، وهو التفسير المباشر لاختفاء حقل الجهد `v`
   من T=130 الذي عُطّلت بسببه مراقبة البطارية. الأولوية للروفر: هو طبقة
   سلامة، والـGPS بلا فائدة داخل المباني أصلاً.
"""
import os
import threading
import time

from pi.config import GPS_PORT, GPS_BAUD, ROVER_PORT, ROVER_MODE

try:
    import serial
    import pynmea2
    _GPS_LIBS_OK = True
except Exception:                     # noqa: BLE001
    _GPS_LIBS_OK = False


def _same_device(a: str, b: str) -> bool:
    """هل المساران لنفس جهاز TTY فعلياً؟ (يفكّ الوصلات الرمزية)."""
    try:
        return os.path.realpath(a) == os.path.realpath(b)
    except Exception:                 # noqa: BLE001
        return a == b


class GPSReader:
    def __init__(self, port: str = GPS_PORT, baud: int = GPS_BAUD):
        self.ok = False
        self.error = None
        self._lock = threading.Lock()
        self._fix = False
        self._lat = 0.0
        self._lng = 0.0
        self._sats = 0
        self._hdop = 0.0
        self._last_fix_ts = 0.0
        self._ser = None
        self._stop = False

        if not _GPS_LIBS_OK:
            self.error = "pyserial/pynmea2 غير مثبّت"
            return
        # ⚠ التعارض يُفحص **قبل** الفتح: بعده يكون الضرر وقع (خيط يقرأ ويبتلع).
        # يُقارَن المسار الحقيقي لا النصّي — /dev/serial0 وصلة رمزية إلى
        # ttyAMA0/ttyS0، فمقارنة الأسماء وحدها تفوّت التعارض نفسه.
        if ROVER_MODE == "real" and _same_device(port, ROVER_PORT):
            self.error = (f"⛔ معطَّل: {port} هو منفذ الروفر نفسه (تعارض UART "
                          f"موثّق في docs/wiring.md). قارئان على منفذ واحد "
                          f"يتخاطفان ردود الروفر (T:1001/1002) فتضيع قراءة "
                          f"الجهد. الحل: محوّل USB-Serial ثم "
                          f"RMS_GPS_PORT=/dev/ttyUSB0")
            return
        try:
            self._ser = serial.Serial(port, baud, timeout=1.0)
            self.ok = True
        except Exception as e:            # noqa: BLE001
            self.error = str(e)
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop:
            try:
                raw = self._ser.readline().decode("ascii", errors="replace").strip()
            except Exception:             # noqa: BLE001
                time.sleep(0.5)
                continue
            if not raw.startswith("$"):
                continue
            try:
                msg = pynmea2.parse(raw)
            except Exception:             # noqa: BLE001
                continue
            with self._lock:
                if isinstance(msg, pynmea2.types.talker.GGA):
                    self._sats = int(msg.num_sats) if msg.num_sats else 0
                    self._hdop = float(msg.horizontal_dil) if msg.horizontal_dil else 0.0
                    self._fix = msg.gps_qual not in (None, 0, "0")
                    if self._fix and msg.latitude and msg.longitude:
                        self._lat, self._lng = msg.latitude, msg.longitude
                        self._last_fix_ts = time.time()
                elif isinstance(msg, pynmea2.types.talker.RMC):
                    if msg.status == "A" and msg.latitude and msg.longitude:
                        self._fix = True
                        self._lat, self._lng = msg.latitude, msg.longitude
                        self._last_fix_ts = time.time()

    def state(self) -> dict:
        with self._lock:
            fresh = self._fix and (time.time() - self._last_fix_ts) < 5.0
            return {
                "ok": self.ok,
                "fix": fresh,
                "lat": round(self._lat, 6),
                "lng": round(self._lng, 6),
                "sats": self._sats,
                "hdop": round(self._hdop, 1),
            }

    def close(self) -> None:
        self._stop = True
        try:
            if self._ser:
                self._ser.close()
        except Exception:                 # noqa: BLE001
            pass
