# -*- coding: utf-8 -*-
"""
config.py — كل الثوابت والمنافذ والعتبات في مكان واحد (نظير config.h في legacy)
==============================================================================
القيم أدناه **مثبتة تجريبياً على العتاد في M0** — لا تُخمّن ولا تُغيّر بلا سبب
موثّق. أي انحراف عنها يكسر القراءة.
"""
import os as _os

# جذر المستودع (للوصول إلى captures/ و logs/ و missions/)
_REPO_ROOT = _os.path.abspath(_os.path.join(_os.path.dirname(__file__), ".."))

# ═══ المنصة ══════════════════════════════════════════════════════
# كل تعامل GPIO عبر lgpio حصراً (pigpio محذوف في Debian trixie).

# ═══ الجيجر (CAJOE J305) ═════════════════════════════════════════
GEIGER_GPIO           = 17        # BCM17 (دبوس 11)
# مثبت على العتاد: RISING_EDGE + SET_PULL_NONE (عائم) — مخرج CAJOE عالي الممانعة،
# أي مقاومة رفع/سحب داخلية تقتل الإشارة → CPM=0. (السطر في pi/sensors/geiger.py)
CPM_PER_USVH          = 111.0     # K — معاير ضد Cs-137 (662 keV)، 2026-07 — ثابت مخبري
DEADTIME_TAU_S        = 200e-6    # τ = 200µs — تصحيح الزمن الميت — ثابت مخبري
GEIGER_WINDOW_S       = 30        # نافذة CPM المنزلقة (ثوانٍ)
GEIGER_EMA_ALPHA      = 0.2       # معامل المتوسط الأسّي السريع (~5s)
HIGH_RATE_WARNING_CPM = 30000.0   # فوقها: تحذير دقة (مقاطعات لينكس تفقد نبضات)

# ═══ GPS (NEO-M8N) ═══════════════════════════════════════════════
GPS_PORT = "/dev/serial0"         # UART العتاد (GPIO15 RXD) — مثبت
GPS_BAUD = 9600

# ═══ BNO055 (IMU) ════════════════════════════════════════════════
BNO055_ADDR = 0x29                # مثبت على العتاد (وليس 0x28 الافتراضي)

# ═══ الكاميرا ════════════════════════════════════════════════════
CAMERA_INDEX = 0                  # /dev/video0 — مثبت
CAMERA_W = 640
CAMERA_H = 480
CAMERA_STREAM_FPS = 12            # إطارات البث الحي MJPEG
# ⚠ اللقطات لا تُكتب على القرص إلا بطلب صريح (زر «حفظ») أو لقطة شذوذ في M2.
CAPTURES_DIR = _os.path.join(_REPO_ROOT, "captures")

# ═══ تصنيف الخطر µSv/h (عتبات BRIEF — ثابتة) ═════════════════════
RISK_LOW_USVH  = 0.5              # Safe < 0.5 | Low 0.5-2
RISK_MED_USVH  = 2.0              # Medium 2-10
RISK_HIGH_USVH = 10.0            # High 10-100
RISK_CRIT_USVH = 100.0            # Critical > 100

# ═══ محاكاة الروفر (sim — الروبوت لم يصل) ════════════════════════
DRIVE_SPEED_MPS = 0.45            # سرعة تقريبية عند قوة 70 (من legacy)
TURN_RATE_DPS   = 70.0            # معدل الدوران بالمكان عند قوة 70 (درجة/ث)
ROVER_SAFETY_TIMEOUT_S = 1.5      # بلا أمر خلالها = توقف (نظير مهلة الأردوينو/الروفر)
SIM_HOME_LAT = 24.7136           # موقع افتراضي قبل أول قفل GPS
SIM_HOME_LNG = 46.6753

# ═══ الشبكة والبث ════════════════════════════════════════════════
WEB_HOST = "0.0.0.0"              # يُفتح محلياً وعبر Tailscale (http://therover:8000)
WEB_PORT = 8000
BROADCAST_S = 1.0                 # بث WebSocket كل ثانية

# ═══ الملاحة الذاتية الداخلية (BRIEF_AUTONOMY_INDOOR / BATCH1) ════
# دائرة عدم اليقين — نموذج نمو **تقديري** يُضبط تجريبياً لاحقاً (لا أرقام سحرية):
DRIFT_PER_METER         = 0.05    # م شك مضاف لكل متر مقطوع (تقديري)
DRIFT_PER_TURN          = 0.08    # م شك مضاف لكل 90° دوران (تقديري — الدوران أخطر)
UNCERTAINTY_INITIAL     = 0.10    # الشك الابتدائي بالمتر
UNCERTAINTY_RESET_FLOOR = 0.10    # أدنى شك بعد تصحيح جدار (يصغر ولا يصفر)
WALL_CORRECTION_SHRINK  = 0.30    # الشك × هذا عند تصحيح ناجح

# تصحيح الجدران — **شرطا أمان إلزاميان** (منع تفسير صندوق وسط الغرفة كجدار):
WALL_ALIGN_TOL_DEG          = 15.0  # تسامح المحاذاة حول محاور الجدران (0/90/180/270)
WALL_CORRECTION_MAX_DELTA_M = 0.5   # أقصى فرق مسافة يُقبل كتصحيح (أكبر = عائق لا جدار)

# المسح الشبكي:
CELL_DWELL_S      = 3.0           # توقّف عند كل خلية لتجميع عد كافٍ إحصائياً
NAV_ANOMALY_SIGMA = 3.0          # قراءة > μ+3σ = شذوذ (Welford)
NAV_ANOMALY_MIN_SAMPLES = 5      # أقل عدد قراءات قبل اعتماد كشف الشذوذ

# إعادة التخطيط حول العوائق:
MAX_REPLANS_PER_TARGET = 3       # بعده الهدف unreachable (منع الحلقات اللانهائية)

# السلامة التفاعلية (البند 3) — **معطّلة الآن** (تلمس عتاداً، تُختبر بدفعة منفصلة):
REACTIVE_SAFETY_ENABLED = False
ULTRASONIC_STOP_CM   = 25.0      # عائق أمامي أقرب = توقف فوري
REACTIVE_LOOP_HZ     = 10        # حلقة السلامة ≥10Hz
ESCAPE_MAX_ATTEMPTS  = 3         # محاولات التحرر قبل تعليم الخلية unreachable
# مهلة heartbeat = ROVER_SAFETY_TIMEOUT_S (1.5ث، معرّفة أعلاه)

# ملفات المعايرة (أرضيات مختلفة): profiles/calibration/*.json
CALIBRATION_DIR = _os.path.join(_REPO_ROOT, "profiles", "calibration")
