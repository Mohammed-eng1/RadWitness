#!/usr/bin/env python3
# مشي مستقيم مصحّح بالجايرو + مرشح السكون (Deadband & Filter)
import serial, time, json, statistics

ser = serial.Serial('/dev/serial0', 115200, timeout=1)
time.sleep(2); ser.reset_input_buffer()

# ---------- إعدادات القيادة ----------
INVERT      = -1
BASE_POWER  = 0.35      # السرعة الأساسية للأمام
GYRO_SCALE  = 0.9275    # المعامل المعاير
DURATION    = 5         # مدة المشي بالثواني

# ---------- وزنية المحركات (Trim) المستقرة ----------
LEFT_TRIM   =  0.028
RIGHT_TRIM  = -0.028

# ---------- ثوابت التصحيح (PID) ----------
KP = 0.035   # تم رفعها من 0.014 لتصحيح أقوى وأسرع
KD = 0.005   # إضافة نسبة طفيفة للمساعدة في استقرار العجلات
# ---------- إعدادات مرشح التنعيم وعتبة السكون (جديد) ----------
DEADBAND    = 1.5       # أي دوران أقل من 1.5°/ثانية يُعتبر ضجيجاً ويُهمل
ALPHA       = 0.3       # معامل التنعيم (Low-Pass Filter)
gz_filtered = 0.0

def read_gz():
    ser.reset_input_buffer()
    ser.write(b'{"T":126}\n')
    t = time.time()
    while time.time() - t < 0.05:
        if ser.in_waiting:
            l = ser.readline().decode(errors='ignore').strip()
            if '1002' in l:
                try: 
                    gz = json.loads(l)['gz']
                    if abs(gz) < 50.0:  # استبعاد القيم الشاذة جداً
                        return gz
                except: pass
    return None

# --- الدالة الجديدة لتنقية القراءات وتجاهل الضجيج ---
def get_corrected_gz(raw_gz, bias):
    global gz_filtered
    
    # 1. طرح الانحياز المرجعي
    val = raw_gz - bias
    
    # 2. تجاهل القفزات الشاذة جداً أثناء الحركة (Spikes > 12°/s)
    if abs(val) > 12.0:
        return gz_filtered if abs(gz_filtered) < 5.0 else 0.0
        
    # 3. تطبيق مرشح التنعيم (Low-Pass Filter) أولاً لامتصاص اهتزاز المحركات
    gz_filtered = (ALPHA * val) + ((1.0 - ALPHA) * gz_filtered)
    
    # 4. تطبيق عتبة السكون ثانياً على القيمة المنعمة (وليس الخام)
    if abs(gz_filtered) < DEADBAND:
        return 0.0
        
    return gz_filtered

def motors(l, r):
    l = max(-0.5, min(0.5, l)); r = max(-0.5, min(0.5, r))
    ser.write((json.dumps({"T":1,"L":l*INVERT,"R":r*INVERT}) + '\n').encode())

def stop(): motors(0, 0)

# ---- معايرة الانحياز المطورة ----
print("=== معايرة الجايرو (لا تلمس الروبوت 5 ثوانٍ) ===")
s, t = [], time.time()

while time.time() - t < 5:
    g = read_gz()
    if g is not None: 
        s.append(g)
    time.sleep(0.01)

if s:
    med = statistics.median(s)
    clean_s = [val for val in s if abs(val - med) < 3.0]
    bias = statistics.median(clean_s) if clean_s else med
    min_g, max_g = min(s), max(s)
    print(f"تم جمع {len(s)} عينة (المقبولة: {len(clean_s)}) | الانحياز = {bias:.3f}")
    print(f"نطاق التذبذب الخام: من {min_g:.3f} إلى {max_g:.3f}\n")
else:
    bias = 0
    print("❌ خطأ: لم يتم استلام أي قراءات صحيحة من الجايرو!\n")

# ---- المشي المستقيم ----
input(f"ضع الروبوت في ممر (يفضّل بين جدارين) واضغط Enter — سيمشي {DURATION}ث...")
print("يمشي مستقيماً...\n")

heading = 0.0
last_error = 0.0
gz_filtered = 0.0   # إعادة تصفير القيمة المصفاة قبل بدء الحركة
last = time.time()
t_start = time.time()

try:
    while time.time() - t_start < DURATION:
        g = read_gz()
        now = time.time()
        dt = now - last
        
        if g is not None and dt > 0:
            last = now
            
            # استدعاء الدالة الجديدة لحساب السرعة الزاوية النظيفة
            gz_clean = get_corrected_gz(g, bias)
            
            # حساب التكامل باستخدام القراءة النظيفة
            heading += gz_clean * dt * GYRO_SCALE

            error = heading
            derivative = (error - last_error) / dt
            correction = KP * error + KD * derivative
            
            correction = max(-0.25, min(0.25, correction))
            last_error = error

            left  = BASE_POWER + LEFT_TRIM - correction
            right = BASE_POWER + RIGHT_TRIM + correction
            motors(left, right)

            print(f"  الاتجاه: {heading:+6.2f}° | تصحيح: {correction:+.3f} | "
                  f"يسار:{left:.2f} يمين:{right:.2f}   ", end="\r", flush=True)

finally:
    stop()
    print(f"\n\nتوقف. الانحراف النهائي: {heading:+.2f}°")
    print("المثالي: قريب من 0° والروبوت مشى مستقيماً")