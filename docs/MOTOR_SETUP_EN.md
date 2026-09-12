# Motor Setup — Simple Guide (Freenove)

What this does: teaches the software **which wheel is which**, and
**which way is forward**. You only do this once per robot.

---

## Before you start

```bash
cd ~/RMS-Rover-v2
git pull
source venv/bin/activate
```

**Stop the server first.** It holds the I2C bus, and two programs on one
bus give wrong readings with no error message:

```bash
pkill -f "pi.web.server" ; sleep 1
```

🔴 **Lift the robot up.** Wheels must spin freely in the air.
The map is not set yet, so the robot may drive the wrong way.

---

## Step 1 — Is the board there?

```bash
i2cdetect -y 1
```

| You see | Meaning |
|---|---|
| **`40`** | ✅ Motor board found |
| `70` | Same chip answering on a second address — **normal**, not a problem |
| `48` | ✅ Battery ADC (reads the motor battery voltage) |
| `42` | Pi battery monitor (INA219) — not needed to drive motors |

🔴 **No `40` = no movement at all.** Check the Freenove board power
before doing anything else.

---

## Step 2 — Do the wheels move?

```bash
python3 -c "
from pi.rover.bridge import RoverControlBridge as B
import time
b = B(mode='real'); b.forward(0.3); time.sleep(2); b.stop(); b.close()"
```

All four wheels should spin for 2 seconds, then stop.

| What you see | What it means |
|---|---|
| All four spin | ✅ Power and wiring are fine |
| Nothing spins | Board power, or `40` missing — go back to Step 1 |
| Only some spin | A loose wire on that wheel |
| They spin and don't stop | Pull the power. Tell me. |

---

## Step 3 — Check each wheel alone 🔴

```bash
python3 -m pi.tests.probe_motor_map --wheels
```

The program pulses **one wheel at a time** and names it first:

```
▶ LEFT FRONT wheel  (channels 0/1) — press Enter…
  What happened?
    o = ONLY this wheel turned  (good)
    x = a DIFFERENT wheel turned
    m = MORE than one wheel turned
    n = NOTHING turned
```

**You want `o` four times.**

### Why this step exists

Step 4 pulses a **whole side** at once. If the two wheels on one side
fight each other, the side cancels itself out and the result looks
random — *"something moved but I can't tell what."*

That is exactly what happened to you before. One wheel was wired
backwards in the code. This step finds it in 30 seconds.

🔴 **If you don't get four `o`, stop.** Do not go to Step 4. Measuring on
top of a broken map measures the fault, not the direction.

---

## Step 4 — Which way is forward?

```bash
python3 -m pi.tests.probe_motor_map
```

Two pulses. After each one it asks:

```
Which side's wheels turned?
  g = the GEIGER TUBE side
  o = the OTHER side
  n = nothing turned

Which way did they roll?
  f = toward the FRONT (camera side)
  b = toward the BACK
```

### Why "geiger side" and not "left"

**Left and right flip depending on where you are standing.** If you look
at the robot from the front, its left is your right.

This exact confusion flipped a setting **twice in two days** on the old
robot. So we point at a real object instead — the geiger tube. That
never flips.

### The answer

```
✅ DONE. Put these two lines in pi/config.py:
   MOTOR_SWAP_LR = False
   MOTOR_INVERT  = 1
```

Copy them into `pi/config.py`. Done.

### If it says something is wrong

| Message | Meaning | Fix |
|---|---|---|
| One field did nothing | Dead side or unpowered channel | Check power + channel map |
| Both fields moved the same side | Channel map error | Fix `FREENOVE_WHEEL_CHANNELS` |
| Sides have opposite polarity | One motor wired backwards | **Fix the wiring** — no setting can fix this |

Telltale sign of the last one: *"go forward" makes the robot spin in
place instead of moving forward.*

---

## Step 5 — Confirm on the ground

Put the robot on the floor, in open space:

```bash
python3 -m pi.tests.check_directions --power 0.25
```

Forward goes forward. Back goes back. Turns go the right way.

⚠ **Stand BEHIND the robot** when judging left and right.

⚠ **Use 90 degree turns, not 180.** After a 180 turn the robot faces the
same way whether it turned left or right — so a 180 turn cannot tell
you if the direction is flipped.

### 🔴 If the turn goes the wrong way

**Do NOT change `MPU6050_GYRO_Z_SIGN`.**

That value was measured by turning the robot **by hand with the motors
off**, so it has nothing to do with motor wiring. Changing it looks like
it fixes the problem, but it just hides it — the two errors cancel each
other out and the safety check goes quiet. That bug lived for a month on
the old robot.

Run Step 4 again instead.

---

## Step 6 — Measure the speed

```bash
python3 -m pi.tests.calibrate_speed --powers 0.20 0.30 0.40 --seconds 3
```

The robot drives straight for 3 seconds at each power. **You measure the
distance with a tape measure** and type it in.

Why a tape measure: there are no wheel encoders on this robot. The
software guesses distance from speed × time. On the old robot one drive
went **0.20 m while the software thought 1.8 m** — 9× wrong, no warning.
A ruler on the floor is the only honest reference we have.

⚠ Use a **charged battery**. Speed drops when the battery is low.
⚠ Open space. This script does not avoid obstacles.

At the end it prints two lines for `pi/config.py`:

```
FREENOVE_SPEED_PER_POWER = 1.50
DRIVE_SPEED_MPS = 0.60
```

---

## Order (short version)

```
1. i2cdetect -y 1               → see 40
2. one-liner                    → wheels spin (LIFTED)
3. probe_motor_map --wheels     → four "o"   (LIFTED)
4. probe_motor_map              → two lines for config  (LIFTED)
5. check_directions             → on the ground
6. calibrate_speed              → tape measure
```

After any change to `pi/config.py`:

```bash
python3 -m pi.tests.check_config_hygiene
python3 -m pi.nav.selftest
```
