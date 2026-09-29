# Robot tools

Both scripts need only `pyserial` (and `matplotlib` for the optional plot in `test_c1.py`).

```bash
sudo apt install -y python3-serial      # Raspberry Pi
pip install pyserial matplotlib         # Windows / other
```

## `test_c1.py` — LiDAR check

Checks an RPLIDAR C1 over its USB adapter (CP210x, 460800 baud): device info, health, rotation speed, points per rotation, and distance in front.

```bash
python test_c1.py                      # auto-detects the port
python test_c1.py --port COM6 --plot   # live top view
python test_c1.py --warmup 120 --seconds 10 --front 0 --csv   # distance check after warm-up
```

Ends with `GO` or `NO-GO`.

## `lidar_teleop.py` — live LiDAR view + driving

Runs on the Raspberry Pi and serves one web page: a live top view of the LiDAR scan with drive buttons.

```bash
python3 lidar_teleop.py                # then open http://<pi-ip>:8080
python3 lidar_teleop.py --no-base      # view only, no driving
```

- Keys: W A S D or arrow keys, Space = stop.
- The robot moves only while a key or button is held. If the page stops sending (closed tab, Wi-Fi drop), the robot stops within about 0.3 s, and the base itself stops 0.5 s after the last command.
- Optional front block: forward motion is refused when an obstacle is closer than the set distance.
- Base link: UGV01 JSON over `/dev/ttyAMA4` at 115200 baud (change with `--base`).
