# Raspberry Pi ArduPilot PID Tuner

A two-file bench tool for manually tuning ArduCopter roll, pitch, and yaw control on a powered one-axis rig. ArduPilot remains in control; the Raspberry Pi records the physical push-and-release response, plots target versus actual attitude/rate, calculates metrics, and safely applies operator-selected gains.

> **Powered propellers are dangerous.** Secure the rig mechanically, keep people and loose objects clear, apply disturbances through the rig from outside the propeller envelope, retain a working RC transmitter and physical power cutoff, and test with propellers removed before running a powered test. Force-disarm stops motors immediately and is only for the secured rig.

## Hardware wiring

Use the Raspberry Pi UART at 3.3 V logic level. Cross TX and RX:

| Raspberry Pi | Physical pin | Flight controller telemetry port |
| --- | ---: | --- |
| Ground | 6 | Ground |
| GPIO14 / TXD | 8 | RX |
| GPIO15 / RXD | 10 | TX |

Do not power the Raspberry Pi from the flight controller telemetry connector. Give the Pi and propulsion system appropriately sized, common-ground power supplies.

On Raspberry Pi OS, run `sudo raspi-config`, disable the login shell over serial, enable the UART hardware, and reboot. The application defaults to `/dev/serial0`.

Configure the FC telemetry port used by the Pi for MAVLink 2 and 921600 baud. For example, TELEM2 commonly uses:

```text
SERIAL2_PROTOCOL = 2
SERIAL2_BAUD = 921
```

The correct `SERIALx` number depends on the flight controller port mapping.

## Install and run

Python 3.11 or newer is required.

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python web.py
```

At startup the server prints each detected private-network URL, and the same address is shown in the Vehicle control panel. Open that URL from a device on the same network, for example `http://192.168.1.42:8000`.

The server intentionally has no authentication and must only be used on an isolated, trusted network.

Options:

```bash
.venv/bin/python web.py \
  --device /dev/serial0 \
  --baud 921600 \
  --host 0.0.0.0 \
  --port 8000 \
  --data-dir runs
```

`web.py` contains the FastAPI service and complete inline browser UI. `tuner.py` contains all MAVLink, parameter, recording, persistence, and analysis logic.

## Test workflow

1. Connect the FC and wait until the page reports current gains.
2. Select the axis and set Stabilize mode.
3. Arm only after the powered rig and surrounding area are secure.
4. Press **Start recording**.
5. Use the rig's external mechanism to push and release the selected axis; never reach into the propeller envelope.
6. Press **Stop and analyze**.
7. Disarm, review the traces and metrics, enter new gains, and confirm the write.
8. Repeat and compare saved runs.

Gain writes are accepted only while disarmed. Each parameter is read back from ArduPilot; a partial failure triggers rollback. Recordings are stored as JSON under `runs/`, and the UI generates CSV exports on demand.

The normal Disarm action respects ArduPilot safety checks. Force-disarm requires typing `FORCE DISARM` and holding the separate control for three seconds. Force-arm is never available.

## Development checks

Run the dependency-free analysis smoke test and syntax checks with:

```bash
python3 tuner.py
python3 -m py_compile tuner.py web.py
```

Use ArduPilot SITL before connecting powered hardware to exercise telemetry, parameter rollback, Stabilize mode, and arm/disarm behavior.
