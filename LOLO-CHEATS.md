# Raspberry Pi transfer cheats

Run these commands from the repository root.

## Create the ZIP

```bash
zip -r raspi-pid-tunner.zip . -x '.git/*' '.venv/*' 'runs/*' '__pycache__/*' '*.pyc' '*.zip'
```

## Send it to the Raspberry Pi desktop over SSH

Replace the username and IP address with the Raspberry Pi values:

```bash
PI_USER=your_pi_username
RASPI_IP=192.168.1.42
scp raspi-pid-tunner.zip "${PI_USER}@${RASPI_IP}:/home/${PI_USER}/Desktop/"
```

On the Pi, recreate the environment with:
```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```