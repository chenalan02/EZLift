# Repository for Mechatronics Engineering 4th year capstone project

## Raspberry Pi Setup

#### Update Pi OS
```bash
sudo apt update && sudo apt upgrade -y
sudo apt-get install portaudio19-dev libatlas-base-dev
```

#### Install Audio Drivers
```bash
sudo apt update
sudo apt install portaudio19-dev
```

#### Install Seeed Voicecard

```bash
git clone https://github.com/HinTak/seeed-voicecard
cd seeed-voicecard
sudo ./install.sh
sudo reboot
```
#### Install Poetry

```bash
python -m pip install --upgrade pip
curl -sSL https://install.python-poetry.org | python3 -
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
poetry config virtualenvs.in-project true
```

#### Setup Environment
```bash
python -m venv --system-site-packages env
source env/bin/activate
env/bin/poetry install
```

#### Install Edge TPU runtime
```bash
https://docs.ultralytics.com/guides/coral-edge-tpu-on-raspberry-pi/#installation-walkthrough
```
