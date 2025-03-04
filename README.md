# Repository for Mechatronics Engineering 4th year capstone project

## Raspberry Pi Setup

### Update Pi OS
```bash
sudo apt update && sudo apt upgrade -y
```
### Install Seeed Voicecard

```bash
git clone https://github.com/HinTak/seeed-voicecard
cd seeed-voicecard
sudo ./install.sh
sudo reboot
```
### Install Poetry

```bash
	python -m pip install --upgrade pip
	curl -sSL https://install.python-poetry.org | python3 -
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
    source ~/.bashrc
	poetry config virtualenvs.in-project true
```
