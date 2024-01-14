# Python script to obtain temperature/pressure data from BMP388 sensor using SPI

Disclaimer: No further work is planned to be done on that repository. It was done purely for university classes.

SPI config is hardcoded: mode 3 and chip select pin 29(D5) on RPi GPIO.

Auto sync on saving files in Visual Studio Code is enabled by default. To disable it set `triggerTaskOnSave.on` inside `./.vscode/settings.json` to `false`.

## Requirements:
- Python 3.10
- Poetry

## Running locally
1. `poetry install`
2. `poetry run bmp388/main.py`

## Running remotely
Repository contains bash scripts to sync and run the repository on remote Raspberry Pi.

To be able to use bash commands it is required to copy .env.example to .env file and set in there environmental variables needed for remote connection.

### Available commands:
- `./sync`: synchronizes the repository with remote directory
- `./run`: runs the project on remote host
- `./poetry`: poetry wrapper

### First run:
1. `./sync`
2. `./poetry install`
3. `./run`
