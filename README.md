# EC-MS Automation
Python code for automated CO2 reduction experiments using EC-MS.

## Instal Dependencies

Build venv in root directory:

```
python -m venv .venv
```

Upgrade pip:

```
.venv/bin/pip install --upgrade pip
```

Install dependencies into new venv:

```
.venv/bin/pip install -e .
```

Activate venv:

```
source .venv/bin/activate
```

Note: Replace *bin* with *Scripts* if using windows.

## Using Platformio to Flash Latest Firmware

Install the [PlatformIO VSCode Extension](https://docs.platformio.org/en/latest/integration/ide/vscode.html) and open a new Pio terminal (found in *Quick Access/Miscellaneous*). Change directory to either *gantry-kit* or *fluid-handling-kit* in the terminal, then connect the the target Arduino Nano via USB and run the following command:

```
cd gantry-kit/
```

```
pio run --target upload
```
## References
1. [UR5e](https://www.universal-robots.com/products/ur5e/)
2. [EC-MS](https://spectroinlets.com/ec-ms-technology/)
