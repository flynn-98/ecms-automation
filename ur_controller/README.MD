# Personal UR Controller with Python

## Setting up a Virtual Environment

To isolate the project's dependencies, it's recommended to use a virtual environment.  
If you haven't already installed Python's `venv` module, you can do so by running:

```bash
sudo apt install python3.12-venv   # For Linux (Debian/Ubuntu)
```

Create the virtual environment:

```bash
python3.12 -m venv venv            # Linux (Ubuntu) and MacOS
```

Activate the virtual environment:

```bash
source venv/bin/activate           # Activate the virtual environment (Linux/MacOS)
```

Install Poetry:

```bash
pip install poetry
```

Install project dependencies:

```bash
poetry install
```

---

## VSCode Compatibility

In case you are using VSCode:  
Native type hinting is not a default for the `ur_rtde` bindings (at the time of writing).  
To add type hinting, `pybind11-stubgen` is installed with the `pyproject.toml` when you install the dependencies through Poetry.

For `ur_rtde`, run:

```bash
pybind11-stubgen rtde_control -o ./typings/
pybind11-stubgen rtde_receive -o ./typings/
pybind11-stubgen rtde_io -o ./typings/
```

---

## EC-MS Automation for CAPeX
```
python3 AutomateCapex.py (glass object insertion for DTU setup)

python3  AutomateCapex_extrinsic.py (test setup insertion for SDU setup with "blind" extrinsic)
```