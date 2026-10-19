# Lab Machine Env
This repo contains the code for controlling the QArms. On lab computers, students are able to run this code, however it remains in the read only section of the drive so that it cannot be edited. Students can also clone this repo onto their personal laptops for controlling the virtual QArm as described in [Lab 0](https://github.com/UBC-ELEC-442-Labs-2026/Lab-0). The following will briefly cover what is in each folder and what each program does.

## Setting up the Virtual QArm

For students looking to run the programs on their own machine and test on a virtual QArm:

1. Follow the [Lab 0](https://github.com/UBC-ELEC-442-Labs-2026/Lab-0) installation process.
2. Run the virtual QArm trhough Quanser Interactive Labs.
3. Run any program that uses the QArm and select virtual QArm in the terminal when prompted.

## QArm-control
This folder contains programs made to allow the (safe) usage of the QArm. This also contains code that is used between multiple labs and stored here for convenience.

**`QArm_keyboard_control.py`**:  
 Implements control of the QArm using the keyboard via pygame. Generates a window that displays the live joint positions of the arm. Also note that the pygame window must be focused for the controls to register.

 **`QArm_functions.py`**:  
This is a key program for QArm usage in this lab. It acts as a middleman between the lab scripts and the Quanser QArm libraries. It passes through nessesary functions to the user such as writing joint positions, reading joint positions, kinematic solvers, etc. and allows for enforcing workspace limits.

- Once imported into your program, create a `QArm_Lab_Interface` object. This will initally give you access to the kinematic solvers. 

- If you create a QArm object then pass it in using `QArm_Lab_Interface.attach_QArm(myArm)`, you will be able to able to use the QArm control functions such as `write_to_arm(phi)`, `read_from_arm()`, etc. Commands and read joint positions are placed in buffers, while the actual QArm control reads/writes to these buffers and take place in a seperate thread running at 25Hz.

## Lab-1
This contains programs used in lab 1 only.

**`integration_err.py`**:  
Contains a script to run a demo that draws circles, showing integration error over time. Also generates a 3D graph.

**`QArm_traj_controllers.py`**:
To be paired with `QArm_traj.py` which is in a lab repo.

## Setup scripts
This repo contains two setup scripts. They are for different computers.

| Script | Use on | Run by |
|---|---|---|
| `setup_personal_computer.bat` | Personal laptops | Students | 
| `setup/windows/Initialize-QArmLabMachine.ps1` | Shared lab computers | Course staff (Admin) |

### `setup_personal_computer.bat`
Tells Windows where the copy of this repo is. The lab code (`constants.py` in each lab repo) reads the `QARM_LAB_ENV` environment variable to find the shared files here, such as `QArm-control/QArm_functions.py`. Doing it this way means no lab file contains a path specific to one computer and this repo can be cloned into any location.

The script checks that `QArm-control` and `Lab-1` sit next to the script, which confirms it is in the root of this repo. Then sets `QARM_LAB_ENV` to this folder for a Windows user account. It does not install Python packages, Quanser software or QLabs. Further, this script should not be confused with the setup script, `configure_python.bat` from Quanser. Those are separate steps in the Lab 0 setup guide.

To run it, open PowerShell in this folder and run:

```text
.\setup_personal_computer.bat
```

When it prints `QARM_LAB_ENV was configured successfully.`, close all VS Code windows and terminals and reopen them. If the repo is ever moved, run the script again from the new location.

Full instructions: [Lab 0 – Personal Computer Software Installation](https://github.com/UBC-ELEC-442-Labs-2026/Lab-0/blob/main/setup_instructions/personal-software-installation.md).

### `setup/windows/Initialize-QArmLabMachine.ps1`
**Staff only. Students do not need to run this.** It builds the shared Python environment that every student account on a lab computer uses, so the students don't each install their own copy of the course packages.

Before running, the machine needs:
- Python 3.13 installed for all users: `C:\Program Files\Python313\python.exe`
- QUARC / Quanser SDK installed. The script installs the Quanser Python wheels from the QUARC install folder.
- Quanser Academic Resources cloned to `C:\Quanser` (branch `dev-windows`)
- This repo cloned to `C:\QArmLab\lab-machine-env`

What it does:
1. Creates `C:\ProgramData\Qarm\Python\venv`, or reuses it if it already exists.
2. Installs the Quanser wheels and Quanser's `requirements.txt`. It swaps `opencv-python` for `opencv-contrib-python` (needed for `cv2.aruco`), adds `pyrealsense2`, and pins `numpy<2.4`.
3. Adds a `.pth` file so the venv can import `C:\Quanser\0_libraries\python` (`pal`, `hal`) and `QArm-control` without copying them.
4. Sets these system-wide (all users) environment variables:

   | Variable | Value |
   |---|---|
   | `QAL_DIR` | `C:\Quanser` |
   | `RTMODELS_DIR` | `C:\Quanser\0_libraries\resources\rt_models` |
   | `QARM_LAB_ENV` | `C:\QArmLab\lab-machine-env` |
   | `QARM_PYTHON` | `C:\ProgramData\Qarm\Python\venv\Scripts\python.exe` |

5. Checks the key imports, runs `pip check`, and writes a package list to `C:\ProgramData\Qarm\Python\installed-packages.txt`. It stops with an error if any step fails.

To run it, open PowerShell as admin and run:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass   # this window only
& "C:\QArmLab\lab-machine-env\setup\windows\Initialize-QArmLabMachine.ps1"
```

The default paths can be changed with the `-QuanserResources`, `-LabMachineEnvironment`, `-BasePython` and `-VenvDirectory` parameters. It is safe to run again after updates. Afterwards, sign out and back in, or open new terminals, so the new variables are picked up.

This script is only part of the lab computer setup. The shared VS Code extensions, taskbar layout, desktop shortcut, MATLAB/QUARC installs and licensing are set up separately; see the lab machine technical handoff document.

## .vscode

These settings allow VScode to autoamatically select the pre-configured shared Python interpreter on lab machines. They also recommend the Python and PDF extensions on any machines.