# IronWatch — Smart Factory Security Access Gate

> A facial-recognition and **ferrous-metal theft-detection** security gate for
> industrial (metal-factory) access control, with a motorised boom barrier under
> PID control.

This folder is a **self-contained continuation** of the IronWatch project
(originally prototyped across Proteus, Arduino IDE, and Python/OpenCV in an
earlier session). It currently contains the **control subsystem** — the boom-arm
dynamics and PID controller — which was the next unstarted step when work paused.

> ⚠️ **Repository note.** This lives inside the `V6-Hybrid-Powertrain-Mechatronics`
> repo only because the working sandbox is scoped to that single GitHub repo and
> could not create a new one. IronWatch is a *separate* project. See
> [Moving this to its own repository](#moving-this-to-its-own-repository) to lift
> it out cleanly.

## System overview

```
                 ┌──────────────────────────────────────────┐
   person  ──►   │  IDENTIFICATION  (face recognition + RFID)│
                 │  → attendance log (entry / exit)          │
                 └───────────────┬──────────────────────────┘
                                 │ authorised?
                 ┌───────────────▼──────────────────────────┐
   exit only ──► │  FERROUS-METAL DETECTION (coil + CD4046)  │
                 │  → iron vs harmless metal discrimination  │
                 └───────────────┬──────────────────────────┘
                                 │ clear & authorised → OPEN
                 ┌───────────────▼──────────────────────────┐
                 │  MOTORISED BOOM BARRIER  (this module)    │
                 │  → PID position control, smooth open/close│
                 └──────────────────────────────────────────┘
```

## Contents

| Path | Status | Description |
|------|--------|-------------|
| `04_Control_PID/` | ✅ **done here** | Boom-arm sizing, dynamic model, PID design, plots, Arduino + MATLAB code |

### Planned / from earlier sessions (to be added)

These were built or drafted previously and should be dropped in as their own
folders when you consolidate the project:

| Path | Description |
|------|-------------|
| `01_Proteus_Electrical/` | Arduino + CD4046 phase-discrimination detection circuit + `metal_gate_detector.ino` |
| `02_Python_Identification/` | `entry_scanner.py`, `exit_scanner.py`, `known_faces/`, attendance CSV, Firebase stub |
| `03_SolidWorks_Mechanical/` | Gantry frame + boom arm + motor/pivot housing assembly and drawings |
| `05_PLC_Sequencing/` | Ladder/Stateflow access sequence with safety interlocks |
| `06_Documentation/` | Internship report (AASTU template) |

## Quick start (control module)

```bash
cd 04_Control_PID
python3 -m venv .venv && source .venv/bin/activate
pip install -r ../requirements.txt
python boom_arm_pid.py        # prints sizing + control report, writes figures/
```

Recommended PID gains from the model: **Kp = 70, Ki = 60, Kd = 4**.
Full derivation in `04_Control_PID/DESIGN_NOTES.md`.

## Moving this to its own repository

The sandbox could not create a new GitHub repo (its token only has access to the
one repo). To give IronWatch its own home, run this locally once you're on your
own machine:

```bash
# 1. copy this folder out of the V6 repo
cp -r IronWatch-Security-Gate ~/IronWatch-Security-Gate
cd ~/IronWatch-Security-Gate

# 2. make it its own git repo
git init
git add .
git commit -m "IronWatch: boom-arm dynamics + PID control subsystem"

# 3. create the GitHub repo and push (needs your own gh login)
gh repo create IronWatch-Security-Gate --private --source=. --push
#   or, if you created the empty repo on github.com first:
# git remote add origin https://github.com/<you>/IronWatch-Security-Gate.git
# git push -u origin main
```

## License

Inherits the parent repository's license unless relocated, at which point add
your own.
