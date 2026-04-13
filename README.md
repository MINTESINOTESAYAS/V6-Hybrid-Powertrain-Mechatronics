# V6 Hybrid Powertrain — Mechatronics Integration Project

![Status](https://img.shields.io/badge/Status-In%20Progress-yellow)
![Tools](https://img.shields.io/badge/Tools-SolidWorks%20%7C%20MATLAB%20%7C%20Simulink-blue)
![Level](https://img.shields.io/badge/Level-4th%20Year%20Mechatronics-green)

## Overview

A fully integrated mechatronics project combining mechanical 
design, CFD simulation, and modern control systems — built 
around a V6 engine hybrid powertrain concept.

This project demonstrates end-to-end engineering capability 
across three domains:

- Mechanical — Full V6 engine assembly in SolidWorks
- Simulation — CFD intake manifold flow analysis
- Control — LQR optimal speed controller with 
  Luenberger observer in MATLAB/Simulink

---

## Project Structure
``` text
V6_Hybrid_Mechatronics_Project/
│
├── 01_SolidWorks_Mechanical/
│   ├── Parts/
│   ├── Assembly/
│   ├── Drawings/
│   └── Renders/
│
├── 02_SolidWorks_Electrical/
│   ├── Wiring_Harness/
│   ├── PCB_Layout/
│   └── Schematics/
│
├── 03_Flow_Simulation/
│   ├── Intake_Manifold_CFD/
│   └── Results/
│
├── 04_MATLAB_Simulink/
│   ├── Motor_Model/
│   ├── Controller_Design/
│   ├── Hybrid_Powertrain/
│   └── Results/
│
├── 05_Integration/
│   └── SW_to_Simulink_Parameters/
│
└── 06_Documentation/
├── Project_Report.pdf
└── Presentation_Slides.pptx

---
```
## Tools Used

| Tool | Purpose |
|------|---------|
| SolidWorks 2024 | 3D mechanical modelling & assembly |
| SolidWorks Flow Simulation | CFD intake manifold analysis |
| SolidWorks Electrical | Wiring harness & PCB layout |
| MATLAB R2024a | State space modelling & LQR design |
| Simulink | Dynamic system simulation |
| Simscape | Physical component modelling |

---

## Completed Phases

- [ ] V6 engine 3D model — all parts and assembly
- [ ] CFD flow simulation — intake manifold
- [ ] DC motor SolidWorks model
- [ ] MATLAB state space motor model
- [ ] LQR controller + Luenberger observer
- [ ] Full HEV Simulink simulation
- [ ] SolidWorks Electrical integration
- [ ] Final report

---

## Key Technical Highlights

#### DC Motor State Space Model
```math
\dot{x} = Ax + Bu
```
#### States:
```text
armature current and angular velocity
```
#### Controller:
```text
LQR optimal with full-state
```
#### feedbackObserver:
```text
Luenberger for unmeasured state estimation
```
---
## HEV Power Split Modes

| Mode | Condition | ICE  | Motor |
|----|---------|---|-----|
| EV Only | Power <15KW, SOC>30% | Off | Active |
| Hybrid | 15KW<Power<80KW | Active | Active |
| ICE Only | Power>80KW | Active | Off |
| Regen Brake | Deceleration | Off | Generator |
| Charging | SOC<40% | Extra | Generator |

## Results

Results will be updated as each phase is completed.

## About

Author: Mintesinot Esayas

Degree: BSC. Mechatronics Engineering (4th)

Focus: Hybrid powertrains, Modern control system

Open to: Engineering roles & freelance CAD/Simulation work 
