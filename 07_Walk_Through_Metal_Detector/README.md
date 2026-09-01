# Walk-Through Metal Detector (WTMD) — SolidWorks Reference Project

![Status](https://img.shields.io/badge/Status-Complete-green)
![CAD Tool](https://img.shields.io/badge/CAD-SolidWorks%202024-blue)
![Domain](https://img.shields.io/badge/Domain-Security%20Mechatronics%20%7C%20Industrial%20Design-orange)

## Overview

This repository folder contains a complete reference project for designing and modeling a high-performance **Walk-Through Metal Detector (WTMD)** in SolidWorks. Developed following professional mechatronic design standards, this project covers full 3D mechanical assembly architecture, structural framing, electromagnetic coil housing, control console design, Bill of Materials (BOM), and step-by-step CAD modeling workflows.

Whether you are building a security portal for airports, courthouses, events, or correctional facilities, this reference project provides the exact blueprint, hierarchy, and parametric specifications required to execute the SolidWorks assembly.

---

## Folder Contents

```text
07_Walk_Through_Metal_Detector/
│
├── README.md                          # Project overview & architectural summary
├── Project_Specification.md           # Engineering specs, dimensions, coil design & standards
├── Bill_of_Materials.md               # Complete hierarchical BOM (mechanical, electrical, hardware)
├── SolidWorks_Modeling_Guide.md       # Step-by-step instructions for part creation & assembly mating
└── wtmd_model.scad                    # Parametric 3D OpenSCAD model script (viewable / exportable to STL/STEP)
```

---

## System Technical Specifications

| Parameter | Specification |
|-----------|---------------|
| **Passage Dimensions** | Height: 2000 mm × Width: 760 mm × Depth: 580 mm |
| **Overall Dimensions** | Height: 2220 mm × Width: 920 mm × Depth: 620 mm |
| **Structural Material** | Anodized Aluminum 6061-T6 Extrusions (40x40 mm) & High-Impact ABS Panels |
| **Detection Zones** | Multi-zone vertical layout (3 independent zones: Top, Middle, Bottom) |
| **Power Supply** | 100-240V AC, 50/60Hz with internal 12V DC LiFePO4 battery backup (4-hour autonomy) |
| **Enclosure Rating** | IP53 (Indoor / Weather-resistant sheltered outdoor) |
| **Total Assembly Weight** | ~48.5 kg |

---

## Key Design Highlights

1. **Modular Assembly Architecture**: Designed with top-level assembly (`WTMD_Main_Assembly.SLDASM`) linking two symmetrical vertical panel sub-assemblies and a rigid top bridge control console.
2. **Electromagnetic Integration**: Integrated coil routing channels inside the composite side panels ensuring precise alignment of pulsed induction transmitter and receiver coil windings.
3. **Ergonomic Control Console**: Angled LCD interface, multi-color LED alarm bars, and acoustic buzzer integrated directly into the upper crossmember.
4. **Stability & Compliance**: Low center of gravity with reinforced heavy-duty floor mounting base plates meeting seismic and tip-over safety margins.

---

## Author & Usage

Created as a professional reference project for mechatronic engineering CAD design. Feel free to use the provided specifications, modeling guide, and parametric code to generate your SolidWorks models.
