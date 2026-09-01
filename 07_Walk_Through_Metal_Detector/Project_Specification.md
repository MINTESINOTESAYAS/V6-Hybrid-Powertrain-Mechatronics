# Engineering & Design Specifications: Walk-Through Metal Detector (WTMD)

## 1. Functional Requirements & Performance Objectives
- **Target Detection**: Ferrous and non-ferrous metallic contraband (guns, knives, metallic components) with uniform detection across all zones.
- **Throughput**: Capable of screening 40 to 60 people per minute without false alarms from minor personal items (keys, belt buckles, coins) when properly calibrated.
- **Environmental Immunity**: Immune to external electromagnetic interference (EMI) from nearby power lines, elevators, and fluorescent lighting.

---

## 2. Dimensional & Geometrical Specifications

### 2.1 Overall Envelope
- **Height**: 2220 mm
- **Width**: 920 mm (outer edge to outer edge)
- **Depth**: 620 mm (base footprint)

### 2.2 Walk-Through Passage
- **Clear Height**: 2000 mm
- **Clear Width**: 760 mm
- **Clear Depth**: 580 mm

---

## 3. Structural Sub-Systems & SolidWorks Part Breakdown

### 3.1 Base Assemblies
- **Base Plates (`01_Base_Plate_Left.SLDPRT` / `02_Base_Plate_Right.SLDPRT`)**: 
  - Material: Cast Aluminum / Powder-coated Structural Steel (Thickness: 15 mm).
  - Dimensions: 620 mm × 250 mm × 15 mm.
  - Features: M10 threaded holes for leveling feet and floor anchoring bolt patterns.

### 3.2 Vertical Frame Extrusions
- **Vertical Extrusions (`03_Extrusion_Vertical.SLDPRT`)**:
  - Profile: 40×40 mm T-Slot Anodized Aluminum Extrusion (ISO standard).
  - Quantity: 4 primary vertical pillars (2 per side panel).

### 3.3 Top Bridge / Control Console
- **Top Crossbar Housing (`07_Top_Console_Enclosure.SLDPRT`)**:
  - Manufacturing: Sheet Metal design (1.5 mm thickness, 5052 Aluminum) with injection-molded ABS end caps.
  - Dimensions: 920 mm (L) × 200 mm (W) × 120 mm (H).
  - Internal Components: Main DSP board, power supply unit, battery backup, LCD screen, speaker buzzer.

### 3.4 Side Panel Covers & Coil Formers
- **Side Panel Shells (`05_Panel_Cover_Left.SLDPRT` / `06_Panel_Cover_Right.SLDPRT`)**:
  - Material: High-impact ABS plastic (Vacuum formed or injection molded).
  - Internal Coil Formers (`08_Coil_Former_Panel.SLDPRT`): Non-conductive ABS/PVC plates featuring machined helical grooves for copper wire winding.

---

## 4. Electromagnetic Coil Architecture
- **Technology**: Pulsed Induction (PI) or Multi-Frequency Continuous Wave (MFCW).
- **Zone Configuration**: 3-zone vertical segmentation (Top, Center, Bottom) utilizing independent Tx/Rx coil pairs.
- **Wire Specification**: Litz wire (multi-strand insulated copper wire) to minimize skin effect losses and maximize magnetic flux sensitivity.

---

## 5. Mechatronic Sensors & I/O
- **Transit Sensors**: Dual-beam infrared (IR) optical transceivers mounted at 900 mm height on both door jambs to detect entry/exit direction and count foot traffic.
- **Alarm Indicators**: 
  - High-intensity LED light bars integrated vertically along both door posts for zone-specific target localization.
  - Audible multi-tone piezo buzzer (adjustable volume 0–90 dB at 1 meter).
- **User Interface**: 4-digit numeric LED display, 2×16 character LCD screen, and membrane keypad for sensitivity adjustment and calibration.
