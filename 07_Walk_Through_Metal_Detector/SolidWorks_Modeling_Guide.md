# SolidWorks Modeling & Assembly Guide: Walk-Through Metal Detector

This guide outlines the professional workflow for building the Walk-Through Metal Detector assembly from scratch in **SolidWorks 2024**.

---

## Phase 1: Part Modeling Workflow

### 1. Base Plate (`Base_Plate.SLDPRT`)
- **Top Plane Sketch**: Draw a rectangle 620 mm × 250 mm with rounded corners (R20).
- **Extruded Boss/Base**: Extrude upward by 15 mm.
- **Hole Wizard / Cuts**: Add four counterbore holes (Ø11 mm thru, Ø18 mm cbore) for floor mounting anchors, and four M8 tapped holes on the top face for attaching vertical aluminum extrusions.

### 2. Vertical & Crossbar Extrusions (`Extrusion_Vertical.SLDPRT`)
- Use SolidWorks Weldments or sketch the standard **40×40 mm T-Slot profile** with 4 central fastener channels (Ø8.5 mm).
- Extrude to length: 2150 mm for vertical pillars, 840 mm for crossbars.

### 3. Top Control Console Enclosure (`Top_Console_Enclosure.SLDPRT`)
- Use **Sheet Metal** features:
  - Base Flange: Sketch the U-profile cross-section (Width 200 mm, Height 120 mm).
  - Edge Flange: Extrude lengthwise to 920 mm.
  - Cut-Extrudes: Add rectangular cutouts on the front face for the LCD screen, LED indicator bars, and keypad membrane.

### 4. Side Panel Shells (`Panel_Cover_Left.SLDPRT`)
- Create a surface loft or extruded shell representing the aerodynamic/aesthetic casing (Height 2050 mm, Width 500 mm, Depth 100 mm).
- Shell feature: Wall thickness 3 mm.

---

## Phase 2: Assembly Architecture (`WTMD_Main_Assembly.SLDASM`)

1. **New Assembly**: Create a blank assembly file `WTMD_Main_Assembly.SLDASM`.
2. **Insert Base Components**:
   - Insert `Base_Plate.SLDPRT` (Left) and fix the component to origin.
   - Insert `Base_Plate.SLDPRT` (Right) and mate it with a 760 mm distance constraint along the X-axis.
3. **Assemble Frame**:
   - Mate the four `Extrusion_Vertical.SLDPRT` instances to the base plates using **Coincident** and **Concentric** mates.
   - Attach top and bottom horizontal crossbars.
4. **Mount Control Console**:
   - Place `Top_Console_Enclosure.SLDPRT` on top of the vertical frame structure.
   - Apply **Coincident** mates between bottom console faces and top extrusion faces.
5. **Attach Panels & Electronics**:
   - Insert left and right panel covers and coil formers.
   - Use **Symmetry** and **Distance** mates to align them accurately within the metal frame.

---

## Phase 3: Drawing & Documentation
- Create 2D Engineering Drawings (`.SLDDRW`) for:
  - Overall Assembly General Arrangement (GA) with BOM table and balloon callouts.
  - Detail drawings for the Base Plate and Control Console sheet metal flat pattern.
