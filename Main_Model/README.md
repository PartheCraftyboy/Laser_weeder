# Jim — Mechanical CAD & Hardware Models

This directory contains the complete mechanical design, 3D CAD models, finite element analysis (FEA), and technical drawings for the **Jim 2-DOF Cartesian Laser Weeding Gantry**.

---

## 📐 Overview

The machine is a precision two-axis Cartesian gantry designed to operate over a **250 mm × 250 mm** agricultural target plot. The gantry moves an optical laser targeting head across the soil surface with high positioning accuracy (\(\pm 0.04\text{ mm}\) full-step resolution).

```
                      +-----------------------------+
                      |   NEMA 17 Stepper (X-Axis)  |
                      +--------------+--------------+
                                     | (Flexible Coupler)
                                     v
                       [T8 Lead Screw + 8mm Guide]
                                     |
                      +--------------v--------------+
                      |       X-Carriage Block      |
                      +--------------+--------------+
                                     | (NEMA 17 Y-Axis)
                                     v
                       [T8 Lead Screw + 8mm Guide]
                                     |
                      +--------------v--------------+
                      |   Y-Carriage / Laser Head   |
                      +-----------------------------+
```

---

## 📁 File Manifest

| File | Type | Description |
| :--- | :--- | :--- |
| `Main.SLDASM` | SolidWorks Assembly | Master CAD assembly of the complete 2-axis gantry |
| `jim_description.SLDASM` | SolidWorks Assembly | Assembly configured for URDF mesh export to ROS 2 |
| `D1_Main.SLDDRW` | SolidWorks Drawing | 2D technical engineering drawing and tolerances |
| `Nima 17 40x42x5mm.STEP` | STEP Standard | Neutral CAD interchange model of the NEMA 17 stepper |
| `Nima 17 40x42x5mm.SLDPRT` | SolidWorks Part | Native SolidWorks NEMA 17 42mm stepper motor model |
| `lead_screw.SLDPRT` | SolidWorks Part | T8 Lead Screw (2 mm pitch, 4 starts = 8 mm lead) |
| `Rod.SLDPRT` | SolidWorks Part | 8 mm precision hardened steel linear guide rail |
| `nut.SLDPRT` | SolidWorks Part | T8 brass trapezoidal lead screw nut |
| `Coupler.SLDPRT` | SolidWorks Part | 5 mm to 8 mm flexible jaw shaft coupler |
| `Lazer block.SLDPRT` / `.STL` | SolidWorks / STL | Laser diode module mount & aim collimator |
| `holding_bracket.SLDPRT` / `.STL` | SolidWorks / STL | End support bracket for guide rod and lead screw |
| `holding_bracket_x.SLDPRT` | SolidWorks Part | X-axis rail end anchor bracket |
| `holding_bracket_y.SLDPRT` / `.STL`| SolidWorks / STL | Y-axis guide rail carriage bracket |
| `mount_bracket.SLDPRT` / `.STL` | SolidWorks / STL | NEMA 17 motor face mount bracket |
| `Part6.SLDPRT` / `Part6.STL` | SolidWorks / STL | Structural cross-carriage mounting block |
| `Part2.SLDPRT`, `Part3.SLDPRT` | SolidWorks Parts | Auxiliary structural framing members |
| `Isometric_ss_asm.png` | Image | High-resolution isometric render of the full assembly |
| `Pic1.PNG` | Image | Close-up render of NEMA 17 actuator model |
| `X-axis.mp4` | Video | SolidWorks motion study animation showing X-axis translation |
| `WhatsApp Image 2026-09-03...` | Image | FEA static nodal stress analysis of `Part6` |
| `WhatsApp Image 2026-09-06...` | Image | Alternative perspective render of gantry assembly |

---

## 🔬 Finite Element Analysis (FEA)

Static stress analysis was conducted in **SolidWorks SimulationXpress** on the critical structural load-bearing member (`Part6` motor holding bracket) to evaluate rigidity and prevent deflection under drive torque and dynamic carriage acceleration:

* **Study Type**: Static nodal stress study (von Mises)
* **Maximum von Mises Stress**: \(1.942 \times 10^6\text{ N/m}^2\) (\(\approx 1.94\text{ MPa}\))
* **Material**: Structural Thermoplastic (PLA / PETG)
* **Yield Strength**: \(> 45\text{ MPa}\) (PETG / PLA)
* **Safety Factor**: \(> 20\times\), confirming that deflection is negligible under operational loads.

---

## 🖨️ 3D Printing Recommendations

For fabrication of custom structural brackets (`*.STL`):

* **Material**: PETG or ABS recommended (PLA acceptable if stepper motors are thermally derated to \(\le 1.0\text{ A}\)).
* **Layer Height**: 0.20 mm (0.16 mm for nut mounting surfaces).
* **Perimeters / Walls**: 4 to 5 perimeters for mechanical stiffness.
* **Infill**: 40%–50% Gyroid or Grid infill.
* **Dimensional Accuracy**: Ensure calibrated horizontal expansion for bearing press-fits and 8 mm smooth rod bores.
