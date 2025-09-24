# frc114-harmonic-drive

[![License: CERN-OHL-S-2.0](https://img.shields.io/badge/License-CERN--OHL--S--2.0-blue.svg)](./LICENSE)
[**🔎 Search (filename)**](https://github.com/OAK-WJR/frc114-harmonic-drive/find/main) · [**📘 Docs**](#old-docs-server-removed)

> Built for FRC robotics: a modular library of **harmonic reducers**, motor-side **adapters**, and **universal joints** (vertical).  
> Repository accepts **`.step` only** (mm, Z↑). *(Legacy non-STEP files will be migrated.)*

<p align="center">
  <img src="docs/real-pictures/real-joint-D25_vertical_top.png" width="49%" alt="Real assembly - D25 vertical (top view)"/>
  <img src="docs/design-example/pictures/joint-D25_vertical_main.png" width="49%" alt="D25 vertical joint - main render"/>
</p>

---

## Definitions

- **Adapter** — the **motor-side connection module**.  
  Provides the motor interface (mounting pattern, pilot, flange) so a specific motor (NEO, Kraken, Falcon, etc.) can mount to the reducer. An adapter may be a flange + connector set, or an **integrated module** that also includes the motor.

- **Joint** — the **output-side connection module**.  
  Links the reducer to the rest of the mechanism, offered in **vertical** (straight-through) or **parallel** (offset) orientations.

- **AJI (Adapter–Joint Interface)** — the **fixed interface standard** between adapters and joints.  
  Geometry is **frozen** (pilot, bolt circle, hole size/count, face thickness, datum, etc.), so **every adapter in a series (e.g., D25) mates with every joint of the same series**. The frozen zone is shown in **blue** in the reference drawings.

---

## Why this repo

- **Benefits of harmonic drives**
  - **Zero/near-zero backlash** *(typical accuracy ≤ 1 arc-min ≈ 0.0167° ≈ 0.000291 rad)*  
  - **Lightweight** *(all D-series < 1 kg; D17 as low as ~0.41 kg)*  
  - **High ratios** *(e.g., 100:1)*  
  - **Compact** *(Ø ~80–142 mm; depth ~18.5–27.9 mm)*  
  - **High torque capacity** *(typical ~90 N·m ≈ 9 kg @ 1 m; some sizes up to ~892 N·m ≈ 90.9 kg @ 1 m)*

- **One interface per model (AJI)**  
  For each reducer size (e.g., **D25**), **all adapters** (motor-side modules) and **all joints** (output-side modules) share the **same AJI**.  
  → You can swap a motor (**change adapter**) or swap the joint orientation (**change joint**) **without redesigning the other side**.

- **Swap-friendly & consistent exports**  
  Hot-swap motors/joints within a model series; export is standardized to **`.step`** for maximum CAD interoperability.

---

## Quick access

- **Browse by directory**
  - Reducers: [`mechanical/base_reducers/`](https://github.com/OAK-WJR/frc114-harmonic-drive/tree/main/mechanical/base_reducers)
  - Adapters: [`D17`](https://github.com/OAK-WJR/frc114-harmonic-drive/tree/main/mechanical/adapters/D17) · [`D20`](https://github.com/OAK-WJR/frc114-harmonic-drive/tree/main/mechanical/adapters/D20) · [`D25`](https://github.com/OAK-WJR/frc114-harmonic-drive/tree/main/mechanical/adapters/D25) · [`J32`](https://github.com/OAK-WJR/frc114-harmonic-drive/tree/main/mechanical/adapters/J32)
  - Joints: [`mechanical/joints/`](https://github.com/OAK-WJR/frc114-harmonic-drive/tree/main/mechanical/joints)
- **Filename search** (works for binaries):  
  → https://github.com/OAK-WJR/frc114-harmonic-drive/find/main  
  Try: `D25-vertical` · `NEOVertex` · `KrakenX60`

---

## What’s inside

```text
mechanical/
  base_reducers/                # reducers only (.step)
  adapters/                     # <Series>/<...MOTOR...>.step
  joints/                       # <Series>-vertical.step
docs/
  design-example/
    D25-MotorBoardAdapter_example-design_reference.step
    pictures/                   # images used in README
  real-pictures/                # real assembly photos
LICENSE
```

**Supported models (now)**: `D17 · D20 · D25 · J32`  
**Example**: `mechanical/joints/D25-vertical.step`

---

## Naming (short)

- **Reducers**: `<Series><Frame>[-<Ratio>].step` → `D25-24.step`, `J32.step`  
- **Adapters**: `adapters/<Series>/<Series>-<MOTOR>-<vN>[-<Mount>].step`  
  - `MOTOR` UPPERCASE recommended: `NEOVERTEX | KRAKENX60 | ...`  
  - `vN`: `v1/v2/v5...`; `Mount` (optional): `P | NT | NF`  
- **Joints**: `joints/<Series>-vertical.step`

Full rules → **Docs**: #old-docs-server-removed

---

## Universal Joint Design Requirements
> **Note**: interchangeability is **within the same series only** (e.g., D25 ↔ D25).  
> Different series and frames (e.g. D17 vs. D20, D25 vs. J32) do **not** mate directly.
**Goal**: joints must be **modular and hot-swappable** so you can change motors (adapter side) or joint orientation **without** redesign.
- **AJI-compatible (frozen)**  
  Use the series’ **AJI** exactly as defined (pilot, bolt-circle, holes, face thickness, datum). **Do not modify the blue “frozen” zone** in the reference drawings.
- **Fits all adapters of the same series**  
  A joint for **D25** must assemble with **every D25 adapter** (NEO/NEOVERTEX/KRAKEN… all versions).
- **Must fit the reference adapter first**  
  Validate against the reference adapter file before anything else:  
  `docs/design-example/D25-MotorBoardAdapter_example-design_reference.step`
- **Orientation variants**  
  Joints come in **vertical** (straight-through) and **parallel** (offset). The AJI side is identical; only the output geometry changes.
- **Keep it modular**  
  No custom features on the AJI face. Reinforcements, cable channels, guards, etc. are allowed **outside** the frozen zone.
- **Clearances & fasteners**  
  Leave tool access for screws; respect min edge distance; avoid interference with typical motor housings and adapter bosses.
- **Envelope (recommended)**  
  Stay within a reasonable OD/height so different joints won’t collide with typical frames. Document any special clearance.
- **Export**  
  `.step` only (mm, Z↑).

**Acceptance checklist**
1) Mates with **reference adapter** (no interference).  
2) Mates with **all adapters** in the same series.  
3) Tool clearance OK; screws seat fully.  
4) No geometry in the **frozen** AJI zone.  
5) File name & path follow the **Naming** rules.

---

## Contributing / Requirements

1. Export **`.step`** only (mm, Z↑).  
2. Follow **Naming** & folder layout.  
3. **AJI compliance**: do **not** change the blue frozen AJI zone.  
4. **Universal joint-specific**:  
   - Must fit the **reference adapter** of the same series.  
   - Must fit **all adapters** of the same series (hot-swap guarantee).  
   - Keep design **modular**; no custom features on the AJI face.  
5. (Optional) update **`config/variants.csv`** (see below).  
6. Submit PR with 1–2 screenshots (one assembly with an adapter).

<p align="center">
  <img src="docs/design-example/pictures/D25-MotorBoardAdapter_back-design_reference.png" width="49%" alt="D25 adapter - back (blue: AJI frozen)"/>
  <img src="docs/design-example/pictures/D25-MotorBoardAdapter_front-design_reference.png" width="49%" alt="D25 adapter - front"/>
</p>
<p align="center"><i>Blue area = AJI mating & datum zone (FROZEN · do not modify).</i></p>

---

## License & attribution

© 2025 Jiarui Wu — Licensed under **CERN-OHL-S v2.0** (see `LICENSE`).  
Vendor “reference-only” models are **not** redistributed; use vendor links locally.




