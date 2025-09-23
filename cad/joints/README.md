# frc114-harmonic-drive

[![License: CERN-OHL-S-2.0](https://img.shields.io/badge/License-CERN--OHL--S--2.0-blue.svg)](./LICENSE)
[**🔎 Search (filename)**](https://github.com/OAK-WJR/frc114-harmonic-drive/find/main) · [**📘 Docs**](#old-docs-server-removed) · [中文 Chinese](./README.zh-CN.md)

> Built for FRC robotics: a modular library of **harmonic reducers**, motor-board **adapters**, and **universal joints** (vertical).  
> Repository accepts **`.step` only** (mm, Z↑). *(Legacy non-STEP files will be migrated.)*

<p align="center">
  <img src="docs/real-pictures/real-joint-D25_vertical_top.png" width="49%" alt="Real assembly - D25 vertical (top view)"/>
  <img src="docs/design-example/pictures/joint-D25_vertical_main.png" width="49%" alt="D25 vertical joint - main render"/>
</p>

---

## Why this repo

- **Single interface per model** (e.g., **D25**): every adapter ↔ every joint mates through the same **AJI (Adapter–Joint Interface)**.  
- **Swap-friendly**: change motors or joints **without** re-drawing the opposite side.  
- **Consistent exports**: only `.step` files to maximize CAD interoperability.

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

## Contributing / Requirements

1. Export **`.step`** only (mm, Z↑).  
2. Follow naming & folder layout above.  
3. Verify AJI fit with **all** opposite-side parts of the same model.  
4. (Optional) update `config/variants.csv`.  
5. Submit PR with 1–2 screenshots.

<p align="center">
  <img src="docs/design-example/pictures/D25-MotorBoardAdapter_back-design_reference.png" width="49%" alt="D25 adapter - back (blue: AJI frozen)"/>
  <img src="docs/design-example/pictures/D25-MotorBoardAdapter_front-design_reference.png" width="49%" alt="D25 adapter - front"/>
</p>
<p align="center"><i>Blue area = AJI mating & datum zone (FROZEN · do not modify).</i></p>

---

## License & attribution

© 2025 Jiarui Wu — Licensed under **CERN-OHL-S v2.0** (see `LICENSE`).  
Vendor “reference-only” models are **not** redistributed; use vendor links locally.
