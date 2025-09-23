# frc114-harmonic-drive

[![License: CERN-OHL-S-2.0](https://img.shields.io/badge/License-CERN--OHL--S--2.0-blue.svg)](./LICENSE)
[**🔎 Search models**](#old-docs-server-removed) · [**📘 Docs**](#old-docs-server-removed) · [中文 Chinese](./README.zh-CN.md)

> Built for FRC robotics: a modular library of **harmonic drive**, with frc motor **adapters**, and **universal joints**.  
> Repository accepts **`.step` only** (mm, Z↑).

<!-- TOP: Real photo (left) + joint main render (right) -->
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

## Quick search (current contents)

- Series: [D17](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2F+D17&type=code) · [D20](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2F+D20&type=code) · [D25](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2F+D25&type=code) · [J32](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2F+J32&type=code)  
- Motors: [NEOVERTEX](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fadapters+NEOVERTEX&type=code) · [KRAKENX60](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fadapters+KRAKENX60&type=code)  
- Joints: [vertical (all)](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fjoints+vertical&type=code) · [D25-vertical only](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fjoints+%22D25-vertical%22&type=code)

> Need a **real input box** with filters? Use the **Search page** → #old-docs-server-removed

---

## What’s inside

mechanical/
base_reducers/ # reducers only (.step)
adapters/ # <Series><Frame>/<Series><Frame>-<MOTOR>-<vN>[-<Mount>].step
joints/ # <Series><Frame>-vertical.step
docs/
design-example/
D25-MotorBoardAdapter_example-design_reference.step
pictures/ # images used in this README
real-pictures/ # real assembly shots
LICENSE

**Supported models (now)**: `D17 · D20 · D25 · J32`  
**Example**: `mechanical/joints/D25-vertical.step`

---

## Naming (short)

- **Reducers**: `<Series><Frame>[-<Ratio>].step` → `D25-24.step`, `J32.step`  
- **Adapters**: `adapters/<Series><Frame>/<Series><Frame>-<MOTOR>-<vN>[-<Mount>].step`  
  - `MOTOR` in UPPERCASE recommended: `NEOVERTEX | KRAKENX60 | ...`  
  - `vN`: `v1/v2/v5...`; `Mount` (optional): `P | NT | NF`  
  - *Note*: legacy files may contain mixed-case names; they’ll be normalized over time.  
- **Joints**: `joints/<Series><Frame>-vertical.step`

Full rules → **Docs**: #old-docs-server-removed

---

## Contributing / Requirements

1. Export **`.step`** only (mm, Z↑).  
2. Follow naming & folder layout above.  
3. Verify AJI fit with **all** opposite-side parts of the same model.  
4. (Optional) update `config/variants.csv`.  
5. Submit PR with 1–2 screenshots.

<!-- BOTTOM: design references side-by-side -->
<p align="center">
  <img src="docs/design-example/pictures/D25-MotorBoardAdapter_back-design_reference.png" width="49%" alt="D25 adapter - back (blue: AJI frozen)"/>
  <img src="docs/design-example/pictures/D25-MotorBoardAdapter_front-design_reference.png" width="49%" alt="D25 adapter - front view"/>
</p>
<p align="center"><i>Blue area = AJI mating & datum zone (FROZEN · do not modify).</i></p>

---

## License & attribution

© 2025 Jiarui Wu — Licensed under **CERN-OHL-S v2.0** (see `LICENSE`).  
Vendor “reference-only” models are **not** redistributed; use vendor links locally.

