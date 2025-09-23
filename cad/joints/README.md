# frc114-harmonic-drive

[![License: CERN-OHL-S-2.0](https://img.shields.io/badge/License-CERN--OHL--S--2.0-blue.svg)](./LICENSE)
[![中文 / Chinese](https://img.shields.io/badge/lang-中文-black)](./README.zh-CN.md)
[![Search this repo](https://img.shields.io/badge/🔎_Search-Open_the_search_page-brightgreen)](https://OAK-WJR.github.io/frc114-harmonic-drive/search.html)

> Tip: Press `/` on any GitHub repo page to focus the built‑in search bar.  
> If you want a **real input box**, use our **Search Page** (green button above).

---

## Highlights
- **Modular** harmonic reducer library for FRC: **base reducers**, **motor-board adapters**, **universal joints (vertical/parallel)**.
- **One AJI per model** (e.g., D25): *every* adapter ↔ *every* joint must mate through the same **Adapter–Joint Interface**.
- **STEP‑only**: the repo accepts **`.step`** files (mm, Z↑).

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/design_reference/D25-MotorBoardAdapter_front-example.png">
    <img src="docs/design_reference/D25-MotorBoardAdapter_front-example.png" alt="D25 Motor Board Adapter - front" width="45%">
  </picture>
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/design_reference/D25-MotorBoardAdapter_back-example.png">
    <img src="docs/design_reference/D25-MotorBoardAdapter_back-example.png" alt="D25 Motor Board Adapter - back" width="45%">
  </picture>
</p>

---

## Quick search chips
- Series: [D17](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2F+D17&type=code) · [D20](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2F+D20&type=code) · [D25](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2F+D25&type=code) · [D32](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2F+D32&type=code) · [J32](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2F+J32&type=code)
- Motors: [NEOVERTEX](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fadapters+NEOVERTEX&type=code) · [NEO](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fadapters+NEO&type=code) · [KRAKENX60](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fadapters+KRAKENX60&type=code) · [FALCON500](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fadapters+FALCON500&type=code)
- Joints: [vertical](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fjoints%2Fvertical+vertical&type=code) · [parallel](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fjoints%2Fparallel+parallel&type=code)

> **Why not a search box inside README?** GitHub READMEs do **not** allow interactive forms/JS for security. Use the **Search Page** instead (GitHub Pages).

---

## Layout
```
mechanical/
  base_reducers/                # reducers only (.step)
  adapters/                     # <Series><Frame>/<Series><Frame>-<MOTOR>-<vN>[-<Mount>].step
  joints/                       # vertical/parallel/<Series><Frame>-<type>.step
docs/
  design_reference/             # AJI reference step + images
  usage.md                      # modeling & naming rules
config/                         # (optional) variants.csv
LICENSE
```

## Naming
**Base reducers**
```
<Series><Frame>[-<Ratio>].step     e.g. D25-24.step, J32.step
```
**Adapters**
```
adapters/<Series><Frame>/<Series><Frame>-<MOTOR>-<vN>[-<Mount>].step
MOTOR: NEOVERTEX|NEO|KRAKENX60|FALCON500 (UPPERCASE); vN: v1/v2/v5; Mount: P/NT/NF (optional)
```
**Universal joints**
```
joints/<type>/<Series><Frame>-<type>.step   # type: vertical|parallel
```

## Adapter–Joint Interface (AJI)
- Required fields: `pilot_d`, `pilot_depth`, `bcd_d`, `holes_n`, `hole_d`, `face_thk`, `datum`; optional envelope `OD/ID/length`.
- *Hard requirements:* same‑model **fit‑all**; mating **only via AJI**; swapping motor/joint **without** re‑drawing the other side.
- See `docs/design_reference/D25-MotorBoardAdapter-example.step` (+ front/back images).

## Contributing
1) Onshape → export `.step` (mm, Z↑).  
2) Follow naming & layout.  
3) Verify AJI fit with **all** opposite‑side parts of same model.  
4) (Optional) update `config/variants.csv`.  
5) PR with 1–2 screenshots.

## Third‑party models
Vendor “reference‑only” STEP files are **not** redistributed. Use vendor links locally; we publish our own envelope models & AJI specs.

## License
© 2025 Jiarui Wu · **CERN‑OHL‑S v2.0**. See [LICENSE](./LICENSE).  
SPDX: `CERN-OHL-S-2.0`
