# frc114-harmonic-drive

[![License: CERN-OHL-S-2.0](https://img.shields.io/badge/License-CERN--OHL--S--2.0-blue.svg)](./LICENSE)
[![🔎 Search this repo](https://img.shields.io/badge/%F0%9F%94%8E_Search-this_repo-brightgreen)](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=)
[![EN](https://img.shields.io/badge/lang-EN-black)](#english) [![中文](https://img.shields.io/badge/语言-中文-black)](#中文)

> EN: Click the green **Search this repo** button or any chips below to open GitHub search with filters.  
> ZH: 点击绿色 **Search this repo** 按钮或下方任意“芯片”，直接打开带过滤的 GitHub 仓库搜索。

---

## 🔎 Quick Search · 快速搜索

**Series / 型号**  
D: [D17](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2F+D17&type=code) · [D20](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2F+D20&type=code) · [D25](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2F+D25&type=code) · [D32](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2F+D32&type=code)  
J: [J32](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2F+J32&type=code)

**Motors / 电机**  
[NEOVERTEX](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fadapters+NEOVERTEX&type=code) · [NEO](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fadapters+NEO&type=code) · [KRAKENX60](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fadapters+KRAKENX60&type=code) · [FALCON500](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fadapters+FALCON500&type=code) · [775PRO](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fadapters+775PRO&type=code)

**Joints / 关节**  
[vertical 垂直](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fjoints%2Fvertical+vertical&type=code) · [parallel 并行](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fjoints%2Fparallel+parallel&type=code)

**Examples / 示例**  
D25 + NEOVERTEX: [🔗](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fadapters%2FD25+NEOVERTEX&type=code)  
D25 vertical joint: [🔗](https://github.com/OAK-WJR/frc114-harmonic-drive/search?q=path%3Amechanical%2Fjoints%2Fvertical+%22D25-vertical%22&type=code)

> Tip: on GitHub repo pages, press `/` to focus the search box.  
> 技巧：在 GitHub 仓库页面按 `/` 可直接聚焦搜索框。

---

<a id="english"></a>
<details open>
<summary><b>English</b> · click to collapse</summary>

### Overview

A modular CAD library for **harmonic reducers**, motor-board **adapters**, and **universal joints** (vertical/parallel) used in FRC robotics.

- **Modularity & Interchangeability**: For a given model (e.g., **D25**), every adapter must mate with every universal joint of the same model via one unified **Adapter–Joint Interface (AJI)**. Swapping motors or joints requires no geometry change on the other side.  
- **Format**: Only **`.step`** files are allowed across the repo.

<p align="center">
  <img src="docs/design_reference/D25-MotorBoardAdapter_front-example.png" alt="D25 Motor Board Adapter - front" width="45%"/>
  <img src="docs/design_reference/D25-MotorBoardAdapter_back-example.png" alt="D25 Motor Board Adapter - back" width="45%"/>
</p>

---

### Repository Layout

```
mechanical/
  base_reducers/                # reducers only (.step)
  adapters/
    D17/ D20/ D25/ J32/ ...
  joints/
    vertical/                   # <Series><Frame>-vertical.step
    parallel/                   # <Series><Frame>-parallel.step
docs/
  design_reference/             # AJI reference step + images
  usage.md                      # modeling & naming rules
config/                         # optional variants.csv
LICENSE
```

---

### Naming Rules

**Base reducers**
```
<Series><Frame>[-<Ratio>].step        e.g. D25-24.step, J32.step
```

**Adapters**
```
adapters/<Series><Frame>/<Series><Frame>-<MOTOR>-<vN>[-<Mount>].step
# MOTOR: NEOVERTEX | NEO | KRAKENX60 | FALCON500 | 775PRO (UPPERCASE)
# vN: v1, v2, v5...
# Mount (optional): P | NT | NF
```

**Universal joints**
```
joints/<type>/<Series><Frame>-<type>.step
# type: vertical | parallel
```

---

### Adapter–Joint Interface (AJI)

- **One AJI per model** (e.g., D25).  
- **Required fields**: `pilot_d`, `pilot_depth`, `bcd_d`, `holes_n`, `hole_d`, `face_thk`, `datum`; optional envelope: `OD/ID/length`.  
- **Reference**: see `docs/design_reference/D25-MotorBoardAdapter-example.step` (plus front/back images).

**Hard requirements**
1) Same-model **fit-all** (all adapters ↔ all joints).  
2) Joints connect **only** via AJI (no private holes).  
3) Swap motor/joint with **no re-draw** on the other side.

---

### Contributing

1. Model in Onshape → export **`.step`** (Z up, mm).  
2. Name files exactly per the rules above.  
3. Verify AJI compliance; test-fit with *all* opposite-side parts of the same model.  
4. (Optional) add an entry to `config/variants.csv`.  
5. PR with 1–2 screenshots.

---

### Third-party Reference Models

Vendor “reference-only” STEP files are **not** redistributed here.  
Use vendor links locally; we publish our own envelope models and unified AJI specs.

---

### License

© 2025 Jiarui Wu  
Licensed under **CERN-OHL-S v2.0**. See [LICENSE](./LICENSE).  
**SPDX**: `CERN-OHL-S-2.0`

</details>

---

<a id="中文"></a>
<details>
<summary><b>中文</b> · 点击展开</summary>

### 简介

面向 FRC 机器人的 **谐波减速器**、电机连接板 **适配件（adapters）** 与 **通用关节（vertical/parallel）** 的模块化 CAD 库。

- **模块化与互配**：对同一型号（如 **D25**）定义唯一 **AJI 接口**。该型号下**所有电机板适配器**必须与**所有通用关节**互配；更换电机或关节时，另一侧几何**无需重画**。  
- **格式**：全库只允许 **`.step`** 文件。

<p align="center">
  <img src="docs/design_reference/D25-MotorBoardAdapter_front-example.png" alt="D25 Motor Board Adapter - 正面" width="45%"/>
  <img src="docs/design_reference/D25-MotorBoardAdapter_back-example.png" alt="D25 Motor Board Adapter - 背面" width="45%"/>
</p>

---

### 仓库结构

```
mechanical/
  base_reducers/                # 仅减速器本体（.step）
  adapters/
    D17/ D20/ D25/ J32/ ...
  joints/
    vertical/                   # <Series><Frame>-vertical.step
    parallel/                   # <Series><Frame>-parallel.step
docs/
  design_reference/             # AJI 参考：示例 step 与配图
  usage.md                      # 建模与命名规则
config/                         # 可选 variants.csv
LICENSE
```

---

### 命名规则

**基础减速器**
```
<Series><Frame>[-<Ratio>].step        例：D25-24.step，J32.step
```

**适配器**
```
adapters/<Series><Frame>/<Series><Frame>-<MOTOR>-<vN>[-<Mount>].step
# MOTOR 全大写：NEOVERTEX | NEO | KRAKENX60 | FALCON500 | 775PRO
# vN: v1, v2, v5...
# Mount（可选）：P | NT | NF
```

**通用关节**
```
joints/<type>/<Series><Frame>-<type>.step
# type: vertical | parallel
```

---

### 适配器–关节统一接口（AJI）

- **每个型号仅一套 AJI**（如 D25）。  
- **最小字段**：`pilot_d`、`pilot_depth`、`bcd_d`、`holes_n`、`hole_d`、`face_thk`、`datum`；可选包络：`OD/ID/length`。  
- **参考**：见 `docs/design_reference/D25-MotorBoardAdapter-example.step`（含正/背面示意图）。

**刚性要求**  
1) 同型号 **全互配**（所有 adapters ↔ 所有关节）。  
2) 关节仅通过 AJI 对接（禁止私有孔位）。  
3) 更换电机/关节时另一侧**无需改图**。

---

### 贡献方式

1. Onshape 建模 → 导出 **`.step`**（坐标 Z↑，单位 mm）。  
2. 严格按命名规则放到正确目录。  
3. 验证 AJI 尺寸并与同型号对侧件完成定位测试。  
4. （可选）在 `config/variants.csv` 登记条目。  
5. 提交 PR 并附 1–2 张截图。

---

### 第三方参考模型

厂商标注“仅供参考（reference-only）”的 STEP **不随仓库发布**。  
请本地使用厂商链接；仓库仅发布自建等效包络与统一 AJI 规范。

---

### 许可

© 2025 Jiarui Wu  
使用许可：**CERN-OHL-S v2.0**。详见 [LICENSE](./LICENSE)。  
**SPDX**：`CERN-OHL-S-2.0`

</details>
