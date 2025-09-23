# frc114-harmonic-drive

[![License: CERN-OHL-S-2.0](https://img.shields.io/badge/License-CERN--OHL--S--2.0-blue.svg)](./LICENSE)

**EN:** A modular CAD library for harmonic reducers, motor-board **adapters**, and **universal joints** (vertical/parallel) used in FRC robotics.  
**ZH:** 面向 FRC 机器人的谐波减速器 CAD 库，包含电机连接板 **适配件（adapters）** 与 **通用关节（vertical/parallel）**，强调模块化与互配。

- **Modularity & Interchangeability** · **模块化与互配**  
  **EN:** For a given model (e.g., D25), **every adapter** must mate with **every universal joint** of the same model via one unified **Adapter–Joint Interface (AJI)**; swapping motors or joints requires no re-drawing on the other side.  
  **ZH:** 同一型号（如 D25）定义唯一 **AJI 接口**。该型号下**所有电机板适配器**与**所有通用关节**必须通过该 AJI 完全互配，更换电机或关节无需改动对侧几何。

---

## 1) Repository layout · 仓库结构

mechanical/
base_reducers/ # reducers only · 仅减速器本体（.step）
adapters/
D17/ D20/ D25/ J32/ ... # <Series><Frame>/...
joints/
vertical/ # <Series><Frame>-vertical.step
parallel/ # <Series><Frame>-parallel.step
docs/
design_reference/ # AJI 参考：示例 STEP 与前/后视图
usage.md # 详细建模&命名规则（团队内规范）
config/ # （可选）variants.csv 等参数表
LICENSE # CERN-OHL-S v2.0 原文
README.md

> **File format · 文件格式**：**Only `.step` is allowed**（全库只允许 `.step`）。  
> 不提交 `.x_t / .stl / .sldprt` 等。

---

## 2) Naming rules · 命名规则

> 统一使用大写电机名、连字符 `-` 分隔，不使用下划线与空格。  
> Use UPPERCASE motor names and hyphens (`-`); avoid underscores/spaces.

### 2.1 Base reducers · 基础减速器
**Pattern:**  
<Series><Frame>[-<Ratio>].step
- `<Series>`: `D | J`  
- `<Frame>`: `17 | 20 | 25 | 32`  
- `<Ratio>`: e.g., `8 | 14 | 24`（未知/不区分可省略）  
**Examples:** `D25-24.step`, `D17-8.step`, `J32.step`

### 2.2 Adapters · 适配器（减速器 + 电机连接板）
**Pattern:**  
adapters/<Series><Frame>/<Series><Frame>-<MOTOR>-<vN>[-<Mount>].step
- `<MOTOR>`: `NEOVERTEX | NEO | KRAKENX60 | FALCON500 | 775PRO ...`（全大写）  
- `<vN>`: version，如 `v1 / v2 / v5`  
- `<Mount>`: 可选板型/装配短码（例：`P / NT / NF`；在 docs/usage.md 解释）  
**Example:** `adapters/D25/D25-NEOVERTEX-v5-P.step`

### 2.3 Universal joints · 通用关节
**Pattern:**  
joints/<type>/<Series><Frame>-<type>.step
- `<type>`: `vertical | parallel`  
**Examples:** `joints/vertical/D25-vertical.step`, `joints/parallel/J32-parallel.step`

---

## 3) Adapter–Joint Interface (AJI) · 适配器–关节统一接口

**Goal 目标**  
- **EN:** One AJI per model (e.g., D25). Every adapter and joint for that model must implement the same AJI on the mating side.  
- **ZH:** 每个型号（如 D25）**仅有一套 AJI**。该型号的所有适配器与关节在对接侧必须实现相同 AJI。

**Fields 字段（最小集合）**  
- `pilot_d`, `pilot_depth`（定位止口直径/深度）  
- `bcd_d`, `holes_n`, `hole_d`（螺栓圆直径/孔数/孔径+公差）  
- `face_thk`（装配面厚度/沉孔/沉头规范）  
- `datum`（基准坐标轴与角度 0° 定义）  
- （可选 optional）包络尺寸：`OD/ID/length`

**Reference · 参考**  
- See `docs/design_reference/` for example STEP and front/back views, e.g.:  
  - `docs/design_reference/D25-MotorBoardAdapter-example.step`  
  - `docs/design_reference/D25-MotorBoardAdapter_front-example.png`  
  - `docs/design_reference/D25-MotorBoardAdapter_back-example.png`

**Hard requirements · 刚性要求**  
1. **Same-model fit-all**：同型号的 **所有 adapters ↔ 所有关节** 必须互配。  
2. **No private holes**：关节与适配器对接**仅通过 AJI**；禁止引入额外私有孔位破坏互换性。  
3. **No re-draw**：更换电机或关节，另一侧无需改动几何。

---

## 4) Find files · 如何查找

- **Browse · 浏览**：进入 `mechanical/adapters/<Series><Frame>/` 或 `mechanical/joints/<type>/`。  
- **Search · 搜索**（在 GitHub 仓库页按 `/` 打开搜索）：
  - By model: `D25` / `J32`  
  - By motor: `NEOVERTEX` / `KRAKENX60` / `NEO`  
  - By joint type: `vertical` / `parallel`  
- **Patterns · 模式**（复制到搜索框）：  
  - `path:mechanical/adapters D25 NEOVERTEX v5`  
  - `path:mechanical/joints vertical D25`

> 建议统一电机名大小写（例如一律 `NEOVERTEX`），保证搜索命中率。

---

## 5) Contributing · 贡献指南

1. **Modeling · 建模**：Onshape 建模，导出 `.step`；坐标统一：**Z↑**，毫米（mm）。  
2. **Naming · 命名**：严格按第 2 节命名，电机名全大写。  
3. **AJI compliance · 接口符合**：适配器关节侧必须匹配本型号 AJI；关节仅通过 AJI 对接。  
4. **Quality checklist · 自检清单**  
   - [ ] 文件为 `.step`；命名合规；目录正确  
   - [ ] 与 **所有同型号**对侧件（adapter 或 joint）完成装配定位测试  
   - [ ] BCD/止口/孔径公差符合 AJI 要求  
   - [ ] 典型电机外廓包络无干涉（NEOVERTEX/NEO/KRAKENX60 等）  
   - [ ] （可选）登记到 `config/variants.csv`  
5. **PR**：附 1–2 张截图（正/背或关节配合），简述更新点与适配对象。

---

## 6) Third-party reference models · 第三方参考模型

**EN:** Vendor “reference-only” STEP files are **not** redistributed in this repository. Use vendor links locally for reference; we publish our own envelope models and the unified AJI specs.  
**ZH:** 厂商标注“仅供参考（reference-only）”的 STEP **不随仓库发布**。请本地参考厂商链接；仓库仅发布我们自建的等效包络与统一 AJI 规范。  
（如需列来源，添加 `THIRD_PARTY_NOTICES.md` 并在此处链接。）

---

## 7) Roadmap · 路线图（可按需修改）

- [ ] 完成 D 系列（D17/D20/D25/D32）与 J 系列的 AJI 尺寸表  
- [ ] 适配主流电机板：`NEOVERTEX / NEO / KRAKENX60 / FALCON500 / 775PRO`  
- [ ] 补充 `joints/parallel/` 系列  
- [ ] `config/variants.csv` 自动生成 README 索引（GitHub Actions）

---

## 8) License · 许可

© 2025 Jiarui Wu  
Licensed under **CERN-OHL-S v2.0**. See [LICENSE](./LICENSE).  
**SPDX**: `CERN-OHL-S-2.0`

> **Summary**（GitHub 亦会显示）：强互惠硬件许可。修改或包含本作品的更大作品再分发时，**必须**在相同许可下提供**完整源**；保留声明；贡献者授予专利许可。  
> **摘要**：强互惠硬件许可；再分发修改/包含件需在同许可下公开完整源、保留声明，并包含专利授权。
