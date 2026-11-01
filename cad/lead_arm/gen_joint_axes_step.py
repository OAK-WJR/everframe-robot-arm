"""从 arm_description 的 URDF 生成"关节旋转轴圆"STEP 骨架。

每个转动关节输出一个标记:圆面(disk)+ 圆周边线,圆心 = 关节原点,
圆面法向 = 旋转轴。导入 Fusion 后对圆面边线做「项目/包括」即可得到草图圆。

用法:
    uv run gen_joint_axes_step.py            # 生成 1:1 与 1:4 两份
"""

import xml.etree.ElementTree as ET
from pathlib import Path

from build123d import Circle, Compound, Plane, export_step, import_step

URDF = Path(__file__).parent / "arm_description" / "urdf" / "arm.xacro"
OUT_DIR = Path(__file__).parent / "step"
RADIUS_MM = 20.0  # 全尺寸下圆的半径(仅作轴线标记,按需改)
SCALES = {"full": 1.0, "1to4": 0.25}

# 中文别名,方便在 CAD 树里认
ALIAS = {
    "Revolute 6": "J_yaw_base",
    "Revolute 2": "J_shoulder",
    "Revolute 3": "J_elbow",
    "Revolute 4": "J_wrist",
}


def parse_joints():
    root = ET.parse(URDF).getroot()
    joints = []
    for j in root.findall("joint"):
        o = j.find("origin")
        rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
        assert all(abs(v) < 1e-9 for v in rpy), f"{j.get('name')} 有非零 rpy,本脚本未处理旋转"
        ax = j.find("axis")
        joints.append(
            dict(
                name=j.get("name"),
                type=j.get("type"),
                parent=j.find("parent").get("link"),
                child=j.find("child").get("link"),
                xyz=[float(v) for v in o.get("xyz").split()],
                axis=[float(v) for v in ax.get("xyz").split()] if ax is not None else None,
            )
        )
    return joints


def world_positions(joints):
    """所有 rpy=0,世界坐标 = 沿链累加 origin。返回 {joint_name: (pos_m, axis)}"""
    link_pos = {"base_link": (0.0, 0.0, 0.0)}
    out = {}
    pending = list(joints)
    while pending:
        progressed = False
        for j in pending[:]:
            if j["parent"] in link_pos:
                p = link_pos[j["parent"]]
                pos = tuple(p[i] + j["xyz"][i] for i in range(3))
                link_pos[j["child"]] = pos
                if j["type"] in ("revolute", "continuous"):
                    out[j["name"]] = (pos, j["axis"])
                pending.remove(j)
                progressed = True
        assert progressed, f"运动链断裂,剩余: {[j['name'] for j in pending]}"
    return out


def build_markers(fk, scale):
    comps = []
    for name, (pos_m, axis) in fk.items():
        center = tuple(v * 1000.0 * scale for v in pos_m)  # m -> mm
        normal = tuple(abs(v) for v in axis)  # 圆面对 ±axis 等价,取正向
        disk = (Plane(origin=center, z_dir=normal) * Circle(RADIUS_MM * scale)).face()
        marker = Compound(children=[disk, disk.edges()[0]])
        marker.label = ALIAS.get(name, name.replace(" ", "_"))
        comps.append(marker)
    return Compound(children=comps)


def main():
    OUT_DIR.mkdir(exist_ok=True)
    fk = world_positions(parse_joints())
    print("转动关节世界坐标 (mm, 1:1):")
    for name, (pos, axis) in fk.items():
        mm = [round(v * 1000, 3) for v in pos]
        print(f"  {ALIAS.get(name, name):12s} center={mm}  axis={axis}")

    for tag, s in SCALES.items():
        path = OUT_DIR / f"arm_joint_axes_{tag}.step"
        export_step(build_markers(fk, s), str(path))
        # 回读验证
        shp = import_step(str(path))
        faces, edges = len(shp.faces()), len(shp.edges())
        centers = sorted(tuple(round(c, 3) for c in f.center()) for f in shp.faces())
        print(f"[{tag}] {path.name}: faces={faces} edges={edges}")
        for c in centers:
            print(f"    circle center @ {c}")
        assert faces == len(fk), f"{path.name} 面数不对"


if __name__ == "__main__":
    main()
