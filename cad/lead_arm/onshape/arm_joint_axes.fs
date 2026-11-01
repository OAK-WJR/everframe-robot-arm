FeatureScript 1560;
import(path : "onshape/std/geometry.fs", version : "1560.0");

// ARM joint axes — 微缩 lead arm 关节骨架
// 来自 arm_description URDF 零位姿的 4 个转动关节。
// 每个关节生成一个草图圆:圆心 = 关节原点,草图面法向 = 旋转轴。
// 所有尺寸按 1:1 (mm) 填写,统一乘 ratio;默认 ratio = 0.25 (1:4)。
// 注:URDF 里 base 有 -0.14 / +6.36 mm 的建模噪声偏移,此处已归零,
//     yaw 轴即世界原点竖直线。有意义的侧向错位 (radial/d1/d2) 保留为参数。

annotation { "Feature Type Name" : "ARM joint axes" }
export const armJointAxes = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Scale ratio (1:4 = 0.25)" }
        isReal(definition.ratio, { (unitless) : [0.001, 0.25, 10] } as RealBoundSpec);

        annotation { "Name" : "Circle radius @1:1" }
        isLength(definition.radius, { (millimeter) : [0.1, 20, 1000] } as LengthBoundSpec);

        annotation { "Name" : "Yaw axis height (ground to yaw)" }
        isLength(definition.yawH, { (millimeter) : [0, 259.291, 5000] } as LengthBoundSpec);

        annotation { "Name" : "Rise (yaw to shoulder, vertical)" }
        isLength(definition.rise, { (millimeter) : [0, 72, 1000] } as LengthBoundSpec);

        annotation { "Name" : "Radial (yaw to shoulder, horizontal)" }
        isLength(definition.radial, { (millimeter) : [-1000, 15, 1000] } as LengthBoundSpec);

        annotation { "Name" : "L1 upper arm (shoulder to elbow)" }
        isLength(definition.L1, { (millimeter) : [0, 500, 5000] } as LengthBoundSpec);

        annotation { "Name" : "d1 upper arm lateral offset" }
        isLength(definition.d1, { (millimeter) : [-1000, 37, 1000] } as LengthBoundSpec);

        annotation { "Name" : "L2 forearm (elbow to wrist)" }
        isLength(definition.L2, { (millimeter) : [0, 304.475, 5000] } as LengthBoundSpec);

        annotation { "Name" : "d2 forearm lateral offset" }
        isLength(definition.d2, { (millimeter) : [-1000, -48.001, 1000] } as LengthBoundSpec);
    }
    {
        const r = definition.ratio;
        const zero = 0 * millimeter;
        const zShoulder = definition.yawH + definition.rise;   // 肩/肘/腕共面高度
        const joints = [
            { name : "yaw",
              center : vector(zero, zero, definition.yawH),
              axis : vector(0, 0, 1) },
            { name : "shoulder",
              center : vector(definition.radial, zero, zShoulder),
              axis : vector(1, 0, 0) },
            { name : "elbow",
              center : vector(definition.radial + definition.d1, -definition.L1, zShoulder),
              axis : vector(1, 0, 0) },
            { name : "wrist",
              center : vector(definition.radial + definition.d1 + definition.d2,
                              -(definition.L1 + definition.L2), zShoulder),
              axis : vector(1, 0, 0) }
        ];

        for (var j in joints)
        {
            var sk = newSketchOnPlane(context, id + j.name, {
                "sketchPlane" : plane(j.center * r, j.axis)
            });
            skCircle(sk, "circle", {
                "center" : vector(0, 0) * millimeter,
                "radius" : definition.radius * r
            });
            skSolve(sk);
        }
    });
