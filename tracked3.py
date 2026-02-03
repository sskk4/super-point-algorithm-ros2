from pxr import UsdPhysics, Gf
import omni.usd
import omni.kit.commands

stage = omni.usd.get_context().get_stage()

TRACK_ROOT = "/World/Robot/Track"
JOINT_ROOT = f"{TRACK_ROOT}/Joints"
PREFIX = "DEFAULT_"

SEQUENCE = [
    68, 101, 135, 35, 83, 117, 159, 67, 100, 134, 33, 82, 116, 150, 66, 99, 133, 32,
    158, 157, 156, 155, 154, 153, 152, 151, 115, 15, 74, 107, 141, 49, 90, 124, 14,
    125, 91, 51, 142, 108, 75, 18, 126, 92, 54, 143, 109, 76, 20, 127, 93, 55, 144,
    110, 77, 21, 128, 94, 58, 145, 111, 78, 24, 129, 95, 59, 146, 112, 79, 25, 130,
    96, 62, 147, 113, 80, 28, 131, 97, 63, 148, 114, 81, 29, 132, 98, 65, 149, 87,
    140, 106, 73, 11, 123, 89, 48, 45, 88, 122, 10, 72, 105, 139, 44, 121, 8, 71,
    104, 138, 41, 86, 120, 6, 70, 103, 137, 39, 85, 119, 3, 69, 102, 136, 38, 84,
    118, 2
]

# --------------------------------------------------

def ensure_xform(path):
    if not stage.GetPrimAtPath(path).IsValid():
        omni.kit.commands.execute(
            "CreatePrim",
            prim_path=path,
            prim_type="Xform"
        )

ensure_xform(JOINT_ROOT)

# --------------------------------------------------

created = 0
skipped = 0

for i in range(len(SEQUENCE)):
    a = SEQUENCE[i]
    b = SEQUENCE[(i + 1) % len(SEQUENCE)]

    path_a = f"{TRACK_ROOT}/{PREFIX}{a}"
    path_b = f"{TRACK_ROOT}/{PREFIX}{b}"

    if not stage.GetPrimAtPath(path_a).IsValid():
        skipped += 1
        continue
    if not stage.GetPrimAtPath(path_b).IsValid():
        skipped += 1
        continue

    joint_path = f"{JOINT_ROOT}/joint_{a}_to_{b}"

    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=joint_path,
        prim_type="PhysicsRevoluteJoint"
    )

    joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(joint_path))

    joint.CreateBody0Rel().SetTargets([path_a])
    joint.CreateBody1Rel().SetTargets([path_b])

    # 🔑 KORZYSTAMY Z PIVOTÓW
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    # Oś obrotu – DOPASUJ jeśli potrzeba
    joint.CreateAxisAttr().Set("X")

    # Limity
    joint.CreateLowerLimitAttr(-30.0)
    joint.CreateUpperLimitAttr(30.0)

    created += 1

print(f"\n✅ GOTOWE")
print(f"   ✔️ Jointy utworzone: {created}")
print(f"   ⚠️ Pominięte: {skipped}")
