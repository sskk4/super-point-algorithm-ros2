from pxr import UsdPhysics, UsdGeom, Gf
import omni.usd
import omni.kit.commands

# ===================== KONFIGURACJA =====================

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

# ===================== FUNKCJE =====================

def ensure_path_exists(path):
    parts = path.strip("/").split("/")
    current = ""
    for part in parts:
        current += f"/{part}"
        if not stage.GetPrimAtPath(current).IsValid():
            omni.kit.commands.execute(
                "CreatePrim",
                prim_path=current,
                prim_type="Xform"
            )

def get_world_position(prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None

    xformable = UsdGeom.Xformable(prim)
    world_transform = xformable.ComputeLocalToWorldTransform(0)
    return world_transform.ExtractTranslation()

def get_midpoint(p1, p2):
    return Gf.Vec3d(
        (p1[0] + p2[0]) * 0.5,
        (p1[1] + p2[1]) * 0.5,
        (p1[2] + p2[2]) * 0.5
    )

def world_to_local(prim_path, world_pos):
    prim = stage.GetPrimAtPath(prim_path)
    xformable = UsdGeom.Xformable(prim)

    world_tf = xformable.ComputeLocalToWorldTransform(0)
    inv_tf = world_tf.GetInverse()

    return inv_tf.Transform(world_pos)

# ===================== START =====================

ensure_path_exists(JOINT_ROOT)

created = 0
skipped = 0

for i in range(len(SEQUENCE)):
    n0 = SEQUENCE[i]
    n1 = SEQUENCE[(i + 1) % len(SEQUENCE)]

    body0_path = f"{TRACK_ROOT}/{PREFIX}{n0}"
    body1_path = f"{TRACK_ROOT}/{PREFIX}{n1}"

    if not stage.GetPrimAtPath(body0_path).IsValid():
        print(f"⚠️ Brak: {PREFIX}{n0}")
        skipped += 1
        continue

    if not stage.GetPrimAtPath(body1_path).IsValid():
        print(f"⚠️ Brak: {PREFIX}{n1}")
        skipped += 1
        continue

    pos0 = get_world_position(body0_path)
    pos1 = get_world_position(body1_path)

    if pos0 is None or pos1 is None:
        skipped += 1
        continue

    joint_world_pos = get_midpoint(pos0, pos1)

    joint_name = f"joint_{n0}_to_{n1}"
    joint_path = f"{JOINT_ROOT}/{joint_name}"

    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=joint_path,
        prim_type="PhysicsRevoluteJoint"
    )

    joint_prim = stage.GetPrimAtPath(joint_path)
    joint = UsdPhysics.RevoluteJoint(joint_prim)

    local0 = world_to_local(body0_path, joint_world_pos)
    local1 = world_to_local(body1_path, joint_world_pos)

    joint.CreateBody0Rel().SetTargets([body0_path])
    joint.CreateBody1Rel().SetTargets([body1_path])

    joint.CreateLocalPos0Attr(local0)
    joint.CreateLocalPos1Attr(local1)

    joint.CreateAxisAttr("X")
    joint.CreateLowerLimitAttr(-30.0)
    joint.CreateUpperLimitAttr(30.0)

    created += 1

    if created % 20 == 0:
        print(f"⏳ {created}/{len(SEQUENCE)} jointów")

print("\n✅ GOTOWE")
print(f"✔️ Utworzono jointów: {created}")
print(f"⚠️ Pominięto: {skipped}")
