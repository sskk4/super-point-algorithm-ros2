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

    prim0 = stage.GetPrimAtPath(body0_path)
    prim1 = stage.GetPrimAtPath(body1_path)

    if not prim0.IsValid() or not prim1.IsValid():
        skipped += 1
        continue

    # === WYMUSZ RIGID BODY + COLLISION ===
    if not UsdPhysics.RigidBodyAPI(prim0).IsValid():
        UsdPhysics.RigidBodyAPI.Apply(prim0)
    if not UsdPhysics.CollisionAPI(prim0).IsValid():
        UsdPhysics.CollisionAPI.Apply(prim0)

    if not UsdPhysics.RigidBodyAPI(prim1).IsValid():
        UsdPhysics.RigidBodyAPI.Apply(prim1)
    if not UsdPhysics.CollisionAPI(prim1).IsValid():
        UsdPhysics.CollisionAPI.Apply(prim1)

    # === WORLD POS SEGMENTÓW ===
    xf0 = UsdGeom.Xformable(prim0).ComputeLocalToWorldTransform(0)
    xf1 = UsdGeom.Xformable(prim1).ComputeLocalToWorldTransform(0)

    world_pos0 = xf0.ExtractTranslation()
    world_pos1 = xf1.ExtractTranslation()

    # === PUNKT JOINTA (WORLD) ===
    joint_world_pos = Gf.Vec3d(
        (world_pos0[0] + world_pos1[0]) * 0.5,
        (world_pos0[1] + world_pos1[1]) * 0.5,
        (world_pos0[2] + world_pos1[2]) * 0.5,
    )

    # === CREATE JOINT ===
    joint_path = f"{JOINT_ROOT}/joint_{n0}_to_{n1}"

    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=joint_path,
        prim_type="PhysicsRevoluteJoint"
    )

    joint_prim = stage.GetPrimAtPath(joint_path)
    joint = UsdPhysics.RevoluteJoint(joint_prim)

    # === WORLD -> LOCAL (KLUCZ) ===
    local0 = xf0.GetInverse().Transform(joint_world_pos)
    local1 = xf1.GetInverse().Transform(joint_world_pos)

    # === PODPIĘCIE BODIES ===
    joint.CreateBody0Rel().SetTargets([body0_path])
    joint.CreateBody1Rel().SetTargets([body1_path])

    # === LOCAL POSITIONS (NIE BĘDĄ 0,0,0) ===
    joint.CreateLocalPos0Attr(local0)
    joint.CreateLocalPos1Attr(local1)

    # === PARAMETRY JOINTA ===
    joint.CreateAxisAttr("X")
    joint.CreateLowerLimitAttr(-30.0)
    joint.CreateUpperLimitAttr(30.0)

    created += 1

    if created % 20 == 0:
        print(f"⏳ Utworzono {created}/{len(SEQUENCE)} jointów")

print("\n✅ ZAKOŃCZONO")
print(f"✔️ Utworzono: {created}")
print(f"⚠️ Pominięto: {skipped}")

