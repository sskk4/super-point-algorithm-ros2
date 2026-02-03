from pxr import Usd, UsdPhysics, UsdGeom, Gf
import omni.kit.commands
import omni.usd

stage = omni.usd.get_context().get_stage()

TRACK_ROOT = "/World/Robot/Track"
JOINT_ROOT = f"{TRACK_ROOT}/Joints"
PREFIX = "DEFAULT_"

SEQUENCE = [ 68, 101, 135, 35, 83, 117, 159, 67, 100, 134, 33, 82, 116, 150, 66, 99, 133, 32, 158, 157, 156, 155, 154, 153, 152, 151, 115, 15, 74, 107, 141, 49, 90, 124, 14, 125, 91, 51, 142, 108, 75, 18, 126, 92, 54, 143, 109, 76, 20, 127, 93, 55, 144, 110, 77, 21, 128, 94, 58, 145, 111, 78, 24, 129, 95, 59, 146, 112, 79, 25, 130, 96, 62, 147, 113, 80, 28, 131, 97, 63, 148, 114, 81, 29, 132, 98, 65, 149, 87, 140, 106, 73, 11, 123, 89, 48, 45, 88, 122, 10, 72, 105, 139, 44, 121, 8, 71, 104, 138, 41, 86, 120, 6, 70, 103, 137, 39, 85, 119, 3, 69, 102, 136, 38, 84, 118, 2 ]

# ===================== FUNKCJE =====================
def ensure_path_exists(path):
    parts = path.strip("/").split("/")
    current = ""
    for part in parts:
        current += f"/{part}"
        if not stage.GetPrimAtPath(current).IsValid():
            omni.kit.commands.execute("CreatePrim", prim_path=current, prim_type="Xform")


def has_valid_mesh(xform_prim):
    """Sprawdza czy pod Xformem jest Mesh"""
    if not xform_prim.IsValid():
        return False
    for child in xform_prim.GetChildren():
        if child.GetTypeName() == "Mesh":
            return True
    return False


# ===================== GŁÓWNA PĘTLA =====================
ensure_path_exists(JOINT_ROOT)

created = 0
skipped = 0

for i in range(len(SEQUENCE)):
    n0 = SEQUENCE[i]
    n1 = SEQUENCE[(i + 1) % len(SEQUENCE)]

    xform0_path = f"{TRACK_ROOT}/{PREFIX}{n0}"
    xform1_path = f"{TRACK_ROOT}/{PREFIX}{n1}"

    prim0 = stage.GetPrimAtPath(xform0_path)
    prim1 = stage.GetPrimAtPath(xform1_path)

    if not prim0.IsValid() or not prim1.IsValid():
        print(f"⚠️ Brak prim: {n0} lub {n1}")
        skipped += 1
        continue

    if not has_valid_mesh(prim0) or not has_valid_mesh(prim1):
        print(f"⚠️ Brak Mesh pod: {n0} lub {n1}")
        skipped += 1
        continue

    # === NAJWAŻNIEJSZA POPRAWKA ===
    # RigidBody i Collision NA XFORMIE, nie na Meshu!
    if not prim0.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim0)
    if not prim0.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim0)

    if not prim1.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim1)
    if not prim1.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim1)

    # === WORLD TRANSFORM (na Xformie!) ===
    xf0 = UsdGeom.Xformable(prim0).ComputeLocalToWorldTransform(0)
    xf1 = UsdGeom.Xformable(prim1).ComputeLocalToWorldTransform(0)

    world_pos0 = xf0.ExtractTranslation()
    world_pos1 = xf1.ExtractTranslation()

    joint_world_pos = Gf.Vec3d(
        (world_pos0[0] + world_pos1[0]) * 0.5,
        (world_pos0[1] + world_pos1[1]) * 0.5,
        (world_pos0[2] + world_pos1[2]) * 0.5,
    )

    # === DEBUG (bardzo ważny!) ===
    print(f"[{n0} → {n1}]  World0: {world_pos0}   World1: {world_pos1}   Joint: {joint_world_pos}")

    local0 = xf0.GetInverse().Transform(joint_world_pos)
    local1 = xf1.GetInverse().Transform(joint_world_pos)

    # === CREATE JOINT ===
    joint_path = f"{JOINT_ROOT}/joint_{n0}_to_{n1}"
    omni.kit.commands.execute("CreatePrim", prim_path=joint_path, prim_type="PhysicsRevoluteJoint")

    joint = UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(joint_path))

    joint.CreateBody0Rel().SetTargets([xform0_path])
    joint.CreateBody1Rel().SetTargets([xform1_path])

    joint.CreateLocalPos0Attr().Set(local0)
    joint.CreateLocalPos1Attr().Set(local1)

    joint.CreateAxisAttr("X")
    joint.CreateLowerLimitAttr(-60.0)
    joint.CreateUpperLimitAttr(60.0)
    joint.CreateDriveDampingAttr(100.0)     # opcjonalnie
    joint.CreateDriveStiffnessAttr(500.0)   # opcjonalnie

    created += 1
    if created % 30 == 0:
        print(f"⏳ Postęp: {created}/{len(SEQUENCE)} jointów")

print("\n" + "="*60)
print(f"✅ ZAKOŃCZONO!")
print(f"   Utworzono: {created}")
print(f"   Pominięto: {skipped}")
