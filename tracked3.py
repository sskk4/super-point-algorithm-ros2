from pxr import UsdPhysics, UsdGeom, Gf
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

def find_mesh_prim(xform_path):
    """Znajduje pierwszy Mesh pod danym Xform"""
    prim = stage.GetPrimAtPath(xform_path)
    if not prim.IsValid():
        return None
    for child in prim.GetAllChildren():
        if child.GetTypeName() in ("Mesh", "Cube", "Sphere", "Capsule", "Cylinder"):
            return child
    return None  # jeśli nie ma mesh, zwraca None

def ensure_rigid_body(prim):
    if prim is None:
        return
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)

def get_world_position(prim):
    xformable = UsdGeom.Xformable(prim)
    xf = xformable.ComputeLocalToWorldTransform(0)
    return xf.ExtractTranslation(), xf

def get_midpoint(p1, p2):
    return Gf.Vec3d(
        (p1[0] + p2[0]) * 0.5,
        (p1[1] + p2[1]) * 0.5,
        (p1[2] + p2[2]) * 0.5,
    )

# ===================== START =====================

ensure_path_exists(JOINT_ROOT)

created = 0
skipped = 0

for i in range(len(SEQUENCE)):
    n0 = SEQUENCE[i]
    n1 = SEQUENCE[(i + 1) % len(SEQUENCE)]

    xform0_path = f"{TRACK_ROOT}/{PREFIX}{n0}"
    xform1_path = f"{TRACK_ROOT}/{PREFIX}{n1}"

    mesh0 = find_mesh_prim(xform0_path)
    mesh1 = find_mesh_prim(xform1_path)

    if mesh0 is None or mesh1 is None:
        print(f"⚠️ Pomijam {PREFIX}{n0} lub {PREFIX}{n1} – brak mesh")
        skipped += 1
        continue

    # === WYMUSZ RIGID BODY + COLLISION ===
    ensure_rigid_body(mesh0)
    ensure_rigid_body(mesh1)

    # === WORLD POSITIONS ===
    world_pos0, xf0 = get_world_position(mesh0)
    world_pos1, xf1 = get_world_position(mesh1)

    joint_world_pos = get_midpoint(world_pos0, world_pos1)

    # === CREATE JOINT ===
    joint_path = f"{JOINT_ROOT}/joint_{n0}_to_{n1}"
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=joint_path,
        prim_type="PhysicsRevoluteJoint"
    )

    joint_prim = stage.GetPrimAtPath(joint_path)
    joint = UsdPhysics.RevoluteJoint(joint_prim)

    # === WORLD -> LOCAL ===
    local0 = xf0.GetInverse().Transform(joint_world_pos)
    local1 = xf1.GetInverse().Transform(joint_world_pos)

    # === BODY RELATIONS ===
    joint.CreateBody0Rel().SetTargets([mesh0.GetPath()])
    joint.CreateBody1Rel().SetTargets([mesh1.GetPath()])

    # === LOCAL POSITIONS ===
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
