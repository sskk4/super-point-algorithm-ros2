from pxr import UsdPhysics, UsdGeom, Gf, PhysxSchema
import omni.usd
import omni.kit.commands

stage = omni.usd.get_context().get_stage()

TRACK_ROOT = "/World/g1/g1"
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

# ===================== KONFIGURACJA =====================
JOINT_AXIS = "Y"        # Oś obrotu: X, Y lub Z
ANGLE_LIMITS = 20.0     # Maksymalny kąt zgięcia (stopnie)
TRACK_MASS = 0.3        # Masa ogniwa (kg)
JOINT_DAMPING = 5.0     # Tłumienie
JOINT_STIFFNESS = 50.0  # Sztywność

# ===================== FUNKCJE =====================

def ensure_path_exists(path):
    parts = path.strip("/").split("/")
    current = ""
    for part in parts:
        current += f"/{part}"
        prim = stage.GetPrimAtPath(current)
        if not prim.IsValid():
            omni.kit.commands.execute(
                "CreatePrim",
                prim_path=current,
                prim_type="Xform"
            )

def find_mesh_in_xform(xform_path):
    prim = stage.GetPrimAtPath(xform_path)
    if not prim.IsValid():
        return None
    
    for child in prim.GetAllChildren():
        if child.GetTypeName() in ("Mesh", "Cube", "Sphere", "Capsule", "Cylinder"):
            return child
    return None

def get_pivot_position(xform_prim):
    """
    Pobiera pozycję pivota w world space
    Omniverse zapisuje pivot jako translate:pivot
    """
    xformable = UsdGeom.Xformable(xform_prim)
    
    # Pobierz translate:pivot jeśli istnieje
    pivot_op = None
    for op in xformable.GetOrderedXformOps():
        if op.GetOpName() == "translate:pivot":
            pivot_op = op
            break
    
    # Oblicz world transform
    world_xform = xformable.ComputeLocalToWorldTransform(0)
    
    if pivot_op:
        # Jeśli jest pivot, użyj go
        pivot_local = pivot_op.Get()
        if pivot_local is None:
            pivot_local = Gf.Vec3d(0, 0, 0)
        pivot_world = world_xform.Transform(pivot_local)
    else:
        # Fallback: środek transformacji
        pivot_world = world_xform.ExtractTranslation()
    
    return pivot_world

def setup_physics_on_mesh(mesh_prim):
    """Konfiguruje physics z convex hull"""
    if mesh_prim is None:
        return
    
    # RigidBody
    if not mesh_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rb = UsdPhysics.RigidBodyAPI.Apply(mesh_prim)
        rb.CreateRigidBodyEnabledAttr(True)
    
    # Collision
    if not mesh_prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(mesh_prim)
    
    # Convex Hull
    if not mesh_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
        mesh_collision.CreateApproximationAttr("convexHull")
    else:
        mesh_collision = UsdPhysics.MeshCollisionAPI(mesh_prim)
        mesh_collision.GetApproximationAttr().Set("convexHull")
    
    # Masa
    mass_api = UsdPhysics.MassAPI.Apply(mesh_prim)
    mass_api.CreateMassAttr(TRACK_MASS)

# ===================== KROK 1: PHYSICS =====================

ensure_path_exists(JOINT_ROOT)

print("🔧 KROK 1: Konfiguracja physics...")
configured = 0

for idx in SEQUENCE:
    xform_path = f"{TRACK_ROOT}/{PREFIX}{idx}"
    mesh = find_mesh_in_xform(xform_path)
    if mesh:
        setup_physics_on_mesh(mesh)
        configured += 1

print(f"✅ Skonfigurowano {configured} elementów\n")

# ===================== KROK 2: JOINTY =====================

print("🔗 KROK 2: Tworzenie jointów...")
created = 0
skipped = 0

for i in range(len(SEQUENCE)):
    n0 = SEQUENCE[i]
    n1 = SEQUENCE[(i + 1) % len(SEQUENCE)]
    
    xform0_path = f"{TRACK_ROOT}/{PREFIX}{n0}"
    xform1_path = f"{TRACK_ROOT}/{PREFIX}{n1}"
    
    xform0_prim = stage.GetPrimAtPath(xform0_path)
    xform1_prim = stage.GetPrimAtPath(xform1_path)
    
    mesh0 = find_mesh_in_xform(xform0_path)
    mesh1 = find_mesh_in_xform(xform1_path)
    
    if not xform0_prim.IsValid() or not xform1_prim.IsValid():
        print(f"⚠️ Xform nie istnieje: {n0} lub {n1}")
        skipped += 1
        continue
    
    if mesh0 is None or mesh1 is None:
        print(f"⚠️ Brak mesh: {n0} lub {n1}")
        skipped += 1
        continue
    
    # === POZYCJE PIVOTÓW W WORLD SPACE ===
    pivot0_world = get_pivot_position(xform0_prim)
    pivot1_world = get_pivot_position(xform1_prim)
    
    # === PUNKT POŁĄCZENIA (środek między pivotami) ===
    joint_world_pos = Gf.Vec3d(
        (pivot0_world[0] + pivot1_world[0]) * 0.5,
        (pivot0_world[1] + pivot1_world[1]) * 0.5,
        (pivot0_world[2] + pivot1_world[2]) * 0.5
    )
    
    # === POZYCJE LOKALNE (względem Xform z pivotem!) ===
    # Wektor od pivot0 do joint
    vec = joint_world_pos - pivot0_world
    local_pos0 = vec
    
    # Wektor od pivot1 do joint
    vec = joint_world_pos - pivot1_world
    local_pos1 = vec
    
    # Debug co 10 elementów
    if created % 10 == 0:
        print(f"\n  Joint {n0}→{n1}:")
        print(f"    Pivot0: {pivot0_world}")
        print(f"    Pivot1: {pivot1_world}")
        print(f"    Joint pos: {joint_world_pos}")
        print(f"    Local0: {local_pos0}")
        print(f"    Local1: {local_pos1}")
    
    # === TWORZENIE JOINTA ===
    joint_path = f"{JOINT_ROOT}/joint_{n0}_to_{n1}"
    
    old_joint = stage.GetPrimAtPath(joint_path)
    if old_joint.IsValid():
        stage.RemovePrim(joint_path)
    
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=joint_path,
        prim_type="PhysicsRevoluteJoint"
    )
    
    joint_prim = stage.GetPrimAtPath(joint_path)
    joint = UsdPhysics.RevoluteJoint(joint_prim)
    
    # === RELACJE ===
    joint.CreateBody0Rel().SetTargets([mesh0.GetPath()])
    joint.CreateBody1Rel().SetTargets([mesh1.GetPath()])
    
    # === POZYCJE LOKALNE ===
    joint.CreateLocalPos0Attr().Set(local_pos0)
    joint.CreateLocalPos1Attr().Set(local_pos1)
    
    # === LOKALNE ROTACJE (opcjonalne, ale zalecane) ===
    # Ustaw rotacje na identyczność
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    
    # === OŚ I LIMITY ===
    joint.CreateAxisAttr(JOINT_AXIS)
    joint.CreateLowerLimitAttr(-ANGLE_LIMITS)
    joint.CreateUpperLimitAttr(ANGLE_LIMITS)
    
    # === PHYSX PARAMETRY ===
    if not joint_prim.HasAPI(PhysxSchema.PhysxJointAPI):
        physx_joint = PhysxSchema.PhysxJointAPI.Apply(joint_prim)
    else:
        physx_joint = PhysxSchema.PhysxJointAPI(joint_prim)
    
    physx_joint.CreateJointFrictionAttr(0.5)
    
    # Drive (sztywność gąsienicy)
    joint.CreateDriveTypeAttr("force")
    joint.CreateDriveTargetAttr(0.0)
    joint.CreateDriveStiffnessAttr(JOINT_STIFFNESS)
    joint.CreateDriveDampingAttr(JOINT_DAMPING)
    
    created += 1

print("\n" + "="*60)
print("✅ ZAKOŃCZONO")
print("="*60)
print(f"✔️ Physics: {configured} elementów")
print(f"✔️ Jointy: {created}")
print(f"⚠️ Pominięto: {skipped}")
print("="*60)
print(f"⚙️  Oś jointa: {JOINT_AXIS}")
print(f"⚙️  Limity: ±{ANGLE_LIMITS}°")
print(f"⚙️  Masa ogniwa: {TRACK_MASS} kg")
print("="*60)
