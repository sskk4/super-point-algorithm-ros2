from pxr import UsdPhysics, UsdGeom, Gf, PhysxSchema
import omni.usd
import omni.kit.commands

stage = omni.usd.get_context().get_stage()

TRACK_ROOT = "/World/g1/g1"
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

def find_mesh_in_xform(xform_path):
    prim = stage.GetPrimAtPath(xform_path)
    if not prim.IsValid():
        return None
    
    for child in prim.GetAllChildren():
        if child.GetTypeName() in ("Mesh", "Cube", "Sphere", "Capsule", "Cylinder"):
            return child
    return None

def get_mesh_bbox_center(mesh_prim):
    """
    Oblicza środek bounding box mesha w world space
    """
    mesh = UsdGeom.Mesh(mesh_prim)
    points_attr = mesh.GetPointsAttr()
    
    if not points_attr:
        return None
    
    points = points_attr.Get()
    if not points or len(points) == 0:
        return None
    
    # Oblicz AABB (axis-aligned bounding box)
    min_point = Gf.Vec3f(float('inf'), float('inf'), float('inf'))
    max_point = Gf.Vec3f(float('-inf'), float('-inf'), float('-inf'))
    
    for pt in points:
        min_point[0] = min(min_point[0], pt[0])
        min_point[1] = min(min_point[1], pt[1])
        min_point[2] = min(min_point[2], pt[2])
        max_point[0] = max(max_point[0], pt[0])
        max_point[1] = max(max_point[1], pt[1])
        max_point[2] = max(max_point[2], pt[2])
    
    # Środek bbox w LOCAL space mesha
    local_center = Gf.Vec3d(
        (min_point[0] + max_point[0]) * 0.5,
        (min_point[1] + max_point[1]) * 0.5,
        (min_point[2] + max_point[2]) * 0.5
    )
    
    # Przekształć do world space
    mesh_xformable = UsdGeom.Xformable(mesh_prim)
    mesh_world_xform = mesh_xformable.ComputeLocalToWorldTransform(0)
    world_center = mesh_world_xform.Transform(local_center)
    
    return world_center

def fix_pivot_for_element(xform_path):
    """
    GŁÓWNA FUNKCJA: Naprawia pivot dla pojedynczego elementu gąsienicy
    
    Struktura:
    DEFAULT_68 (Xform) <- tutaj ustawiamy nowy pivot
      └─ mesh (Mesh)   <- tego geometria zostaje w miejscu
    """
    xform_prim = stage.GetPrimAtPath(xform_path)
    if not xform_prim.IsValid():
        return False
    
    mesh_prim = find_mesh_in_xform(xform_path)
    if not mesh_prim:
        return False
    
    # 1. Pobierz środek geometrii w world space
    bbox_center = get_mesh_bbox_center(mesh_prim)
    if bbox_center is None:
        return False
    
    # 2. Obecna transformacja Xform
    xform = UsdGeom.Xformable(xform_prim)
    current_xform_matrix = xform.ComputeLocalToWorldTransform(0)
    current_position = current_xform_matrix.ExtractTranslation()
    
    # 3. Offset który musimy dodać do Xform
    offset = bbox_center - current_position
    
    # 4. Przesuń Xform do środka geometrii
    new_position = bbox_center
    
    # 5. Oblicz przeciwny offset dla mesha (żeby został w miejscu)
    mesh_xform = UsdGeom.Xformable(mesh_prim)
    
    # Ustaw nową pozycję dla Xform
    xform.ClearXformOpOrder()
    translate_op = xform.AddTranslateOp()
    translate_op.Set(new_position)
    
    # Kompensuj przesunięcie w meshu (żeby geometria nie uciekła)
    mesh_xform.ClearXformOpOrder()
    mesh_translate_op = mesh_xform.AddTranslateOp()
    mesh_translate_op.Set(-offset)  # Przeciwny offset
    
    return True

# ===================== KROK 1: NAPRAWA PIVOTÓW =====================

print("🔧 NAPRAWA PIVOTÓW GĄSIENICY")
print("="*60)

fixed = 0
failed = 0

for idx in SEQUENCE:
    xform_path = f"{TRACK_ROOT}/{PREFIX}{idx}"
    
    if fix_pivot_for_element(xform_path):
        fixed += 1
        if fixed % 20 == 0:
            print(f"⏳ Naprawiono {fixed}/{len(SEQUENCE)} elementów...")
    else:
        print(f"⚠️ Nie udało się naprawić {PREFIX}{idx}")
        failed += 1

print("\n✅ PIVOTY NAPRAWIONE")
print(f"✔️ Naprawiono: {fixed}")
print(f"❌ Błędów: {failed}")

# ===================== KROK 2: PHYSICS =====================

def setup_physics_on_mesh(mesh_prim):
    if mesh_prim is None:
        return
    
    if not mesh_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rb = UsdPhysics.RigidBodyAPI.Apply(mesh_prim)
        rb.CreateRigidBodyEnabledAttr(True)
    
    if not mesh_prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(mesh_prim)
    
    if not mesh_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
        mesh_collision.CreateApproximationAttr("convexHull")
    else:
        mesh_collision = UsdPhysics.MeshCollisionAPI(mesh_prim)
        mesh_collision.GetApproximationAttr().Set("convexHull")
    
    mass_api = UsdPhysics.MassAPI.Apply(mesh_prim)
    mass_api.CreateMassAttr(0.3)

print("\n🔧 KONFIGURACJA PHYSICS...")

configured = 0
for idx in SEQUENCE:
    xform_path = f"{TRACK_ROOT}/{PREFIX}{idx}"
    mesh = find_mesh_in_xform(xform_path)
    if mesh:
        setup_physics_on_mesh(mesh)
        configured += 1

print(f"✅ Skonfigurowano physics: {configured}")

# ===================== KROK 3: JOINTY =====================

JOINT_ROOT = f"{TRACK_ROOT}/Joints"
JOINT_AXIS = "Y"  # Dostosuj: X, Y lub Z
ANGLE_LIMITS = 20.0

def ensure_path_exists(path):
    parts = path.strip("/").split("/")
    current = ""
    for part in parts:
        current += f"/{part}"
        prim = stage.GetPrimAtPath(current)
        if not prim.IsValid():
            omni.kit.commands.execute("CreatePrim", prim_path=current, prim_type="Xform")

ensure_path_exists(JOINT_ROOT)

print("\n🔗 TWORZENIE JOINTÓW...")

created = 0
skipped = 0

for i in range(len(SEQUENCE)):
    n0 = SEQUENCE[i]
    n1 = SEQUENCE[(i + 1) % len(SEQUENCE)]
    
    xform0_path = f"{TRACK_ROOT}/{PREFIX}{n0}"
    xform1_path = f"{TRACK_ROOT}/{PREFIX}{n1}"
    
    mesh0 = find_mesh_in_xform(xform0_path)
    mesh1 = find_mesh_in_xform(xform1_path)
    
    if mesh0 is None or mesh1 is None:
        skipped += 1
        continue
    
    # Teraz pozycje Xform = środki geometrii!
    xform0 = UsdGeom.Xformable(stage.GetPrimAtPath(xform0_path))
    xform1 = UsdGeom.Xformable(stage.GetPrimAtPath(xform1_path))
    
    pos0 = xform0.ComputeLocalToWorldTransform(0).ExtractTranslation()
    pos1 = xform1.ComputeLocalToWorldTransform(0).ExtractTranslation()
    
    # Punkt połączenia: punkt styku między elementami
    vec = pos1 - pos0
    joint_world_pos = pos0 + vec * 0.5  # środek między pivotami
    
    # Pozycje lokalne (teraz proste, bo pivoty są poprawne!)
    local_pos0 = Gf.Vec3d(vec[0] * 0.5, vec[1] * 0.5, vec[2] * 0.5)
    local_pos1 = Gf.Vec3d(-vec[0] * 0.5, -vec[1] * 0.5, -vec[2] * 0.5)
    
    # Tworzenie jointa
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
    
    joint.CreateBody0Rel().SetTargets([mesh0.GetPath()])
    joint.CreateBody1Rel().SetTargets([mesh1.GetPath()])
    
    joint.CreateLocalPos0Attr().Set(local_pos0)
    joint.CreateLocalPos1Attr().Set(local_pos1)
    
    joint.CreateAxisAttr(JOINT_AXIS)
    joint.CreateLowerLimitAttr(-ANGLE_LIMITS)
    joint.CreateUpperLimitAttr(ANGLE_LIMITS)
    
    # PhysX parameters
    if not joint_prim.HasAPI(PhysxSchema.PhysxJointAPI):
        physx_joint = PhysxSchema.PhysxJointAPI.Apply(joint_prim)
    else:
        physx_joint = PhysxSchema.PhysxJointAPI(joint_prim)
    
    physx_joint.CreateJointFrictionAttr(0.5)
    
    created += 1
    if created % 20 == 0:
        print(f"⏳ Utworzono {created}/{len(SEQUENCE)} jointów")

print("\n" + "="*60)
print("✅ ZAKOŃCZONO")
print("="*60)
print(f"✔️ Pivoty naprawione: {fixed}")
print(f"✔️ Jointy utworzone: {created}")
print(f"⚠️ Pominięto: {skipped}")
print("="*60)
