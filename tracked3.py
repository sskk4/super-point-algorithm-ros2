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

def setup_physics_on_mesh(mesh_prim):
    """
    Konfiguruje physics z CONVEX HULL zamiast triangle mesh
    """
    if mesh_prim is None:
        return
    
    # RigidBody API
    if not mesh_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rb = UsdPhysics.RigidBodyAPI.Apply(mesh_prim)
        rb.CreateRigidBodyEnabledAttr(True)
    
    # Collision API
    if not mesh_prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(mesh_prim)
    
    # KLUCZOWA ZMIANA: CONVEX HULL zamiast triangle mesh
    if not mesh_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
        # "convexHull" zamiast domyślnego "meshSimplification"
        mesh_collision.CreateApproximationAttr("convexHull")
    else:
        mesh_collision = UsdPhysics.MeshCollisionAPI(mesh_prim)
        mesh_collision.GetApproximationAttr().Set("convexHull")
    
    # Masa i inne parametry
    mass_api = UsdPhysics.MassAPI.Apply(mesh_prim)
    mass_api.CreateMassAttr(0.5)  # 0.5 kg na element gąsienicy

def get_world_transform(prim):
    xformable = UsdGeom.Xformable(prim)
    return xformable.ComputeLocalToWorldTransform(0)

def get_midpoint(p1, p2):
    return Gf.Vec3d(
        (p1[0] + p2[0]) * 0.5,
        (p1[1] + p2[1]) * 0.5,
        (p1[2] + p2[2]) * 0.5,
    )

# ===================== START =====================
ensure_path_exists(JOINT_ROOT)

print("🔧 KROK 1: Konfiguracja physics na wszystkich elementach...")
configured = 0
for idx in SEQUENCE:
    xform_path = f"{TRACK_ROOT}/{PREFIX}{idx}"
    mesh = find_mesh_in_xform(xform_path)
    if mesh:
        setup_physics_on_mesh(mesh)
        configured += 1

print(f"✅ Skonfigurowano {configured} elementów z convex hull\n")

print("🔗 KROK 2: Tworzenie jointów...")
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
        print(f"⚠️ Pomijam {PREFIX}{n0} ↔ {PREFIX}{n1}")
        skipped += 1
        continue
    
    # === WORLD TRANSFORMS ===
    xf0 = get_world_transform(mesh0)
    xf1 = get_world_transform(mesh1)
    
    world_pos0 = xf0.ExtractTranslation()
    world_pos1 = xf1.ExtractTranslation()
    
    joint_world_pos = get_midpoint(world_pos0, world_pos1)
    
    # === KONWERSJA DO LOCAL SPACE ===
    inv_xf0 = xf0.GetInverse()
    inv_xf1 = xf1.GetInverse()
    
    local_pos0 = inv_xf0.Transform(joint_world_pos)
    local_pos1 = inv_xf1.Transform(joint_world_pos)
    
    # === TWORZENIE JOINTA ===
    joint_path = f"{JOINT_ROOT}/joint_{n0}_to_{n1}"
    
    # Usuń stary joint jeśli istnieje
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
    
    # === RELACJE BODY ===
    joint.CreateBody0Rel().SetTargets([mesh0.GetPath()])
    joint.CreateBody1Rel().SetTargets([mesh1.GetPath()])
    
    # === POZYCJE LOKALNE ===
    joint.CreateLocalPos0Attr().Set(local_pos0)
    joint.CreateLocalPos1Attr().Set(local_pos1)
    
    # === OŚ OBROTU ===
    # Dla gąsienicy zwykle Y (pionowa) lub Z (głębokość)
    joint.CreateAxisAttr("Y")  # Zmień na "Z" jeśli oś jest inna
    
    # === LIMITY KĄTOWE ===
    joint.CreateLowerLimitAttr(-15.0)  # Mniejszy zakres dla stabilności
    joint.CreateUpperLimitAttr(15.0)
    
    # === OPCJONALNIE: Damping i stiffness ===
    # Zapobiega chaotycznym ruchom
    if joint_prim.HasAPI(PhysxSchema.PhysxJointAPI):
        physx_joint = PhysxSchema.PhysxJointAPI(joint_prim)
    else:
        physx_joint = PhysxSchema.PhysxJointAPI.Apply(joint_prim)
    
    # Tłumienie kątowe
    physx_joint.CreateJointFrictionAttr(0.1)
    
    created += 1
    if created % 20 == 0:
        print(f"⏳ Utworzono {created}/{len(SEQUENCE)} jointów")

print("\n✅ ZAKOŃCZONO")
print(f"✔️ Utworzono jointów: {created}")
print(f"⚠️ Pominięto: {skipped}")
print("\n💡 WSKAZÓWKI:")
print("   - Jeśli elementy nadal latają, sprawdź oś jointa (Y/Z/X)")
print("   - Możesz zmniejszyć limity kątowe (-5° do 5°)")
print("   - Upewnij się że masa pojazdu > suma mas gąsienicy")
