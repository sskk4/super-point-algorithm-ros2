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

# ===================== KONFIGURACJA GĄSIENICY =====================
# Dostosuj te wartości do swojego modelu:

JOINT_AXIS = "Y"  # Oś obrotu: "X", "Y" lub "Z"
                  # Y = pionowa, Z = głębokość, X = szerokość

ANGLE_LIMITS = 25.0  # Maksymalny kąt zgięcia (stopnie)
                     # Zwiększ dla bardziej elastycznej gąsienicy

TRACK_MASS = 0.3     # Masa pojedynczego ogniwa (kg)

JOINT_STIFFNESS = 100.0   # Sztywność połączenia (N⋅m/rad)
                          # Wyższa = sztywniejsza gąsienica

JOINT_DAMPING = 10.0      # Tłumienie (N⋅m⋅s/rad)
                          # Wyższa = mniej drgań

# ===================================================================

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
    if mesh_prim is None:
        return
    
    # RigidBody
    if not mesh_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rb = UsdPhysics.RigidBodyAPI.Apply(mesh_prim)
        rb.CreateRigidBodyEnabledAttr(True)
    
    # Collision
    if not mesh_prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(mesh_prim)
    
    # Convex Hull collision
    if not mesh_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
        mesh_collision.CreateApproximationAttr("convexHull")
    else:
        mesh_collision = UsdPhysics.MeshCollisionAPI(mesh_prim)
        mesh_collision.GetApproximationAttr().Set("convexHull")
    
    # Masa
    mass_api = UsdPhysics.MassAPI.Apply(mesh_prim)
    mass_api.CreateMassAttr(TRACK_MASS)

def get_local_pos_for_joint(body_prim, joint_world_pos):
    """
    KLUCZOWA FUNKCJA: Oblicza LOCAL position względem PARENT Xform
    USD Physics wymaga pozycji względem rodzica body, nie samego body!
    """
    # Pobierz parent (Xform)
    parent = body_prim.GetParent()
    
    if parent and parent.IsValid():
        # Transformacja parent -> world
        parent_xformable = UsdGeom.Xformable(parent)
        parent_world_xform = parent_xformable.ComputeLocalToWorldTransform(0)
        
        # World -> parent local
        inv_parent_xform = parent_world_xform.GetInverse()
        local_pos = inv_parent_xform.Transform(joint_world_pos)
        
        return local_pos
    else:
        # Fallback: jeśli nie ma parent, użyj samego body
        body_xformable = UsdGeom.Xformable(body_prim)
        body_world_xform = body_xformable.ComputeLocalToWorldTransform(0)
        inv_body_xform = body_world_xform.GetInverse()
        return inv_body_xform.Transform(joint_world_pos)

def get_world_position(prim):
    """Pobiera pozycję w world space"""
    xformable = UsdGeom.Xformable(prim)
    world_xform = xformable.ComputeLocalToWorldTransform(0)
    return world_xform.ExtractTranslation()

def get_contact_point(pos0, pos1):
    """
    Oblicza punkt styku między dwoma ogniwami gąsienicy
    Zamiast środka, używa punktu bliżej krawędzi
    """
    # Wektor między elementami
    vec = pos1 - pos0
    distance = vec.GetLength()
    
    # Punkt styku: 40% drogi od pos0 do pos1
    # (dostosuj % jeśli elementy nachodzą na siebie)
    contact_point = pos0 + vec * 0.4
    
    return contact_point

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
    
    mesh0 = find_mesh_in_xform(xform0_path)
    mesh1 = find_mesh_in_xform(xform1_path)
    
    if mesh0 is None or mesh1 is None:
        skipped += 1
        continue
    
    # === POZYCJE W WORLD SPACE ===
    world_pos0 = get_world_position(mesh0)
    world_pos1 = get_world_position(mesh1)
    
    # Punkt styku (nie środek!)
    joint_world_pos = get_contact_point(world_pos0, world_pos1)
    
    # === KONWERSJA DO LOCAL SPACE (względem PARENT!) ===
    local_pos0 = get_local_pos_for_joint(mesh0, joint_world_pos)
    local_pos1 = get_local_pos_for_joint(mesh1, joint_world_pos)
    
    # Debug co 10 jointów
    if created % 10 == 0:
        print(f"\n  Joint {n0}→{n1}:")
        print(f"    World pos: {joint_world_pos}")
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
    
    # === OŚ I LIMITY ===
    joint.CreateAxisAttr(JOINT_AXIS)
    joint.CreateLowerLimitAttr(-ANGLE_LIMITS)
    joint.CreateUpperLimitAttr(ANGLE_LIMITS)
    
    # === PHYSX PARAMETRY ===
    if not joint_prim.HasAPI(PhysxSchema.PhysxJointAPI):
        physx_joint = PhysxSchema.PhysxJointAPI.Apply(joint_prim)
    else:
        physx_joint = PhysxSchema.PhysxJointAPI(joint_prim)
    
    # Drive dla sztywności
    drive = PhysxSchema.PhysxJointAPI(joint_prim)
    drive.CreateJointFrictionAttr(1.0)
    
    # Angular drive (opcjonalne - symuluje sprężynę)
    joint.CreateDriveTypeAttr("force")  # lub "acceleration"
    joint.CreateDriveTargetAttr(0.0)    # Pozycja neutralna
    joint.CreateDriveStiffnessAttr(JOINT_STIFFNESS)
    joint.CreateDriveDampingAttr(JOINT_DAMPING)
    
    created += 1

print(f"\n✅ ZAKOŃCZONO")
print(f"✔️ Utworzono: {created}")
print(f"⚠️ Pominięto: {skipped}")

print("\n" + "="*60)
print("⚙️  PARAMETRY GĄSIENICY (dostosuj na górze skryptu):")
print("="*60)
print(f"  Oś jointa:        {JOINT_AXIS}")
print(f"  Limity kątowe:    ±{ANGLE_LIMITS}°")
print(f"  Masa ogniwa:      {TRACK_MASS} kg")
print(f"  Sztywność:        {JOINT_STIFFNESS} N⋅m/rad")
print(f"  Tłumienie:        {JOINT_DAMPING} N⋅m⋅s/rad")
print("="*60)
