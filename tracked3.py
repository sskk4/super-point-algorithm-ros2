from pxr import UsdPhysics, UsdGeom, Gf
import omni.usd
import omni.kit.commands

stage = omni.usd.get_context().get_stage()

TRACK_ROOT = "/World/g1"
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
    return None

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

def get_bounding_box_center(prim):
    """Pobiera środek bounding box w przestrzeni świata"""
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    bound = bbox_cache.ComputeWorldBound(prim)
    bbox = bound.ComputeAlignedBox()
    center = (bbox.GetMin() + bbox.GetMax()) * 0.5
    return center

def get_joint_position_between_segments(prim0, prim1):
    """
    Znajduje optymalną pozycję jointa między dwoma segmentami.
    W przypadku tracku, joint powinien być na krawędziach, nie w środkach.
    """
    # Pobierz bounding boxy obu segmentów
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    
    bound0 = bbox_cache.ComputeWorldBound(prim0)
    bbox0 = bound0.ComputeAlignedBox()
    
    bound1 = bbox_cache.ComputeWorldBound(prim1)
    bbox1 = bound1.ComputeAlignedBox()
    
    # Środki bounding boxów
    center0 = (bbox0.GetMin() + bbox0.GetMax()) * 0.5
    center1 = (bbox1.GetMin() + bbox1.GetMax()) * 0.5
    
    # Wersja 1: Jeśli segmenty są ustawione wzdłuż osi Z, joint powinien być na krawędzi
    # Zakładamy, że segmenty są obrócone tak, że oś Z wskazuje kierunek toru
    
    # Sprawdź czy segmenty są blisko siebie wzdłuż osi Z
    if abs(center1[2] - center0[2]) > abs(center1[0] - center0[0]) and abs(center1[2] - center0[2]) > abs(center1[1] - center0[1]):
        # Głównie przesunięcie w Z
        if center1[2] > center0[2]:
            # segment1 jest w kierunku +Z od segment0
            joint_world = Gf.Vec3d(
                center0[0],
                center0[1],
                bbox0.GetMax()[2]  # Przód segment0
            )
        else:
            # segment1 jest w kierunku -Z od segment0
            joint_world = Gf.Vec3d(
                center0[0],
                center0[1],
                bbox0.GetMin()[2]  # Tył segment0
            )
    # Sprawdź czy segmenty są blisko siebie wzdłuż osi X
    elif abs(center1[0] - center0[0]) > abs(center1[1] - center0[1]) and abs(center1[0] - center0[0]) > abs(center1[2] - center0[2]):
        # Głównie przesunięcie w X
        if center1[0] > center0[0]:
            # segment1 jest w kierunku +X od segment0
            joint_world = Gf.Vec3d(
                bbox0.GetMax()[0],  # Prawa strona segment0
                center0[1],
                center0[2]
            )
        else:
            # segment1 jest w kierunku -X od segment0
            joint_world = Gf.Vec3d(
                bbox0.GetMin()[0],  # Lewa strona segment0
                center0[1],
                center0[2]
            )
    else:
        # Głównie przesunięcie w Y lub skomplikowane ułożenie
        # Użyj środka między środkami jako fallback
        joint_world = (center0 + center1) * 0.5
    
    return joint_world

# ===================== START =====================

ensure_path_exists(JOINT_ROOT)

created = 0
skipped = 0

for i in range(len(SEQUENCE)):
    n0 = SEQUENCE[i]
    n1 = SEQUENCE[(i + 1) % len(SEQUENCE)]

    path0 = f"{TRACK_ROOT}/{PREFIX}{n0}"
    path1 = f"{TRACK_ROOT}/{PREFIX}{n1}"

    prim0 = stage.GetPrimAtPath(path0)
    prim1 = stage.GetPrimAtPath(path1)

    if not prim0.IsValid() or not prim1.IsValid():
        skipped += 1
        print(f"⚠️ Pomijam {path0} lub {path1}")
        continue

    # --- World positions i transformacje
    xf0 = UsdGeom.Xformable(prim0).ComputeLocalToWorldTransform(0)
    xf1 = UsdGeom.Xformable(prim1).ComputeLocalToWorldTransform(0)
    
    # --- Znajdź optymalną pozycję jointa na styku segmentów
    joint_world = get_joint_position_between_segments(prim0, prim1)
    
    # --- Sprawdź odległość między segmentami
    center0 = get_bounding_box_center(prim0)
    center1 = get_bounding_box_center(prim1)
    distance = (center1 - center0).GetLength()
    
    print(f"Segment {n0} -> {n1}: odległość = {distance:.3f}")
    
    # --- Create joint
    joint_path = f"{JOINT_ROOT}/joint_{n0}_to_{n1}"
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=joint_path,
        prim_type="PhysicsRevoluteJoint"
    )

    joint_prim = stage.GetPrimAtPath(joint_path)
    joint = UsdPhysics.RevoluteJoint(joint_prim)

    # --- Oblicz lokalne pozycje jointa
    # Transformuj pozycję jointa w świecie do lokalnej przestrzeni każdego segmentu
    local0 = xf0.GetInverse().Transform(joint_world)
    local1 = xf1.GetInverse().Transform(joint_world)

    # --- Podłącz body
    joint.CreateBody0Rel().SetTargets([path0])
    joint.CreateBody1Rel().SetTargets([path1])

    # --- Ustaw lokalne pozycje
    joint.CreateLocalPos0Attr().Set(local0)
    joint.CreateLocalPos1Attr().Set(local1)

    # --- Ustaw orientację jointa (ważne dla revolute joint)
    # Zakładamy, że joint ma obracać się wokół osi X
    joint.CreateAxisAttr().Set("X")
    
    # --- Ustaw ograniczenia (dopasuj do swoich potrzeb)
    joint.CreateLowerLimitAttr().Set(-30.0)
    joint.CreateUpperLimitAttr().Set(30.0)
    
    # --- Ustaw sztywność i tłumienie (aby track był stabilny)
    if hasattr(joint, 'CreateStiffnessAttr'):
        joint.CreateStiffnessAttr().Set(1000000.0)  # Wysoka sztywność
    if hasattr(joint, 'CreateDampingAttr'):
        joint.CreateDampingAttr().Set(1000.0)  # Tłumienie

    created += 1
    if created % 20 == 0:
        print(f"⏳ Utworzono {created}/{len(SEQUENCE)} jointów")

print("\n✅ ZAKOŃCZONO")
print(f"✔️ Utworzono: {created}")
print(f"⚠️ Pominięto: {skipped}")

# ===================== DODATKOWE USTAWIENIA =====================
print("\n🔧 Ustawiam dodatkowe właściwości fizyczne...")

# Upewnij się, że wszystkie segmenty mają rigid body
for n in SEQUENCE:
    path = f"{TRACK_ROOT}/{PREFIX}{n}"
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        # Znajdź mesh pod tym primem
        mesh = find_mesh_prim(path)
        if mesh:
            ensure_rigid_body(mesh)
            
            # Ustaw masę (dopasuj do potrzeb)
            mass_api = UsdPhysics.MassAPI.Apply(mesh)
            mass_api.CreateMassAttr().Set(10.0)  # Masa 10kg
            
            # Ustaw moment bezwładności (ważne dla stabilności)
            mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))

print("✅ Gotowe! Uruchom symulację.")
