from pxr import UsdPhysics, UsdGeom, Gf
import omni.usd
import omni.kit.commands

stage = omni.usd.get_context().get_stage()
TRACK_ROOT = "/World/Robot/Track"
JOINT_ROOT = f"{TRACK_ROOT}/Joints"
PREFIX = "DEFAULT_"

# --- DOKŁADNA KOLEJNOŚĆ ŁĄCZENIA ---
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

# --- 1️⃣ Upewnij się że folder Joints istnieje ---
def ensure_path_exists(path):
    parts = path.strip('/').split('/')
    current_path = ""
    for part in parts:
        current_path += f"/{part}"
        if not stage.GetPrimAtPath(current_path).IsValid():
            print(f"📁 Tworzę: {current_path}")
            omni.kit.commands.execute(
                "CreatePrim",
                prim_path=current_path,
                prim_type="Xform"
            )

def get_world_position(prim_path):
    """Pobiera pozycję świata (world space) danego prima"""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    
    xformable = UsdGeom.Xformable(prim)
    world_transform = xformable.ComputeLocalToWorldTransform(0)
    return world_transform.ExtractTranslation()

def get_midpoint(pos1, pos2):
    """Oblicza punkt środkowy między dwoma pozycjami"""
    return Gf.Vec3d(
        (pos1[0] + pos2[0]) / 2.0,
        (pos1[1] + pos2[1]) / 2.0,
        (pos1[2] + pos2[2]) / 2.0
    )

ensure_path_exists(JOINT_ROOT)

# --- 2️⃣ Tworzenie jointów według kolejności ---
joints_created = 0
joints_skipped = 0

for i in range(len(SEQUENCE)):
    num1 = SEQUENCE[i]
    num2 = SEQUENCE[(i + 1) % len(SEQUENCE)]
    
    path1 = f"{TRACK_ROOT}/{PREFIX}{num1}"
    path2 = f"{TRACK_ROOT}/{PREFIX}{num2}"
    
    # Sprawdź czy oba segmenty istnieją
    if not stage.GetPrimAtPath(path1).IsValid():
        print(f"⚠️ Pomijam: {PREFIX}{num1} nie istnieje")
        joints_skipped += 1
        continue
    
    if not stage.GetPrimAtPath(path2).IsValid():
        print(f"⚠️ Pomijam: {PREFIX}{num2} nie istnieje")
        joints_skipped += 1
        continue
    
    # Pobierz pozycje segmentów
    pos1 = get_world_position(path1)
    pos2 = get_world_position(path2)
    
    if pos1 is None or pos2 is None:
        print(f"⚠️ Nie mogę pobrać pozycji dla {num1} lub {num2}")
        joints_skipped += 1
        continue
    
    # Oblicz punkt środkowy między segmentami
    joint_position = get_midpoint(pos1, pos2)
    
    # Nazwa jointa
    joint_name = f"joint_{num1}_to_{num2}"
    joint_path = f"{JOINT_ROOT}/{joint_name}"
    
    # Tworzenie jointa
    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=joint_path,
        prim_type="PhysicsRevoluteJoint"
    )
    
    joint_prim = stage.GetPrimAtPath(joint_path)
    joint = UsdPhysics.RevoluteJoint(joint_prim)
    
    # Ustaw pozycję jointa
    xform = UsdGeom.Xformable(joint_prim)
    xform.ClearXformOpOrder()
    translate_op = xform.AddTranslateOp()
    translate_op.Set(joint_position)
    
    # Ustaw parametry jointa
    joint.CreateAxisAttr("X")
    joint.CreateBody0Rel().SetTargets([path1])
    joint.CreateBody1Rel().SetTargets([path2])
    joint.CreateLowerLimitAttr(-30.0)
    joint.CreateUpperLimitAttr(30.0)
    
    joints_created += 1
    
    # Co 20 jointów wypisz postęp
    if joints_created % 20 == 0:
        print(f"⏳ Utworzono {joints_created}/{len(SEQUENCE)} jointów...")

print(f"\n✅ ZAKOŃCZONO:")
print(f"   ✔️ Utworzono: {joints_created} jointów")
if joints_skipped > 0:
    print(f"   ⚠️ Pominięto: {joints_skipped} połączeń (brak segmentów)")
