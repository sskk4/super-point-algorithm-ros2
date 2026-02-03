from pxr import Usd, UsdPhysics, UsdGeom, Gf, Sdf
import omni.usd
import omni.kit.commands

# =====================================================
# KONFIGURACJA
# =====================================================

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

# =====================================================
# FUNKCJE POMOCNICZE
# =====================================================

def ensure_xform(path):
    if not stage.GetPrimAtPath(path).IsValid():
        omni.kit.commands.execute(
            "CreatePrim",
            prim_path=path,
            prim_type="Xform"
        )

def get_pivot(prim):
    """
    Pobiera wartość xformOp:translate:pivot.
    Zwraca zawsze Gf.Vec3f, aby uniknąć błędów C++.
    """
    # Sprawdzamy czy atrybut istnieje bezpośrednio
    attr = prim.GetAttribute("xformOp:translate:pivot")
    if attr.IsValid():
        val = attr.Get()
        if val is not None:
            # Konwersja na Vec3f dla pewności
            return Gf.Vec3f(val[0], val[1], val[2])
    
    # Jeśli brak pivota, zwracamy wektor zerowy
    return Gf.Vec3f(0.0, 0.0, 0.0)

def apply_pivot_to_mesh(xform_prim, pivot_val):
    """
    Znajduje dziecko typu Mesh i ustawia mu pivot używając AddPivotOp.
    """
    for child in xform_prim.GetChildren():
        if child.IsA(UsdGeom.Mesh):
            # Rzutujemy na Xformable (to pozwala edytować transformacje)
            xformable = UsdGeom.Xformable(child)
            
            # Dodajemy operację Pivot (lub pobieramy istniejącą)
            pivot_op = xformable.AddPivotOp()
            
            # Ustawiamy wartość (musi być Gf.Vec3f/d)
            if pivot_op:
                pivot_op.Set(pivot_val)

def ensure_rigid_body(xform_prim):
    if not xform_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(xform_prim)

    # Ustawienie masy
    mass_api = UsdPhysics.MassAPI.Apply(xform_prim)
    # Sprawdzamy czy atrybut ma wartość, jeśli nie - ustawiamy
    if not mass_api.GetMassAttr().Get():
        mass_api.CreateMassAttr(1.0) # Zwiększyłem masę dla stabilności gąsienic

def ensure_collision(xform_prim):
    """Dodaje CollisionAPI do mesha wewnątrz Xforma."""
    for c in xform_prim.GetChildren():
        if c.IsA(UsdGeom.Mesh):
            if not c.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(c)

# =====================================================
# START SKRYPTU
# =====================================================

ensure_xform(JOINT_ROOT)

created = 0
skipped = 0

print("Rozpoczynam tworzenie gąsienicy...")

# Iterujemy przez sekwencję
for i in range(len(SEQUENCE)):
    # Pobieramy ID obecnego i następnego elementu
    idx_a = SEQUENCE[i]
    idx_b = SEQUENCE[(i + 1) % len(SEQUENCE)] # Pętla zamknięta

    path_a = f"{TRACK_ROOT}/{PREFIX}{idx_a}"
    path_b = f"{TRACK_ROOT}/{PREFIX}{idx_b}"

    prim_a = stage.GetPrimAtPath(path_a)
    prim_b = stage.GetPrimAtPath(path_b)

    if not prim_a.IsValid() or not prim_b.IsValid():
        print(f"⚠️ Nie znaleziono: {path_a} lub {path_b}")
        skipped += 1
        continue

    # 1. POBIERZ PIVOT z XFORM (zawsze Gf.Vec3f)
    pivot_a = get_pivot(prim_a)
    pivot_b = get_pivot(prim_b)

    # 2. DODAJ RIGIDBODY (XFORM)
    ensure_rigid_body(prim_a)
    ensure_rigid_body(prim_b)

    # 3. DODAJ COLLISION (MESH)
    ensure_collision(prim_a)
    ensure_collision(prim_b)

    # 4. USTAW TEN SAM PIVOT DLA MESHA
    # Naprawiona funkcja - teraz nie wywali błędu C++ signature
    apply_pivot_to_mesh(prim_a, pivot_a)
    apply_pivot_to_mesh(prim_b, pivot_b)

    # 5. STWÓRZ REVOLUTE JOINT
    joint_path = f"{JOINT_ROOT}/joint_{idx_a}_to_{idx_b}"
    
    # Usuwamy stary joint jeśli istnieje
    if stage.GetPrimAtPath(joint_path).IsValid():
        stage.RemovePrim(joint_path)

    omni.kit.commands.execute(
        "CreatePrim",
        prim_path=joint_path,
        prim_type="PhysicsRevoluteJoint"
    )

    joint_prim = stage.GetPrimAtPath(joint_path)
    joint = UsdPhysics.RevoluteJoint(joint_prim)

    # Relacje Body0 i Body1
    joint.CreateBody0Rel().SetTargets([prim_a.GetPath()])
    joint.CreateBody1Rel().SetTargets([prim_b.GetPath()])

    # 6. USTAW LOCAL POSITION NA WARTOŚĆ PIVOTA
    # Ponieważ pivot_a/b są typu Gf.Vec3f, C++ to zaakceptuje
    joint.CreateLocalPos0Attr().Set(pivot_a)
    joint.CreateLocalPos1Attr().Set(pivot_b)

    # Domyślna oś (X)
    joint.CreateAxisAttr().Set("X")

    # Wyłącz kolizje między połączonymi ogniwami
    joint.CreateCollisionEnabledAttr(False)

    created += 1

print("\n" + "="*30)
print(f"✅ ZAKOŃCZONO SUKCESEM")
print(f"🔗 Utworzono połączeń: {created}")
print(f"⚠️ Pominięto: {skipped}")
print("="*30)
