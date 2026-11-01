"""Kiểm tra môi trường mô phỏng UAV: mỗi công cụ chạy một phép thử nhỏ có đáp án biết trước.

Chạy: /opt/uav-venv/bin/python scripts/uav/check_env.py
Mã thoát 0 nếu tất cả đạt, 1 nếu có mục lỗi.
"""
import importlib
import os
import shutil
import subprocess
import sys
import tempfile

results = []


def check(name, fn):
    try:
        info = fn()
        results.append((name, True, info or ""))
    except Exception as e:  # noqa: BLE001 - báo mọi lỗi
        results.append((name, False, f"{type(e).__name__}: {e}"[:160]))


def ver(mod):
    m = importlib.import_module(mod)
    return getattr(m, "__version__", "ok")


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=120, **kw)


# --- Gói Python: import + phiên bản ---
for mod in ["numpy", "scipy", "sympy", "pandas", "matplotlib", "numba", "control", "casadi",
            "cvxpy", "openmdao", "filterpy", "aerosandbox", "neuralfoil", "ambiance", "mujoco",
            "pybullet", "gymnasium", "stable_baselines3", "torch", "gym_pybullet_drones",
            "pymavlink", "mavsdk", "pyulog", "gmsh", "meshio", "skfem", "Pynite",
            "sectionproperties", "pyvista", "genesis", "do_mpc", "acados_template"]:
    check(f"import {mod}", lambda mod=mod: ver(mod))


# --- Phép thử có đáp án ---
def t_isa():
    # ISA: tầng đối lưu kết thúc ở độ cao ĐỊA THẾ 11 km, T = 216.65 K.
    # ambiance nhận độ cao HÌNH HỌC h; đổi từ địa thế: h = r0*H/(r0 - H)
    from ambiance import Atmosphere
    r0, H = 6356766.0, 11000.0
    h = r0 * H / (r0 - H)
    T = float(Atmosphere(h).temperature[0])
    assert abs(T - 216.65) < 0.01, T
    return f"T(H=11 km địa thế, h={h:.0f} m) = {T:.2f} K"


def t_neuralfoil():
    # NACA 0012, α = 0°: CL ≈ 0 do đối xứng
    import neuralfoil as nf
    r = nf.get_aero_from_kulfan_parameters(
        __import__("aerosandbox").Airfoil("naca0012").to_kulfan_airfoil().kulfan_parameters,
        alpha=0, Re=1e6)
    cl = float(__import__("numpy").ravel(r["CL"])[0])
    assert abs(cl) < 0.01, cl
    return f"NACA0012 α=0°: CL = {cl:.4f}"


def t_vlm():
    # Cánh phẳng AR=8: CLα theo VLM phải gần công thức Helmbold (~4.9 /rad)
    import aerosandbox as asb
    import numpy as np
    wing = asb.Wing(symmetric=True, xsecs=[
        asb.WingXSec(xyz_le=[0, 0, 0], chord=1, airfoil=asb.Airfoil("naca0012")),
        asb.WingXSec(xyz_le=[0, 4, 0], chord=1, airfoil=asb.Airfoil("naca0012"))])
    ap = asb.Airplane(wings=[wing], s_ref=8, c_ref=1, b_ref=8)
    cl = [asb.VortexLatticeMethod(ap, asb.OperatingPoint(velocity=20, alpha=a),
                                  spanwise_resolution=8, chordwise_resolution=4).run()["CL"]
          for a in (0, 2)]
    cla = (cl[1] - cl[0]) / np.radians(2)
    A = 8
    helm = 2 * np.pi * A / (2 + np.sqrt(A**2 + 4))
    assert abs(cla - helm) / helm < 0.1, (cla, helm)
    return f"CLα VLM = {cla:.3f}/rad, Helmbold = {helm:.3f}/rad"


def t_lqr():
    # LQR cho tích phân kép: hệ kín phải ổn định
    import control
    import numpy as np
    K, _, E = control.lqr(np.array([[0, 1], [0, 0]]), np.array([[0], [1]]), np.eye(2), 1)
    assert np.all(np.real(E) < 0)
    return f"K = {np.round(K, 3).tolist()}"


def t_mujoco():
    # Rơi tự do 1 s: z = -g t²/2 ≈ -4.905 m
    import mujoco
    m = mujoco.MjModel.from_xml_string(
        '<mujoco><option timestep="0.001"/><worldbody><body><freejoint/>'
        '<geom size=".1"/></body></worldbody></mujoco>')
    d = mujoco.MjData(m)
    for _ in range(1000):
        mujoco.mj_step(m, d)
    z = d.qpos[2]
    assert abs(z + 4.905) < 0.02, z
    return f"z(1 s) = {z:.3f} m"


def t_pybullet_drone():
    from gym_pybullet_drones.envs.HoverAviary import HoverAviary
    env = HoverAviary(gui=False)
    env.reset()
    env.step(env.action_space.sample())
    env.close()
    return "HoverAviary chạy được"


def t_cantilever():
    # Dầm công-xôn: δ = P L³ / (3 E I)
    from Pynite import FEModel3D
    E, G, Iy, Iz, J, A, L, P = 70e3, 26e3, 1e4, 1e4, 2e4, 100.0, 1000.0, 100.0  # N, mm, MPa
    m = FEModel3D()
    m.add_material("Al", E, G, 0.33, 2.7e-9)
    m.add_section("S", A, Iy, Iz, J)
    m.add_node("A", 0, 0, 0)
    m.add_node("B", L, 0, 0)
    m.add_member("M", "A", "B", "Al", "S")
    m.def_support("A", True, True, True, True, True, True)
    m.add_node_load("B", "FY", -P)
    m.analyze(check_statics=False)
    d = abs(m.nodes["B"].DY["Combo 1"])
    exact = P * L**3 / (3 * E * Iz)
    assert abs(d - exact) / exact < 1e-3, (d, exact)
    return f"δ = {d:.4f} mm (giải tích {exact:.4f} mm)"


def t_section():
    # Chữ nhật 20×40 mm: Ixx = b h³/12 = 106667 mm⁴
    from sectionproperties.analysis import Section
    from sectionproperties.pre.library import rectangular_section
    g = rectangular_section(d=40, b=20)
    g.create_mesh(mesh_sizes=[10])
    s = Section(g)
    s.calculate_geometric_properties()
    ixx = s.get_ic()[0]
    assert abs(ixx - 20 * 40**3 / 12) / (20 * 40**3 / 12) < 1e-3, ixx
    return f"Ixx = {ixx:.0f} mm⁴"


def t_gmsh():
    import gmsh
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.model.occ.addBox(0, 0, 0, 1, 1, 1)
    gmsh.model.occ.synchronize()
    gmsh.model.mesh.generate(3)
    n = len(gmsh.model.mesh.getNodes()[0])
    gmsh.finalize()
    assert n > 8
    return f"lưới khối hộp: {n} nút"


def t_ccx():
    # CalculiX: thanh kéo 1 phần tử C3D8, ứng suất = F/A
    exe = shutil.which("ccx")
    assert exe, "không có ccx"
    inp = """*NODE
1,0,0,0
2,1,0,0
3,1,1,0
4,0,1,0
5,0,0,1
6,1,0,1
7,1,1,1
8,0,1,1
*ELEMENT,TYPE=C3D8,ELSET=E
1,1,2,3,4,5,6,7,8
*MATERIAL,NAME=AL
*ELASTIC
70000,0.33
*SOLID SECTION,ELSET=E,MATERIAL=AL
*BOUNDARY
1,1,3
4,1,1
4,3,3
5,1,2
8,1,1
*STEP
*STATIC
*CLOAD
2,1,25
3,1,25
6,1,25
7,1,25
*EL PRINT,ELSET=E
S
*END STEP
"""
    with tempfile.TemporaryDirectory() as d:
        open(os.path.join(d, "bar.inp"), "w").write(inp)
        r = run([exe, "bar"], cwd=d)
        assert r.returncode == 0, r.stdout[-300:]
        lines = [ln for ln in open(os.path.join(d, "bar.dat")) if ln.strip()[:1].isdigit()]
        sxx = float(lines[0].split()[2])
    assert abs(sxx - 100.0) < 0.5, sxx
    return f"σxx = {sxx:.2f} MPa (đúng F/A = 100 MPa)"


def t_bin(name, args):
    def f():
        exe = shutil.which(name)
        assert exe, f"không có {name} trong PATH"
        r = run([exe, *args])
        out = (r.stdout + r.stderr).strip().splitlines()
        return out[0][:80] if out else exe
    return f


def t_xfoil():
    exe = shutil.which("xfoil")
    assert exe
    # XFOIL ghi file lớp biên (:00.bl) vào thư mục hiện tại -> chạy trong thư mục tạm
    with tempfile.TemporaryDirectory() as d:
        r = subprocess.run([exe], input="naca 2412\noper\nvisc 1e6\nalfa 4\n\nquit\n",
                           capture_output=True, text=True, timeout=60, cwd=d,
                           env={**os.environ, "DISPLAY": ""})
    cl = [ln for ln in r.stdout.splitlines() if "CL =" in ln]
    assert cl, r.stdout[-300:]
    return cl[-1].strip()[:80]


def t_openfoam():
    rc = [p for p in ["/usr/share/openfoam/etc/bashrc"] if os.path.exists(p)]
    rc += [os.path.join("/usr/lib/openfoam", d, "etc/bashrc")
           for d in (os.listdir("/usr/lib/openfoam") if os.path.isdir("/usr/lib/openfoam") else [])]
    assert rc, "không tìm thấy bashrc OpenFOAM"
    r = run(["bash", "-c", f"source {rc[0]} >/dev/null 2>&1; which simpleFoam && echo $WM_PROJECT_VERSION"])
    assert r.returncode == 0 and "simpleFoam" in r.stdout, r.stdout + r.stderr
    return "simpleFoam OK, phiên bản " + r.stdout.split()[-1]


def t_ardupilot():
    # Khởi động SITL quad, chờ heartbeat MAVLink: type 2 = quadrotor, autopilot 3 = ArduPilot
    import time
    from pymavlink import mavutil
    exe = "/opt/ardupilot/build/sitl/bin/arducopter"
    assert os.path.exists(exe), "chưa build ArduPilot SITL"
    with tempfile.TemporaryDirectory() as d:
        p = subprocess.Popen([exe, "--model", "quad", "-I0", "--defaults",
                              "/opt/ardupilot/Tools/autotest/default_params/copter.parm"],
                             cwd=d, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            time.sleep(3)
            hb = mavutil.mavlink_connection("tcp:127.0.0.1:5760").wait_heartbeat(timeout=60)
            assert hb and hb.type == 2 and hb.autopilot == 3, hb
        finally:
            p.terminate()
            p.wait(timeout=10)
    return "arducopter SITL gửi heartbeat (quadrotor, ArduPilot)"


def t_genesis():
    # Rơi tự do 1 s từ 10 m, dt = 1 ms: z ≈ 10 - 4.905 (Euler bán ẩn lệch thêm ~g·dt·n/2 = 4.9 mm)
    import genesis as gs
    gs.init(backend=gs.cpu, logging_level="warning")
    scene = gs.Scene(show_viewer=False, sim_options=gs.options.SimOptions(dt=0.001))
    scene.add_entity(gs.morphs.Plane())
    box = scene.add_entity(gs.morphs.Box(size=(0.1, 0.1, 0.1), pos=(0, 0, 10)))
    scene.build()
    for _ in range(1000):
        scene.step()
    z = float(box.get_pos()[2])
    assert abs(z - 5.095) < 0.02, z
    return f"z(1 s) = {z:.3f} m"


def t_gazebo():
    # Gazebo Harmonic headless: chạy 1000 bước world mẫu
    exe = shutil.which("gz")
    assert exe, "không có lệnh gz"
    r = run([exe, "sim", "-s", "-r", "--iterations", "1000", "-v", "1", "empty.sdf"])
    assert r.returncode == 0, (r.stdout + r.stderr)[-300:]
    v = run([exe, "sim", "--versions"]).stdout.split()
    return f"gz-sim {v[0] if v else '?'} chạy 1000 bước headless"


def t_px4():
    # PX4 SITL (SIH quad): arm + cất cánh tới MIS_TAKEOFF_ALT qua MAVLink
    here = os.path.dirname(os.path.abspath(__file__))
    p = subprocess.Popen([os.path.join(here, "px4_sitl.sh"), "sihsim_quadx"],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        r = subprocess.run([sys.executable, os.path.join(here, "examples", "px4_takeoff.py")],
                           capture_output=True, text=True, timeout=240)
    finally:
        p.terminate()
        p.wait(timeout=20)
    out = r.stdout.strip().splitlines()
    assert r.returncode == 0, (r.stdout + r.stderr)[-300:]
    return out[-1] if out else "ok"


def t_openfoam2412():
    # Tutorial cavity (icoFoam) phải chạy tới t = 0.5 s
    assert shutil.which("of2412"), "không có lệnh of2412"
    src = "/opt/openfoam2412/tutorials/incompressible/icoFoam/cavity/cavity"
    with tempfile.TemporaryDirectory() as d:
        case = shutil.copytree(src, os.path.join(d, "cavity"))
        for app in ("blockMesh", "icoFoam"):
            r = run(["of2412", app], cwd=case)
            assert r.returncode == 0, f"{app}: {(r.stdout + r.stderr)[-300:]}"
        assert os.path.isdir(os.path.join(case, "0.5")), "không có kết quả t = 0.5"
    return "icoFoam cavity chạy tới t = 0.5 s"


def t_aster():
    # Dầm công-xôn POU_D_E, cùng số liệu với PyNite: δ = P L³ / (3 E I) = 47.619 mm
    assert shutil.which("run_aster"), "không có lệnh run_aster"
    mail = """COOR_3D
N1 0. 0. 0.
N2 250. 0. 0.
N3 500. 0. 0.
N4 750. 0. 0.
N5 1000. 0. 0.
FINSF
SEG2
E1 N1 N2
E2 N2 N3
E3 N3 N4
E4 N4 N5
FINSF
GROUP_NO
FIX N1
FINSF
GROUP_NO
TIP N5
FINSF
GROUP_MA
BEAM E1 E2 E3 E4
FINSF
FIN
"""
    comm = """from code_aster import CA
from code_aster.Commands import *
CA.init()
DEFI_FICHIER(ACTION="ASSOCIER", FICHIER="{mail}", UNITE=20, ACCES="OLD")
mesh = LIRE_MAILLAGE(FORMAT="ASTER", UNITE=20)
model = AFFE_MODELE(MAILLAGE=mesh, AFFE=_F(GROUP_MA="BEAM", PHENOMENE="MECANIQUE", MODELISATION="POU_D_E"))
mat = DEFI_MATERIAU(ELAS=_F(E=70000.0, NU=0.33))
cm = AFFE_MATERIAU(MAILLAGE=mesh, AFFE=_F(TOUT="OUI", MATER=mat))
cara = AFFE_CARA_ELEM(MODELE=model, POUTRE=_F(GROUP_MA="BEAM", SECTION="GENERALE",
    CARA=("A", "IY", "IZ", "JX"), VALE=(100.0, 1.0e4, 1.0e4, 2.0e4)))
bc = AFFE_CHAR_MECA(MODELE=model, DDL_IMPO=_F(GROUP_NO="FIX", DX=0, DY=0, DZ=0, DRX=0, DRY=0, DRZ=0),
    FORCE_NODALE=_F(GROUP_NO="TIP", FY=-100.0))
res = MECA_STATIQUE(MODELE=model, CHAM_MATER=cm, CARA_ELEM=cara, EXCIT=_F(CHARGE=bc))
tab = POST_RELEVE_T(ACTION=_F(INTITULE="tip", GROUP_NO="TIP", RESULTAT=res, NOM_CHAM="DEPL",
    NOM_CMP="DY", OPERATION="EXTRACTION"))
print("DY_TIP=%.6f" % tab.EXTR_TABLE().values()["DY"][0])
FIN()
"""
    with tempfile.TemporaryDirectory() as d:
        mail_path = os.path.join(d, "beam.mail")
        open(mail_path, "w").write(mail)
        open(os.path.join(d, "beam.py"), "w").write(comm.format(mail=mail_path))
        r = run(["run_aster", "beam.py"], cwd=d)
    lines = [ln for ln in r.stdout.splitlines() if ln.startswith("DY_TIP=")]
    assert r.returncode == 0 and lines, (r.stdout + r.stderr)[-300:]
    d_tip = abs(float(lines[0].split("=")[1]))
    exact = 100 * 1000**3 / (3 * 70e3 * 1e4)
    assert abs(d_tip - exact) / exact < 1e-3, (d_tip, exact)
    return f"δ = {d_tip:.4f} mm (giải tích {exact:.4f} mm)"


def t_fenicsx():
    # Kéo đơn trục khối lập phương cạnh 1 mm, σ = 100 MPa, E = 70 GPa: u_x(x=1) = σL/E
    assert shutil.which("fenicsx-python"), "không có lệnh fenicsx-python"
    code = """
from mpi4py import MPI
import numpy as np, ufl
from dolfinx import mesh, fem
from dolfinx.fem.petsc import LinearProblem
dom = mesh.create_unit_cube(MPI.COMM_WORLD, 4, 4, 4)
V = fem.functionspace(dom, ("Lagrange", 1, (3,)))
E, nu = 70e3, 0.33
mu, lam = E / (2 * (1 + nu)), E * nu / ((1 + nu) * (1 - 2 * nu))
fdim = dom.topology.dim - 1
bcs = []
for i in range(3):  # đối xứng: mặt x_i = 0 khoá chuyển vị theo phương i -> kéo đơn trục thuần
    facets = mesh.locate_entities_boundary(dom, fdim, lambda x, i=i: np.isclose(x[i], 0))
    dofs = fem.locate_dofs_topological(V.sub(i), fdim, facets)
    bcs.append(fem.dirichletbc(0.0, dofs, V.sub(i)))
right = mesh.locate_entities_boundary(dom, fdim, lambda x: np.isclose(x[0], 1))
tags = mesh.meshtags(dom, fdim, right, np.full(len(right), 1, dtype=np.int32))
ds = ufl.Measure("ds", domain=dom, subdomain_data=tags)
u, v = ufl.TrialFunction(V), ufl.TestFunction(V)
eps = lambda w: ufl.sym(ufl.grad(w))
sig = lambda w: 2 * mu * eps(w) + lam * ufl.tr(eps(w)) * ufl.Identity(3)
a = ufl.inner(sig(u), eps(v)) * ufl.dx
L = ufl.dot(fem.Constant(dom, (100.0, 0.0, 0.0)), v) * ds(1)
uh = LinearProblem(a, L, bcs=bcs, petsc_options_prefix="t_",
                   petsc_options={"ksp_type": "preonly", "pc_type": "lu"}).solve()
print("UX_MAX=%.8f" % uh.x.array.reshape(-1, 3)[:, 0].max())
"""
    r = run(["fenicsx-python", "-c", code])
    lines = [ln for ln in r.stdout.splitlines() if ln.startswith("UX_MAX=")]
    assert r.returncode == 0 and lines, (r.stdout + r.stderr)[-300:]
    ux = float(lines[0].split("=")[1])
    exact = 100.0 / 70e3
    # Phần tử P1 cho nghiệm chính xác khi ứng suất đều; sai số chỉ do làm tròn
    assert abs(ux - exact) / exact < 1e-5, (ux, exact)
    return f"u_x = {ux:.6e} mm (giải tích {exact:.6e} mm)"


def t_ccx_version():
    r = run([shutil.which("ccx") or "ccx", "-v"])
    out = (r.stdout + r.stderr).strip()
    assert "2.23" in out, out[-200:]
    return out.splitlines()[-1][:60]


def t_acados():
    # OCP tích phân kép: đưa x0 = (1, 0) về 0; acados phải giải được (status 0)
    import casadi as ca
    import numpy as np
    from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
    m = AcadosModel()
    m.name = "dint"
    m.x, m.u = ca.SX.sym("x", 2), ca.SX.sym("u")
    m.f_expl_expr = ca.vertcat(m.x[1], m.u)
    ocp = AcadosOcp()
    ocp.model = m
    ocp.solver_options.N_horizon, ocp.solver_options.tf = 20, 2.0
    ocp.cost.cost_type = ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.Vx, ocp.cost.Vu = np.vstack([np.eye(2), [[0, 0]]]), np.array([[0], [0], [1.0]])
    ocp.cost.Vx_e, ocp.cost.W, ocp.cost.W_e = np.eye(2), np.eye(3), 10 * np.eye(2)
    ocp.cost.yref, ocp.cost.yref_e = np.zeros(3), np.zeros(2)
    ocp.constraints.idxbu, ocp.constraints.lbu, ocp.constraints.ubu = [0], [-1.0], [1.0]
    ocp.constraints.x0 = np.array([1.0, 0.0])
    with tempfile.TemporaryDirectory() as d:
        cwd = os.getcwd()
        os.chdir(d)
        try:
            ocp.code_export_directory = os.path.join(d, "c_generated_code")
            solver = AcadosOcpSolver(ocp, json_file=os.path.join(d, "ocp.json"), verbose=False)
            status = solver.solve()
            u0 = float(solver.get(0, "u")[0])
        finally:
            os.chdir(cwd)
    assert status == 0, status
    assert -1.0 - 1e-6 <= u0 < 0, u0   # phải đẩy ngược chiều x0, trong giới hạn |u| ≤ 1
    return f"status 0, u0 = {u0:.3f}"


def t_quad_mpc():
    here = os.path.dirname(os.path.abspath(__file__))
    r = subprocess.run([sys.executable, os.path.join(here, "examples", "quad_mpc.py")],
                       capture_output=True, text=True, timeout=600)
    out = r.stdout.strip().splitlines()
    assert r.returncode == 0, (r.stdout + r.stderr)[-400:]
    return out[0] if out else "ok"


check("ISA (ambiance)", t_isa)
check("NeuralFoil", t_neuralfoil)
check("AeroSandbox VLM", t_vlm)
check("python-control LQR", t_lqr)
check("MuJoCo rơi tự do", t_mujoco)
check("gym-pybullet-drones", t_pybullet_drone)
check("PyNite dầm công-xôn", t_cantilever)
check("sectionproperties", t_section)
check("gmsh", t_gmsh)
check("CalculiX (ccx)", t_ccx)
check("XFOIL", t_xfoil)
check("OpenFOAM", t_openfoam)
check("SU2", t_bin("SU2_CFD", ["--help"]))
check("ParaView (pvbatch)", t_bin("pvbatch", ["--version"]))
check("ArduPilot SITL", t_ardupilot)
check("Genesis rơi tự do", t_genesis)
check("Gazebo Harmonic", t_gazebo)
check("PX4 SITL cất cánh (SIH)", t_px4)
check("OpenFOAM v2412", t_openfoam2412)
check("Code_Aster dầm công-xôn", t_aster)
check("FEniCSx kéo đơn trục", t_fenicsx)
check("CalculiX 2.23", t_ccx_version)
check("acados OCP", t_acados)
check("NMPC quadrotor (acados)", t_quad_mpc)

w = max(len(n) for n, _, _ in results)
for name, ok, info in results:
    print(f"{'✅' if ok else '❌'} {name:<{w}}  {info}")
bad = [n for n, ok, _ in results if not ok]
print(f"\n{len(results) - len(bad)}/{len(results)} đạt" + (f"; lỗi: {', '.join(bad)}" if bad else ""))
sys.exit(1 if bad else 0)
