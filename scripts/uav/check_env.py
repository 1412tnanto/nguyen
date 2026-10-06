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
            "sectionproperties", "pyvista"]:
    check(f"import {mod}", lambda mod=mod: ver(mod))


# --- Phép thử có đáp án ---
def t_isa():
    # Khí quyển ISA ở 11 km: T = 216.65 K
    from ambiance import Atmosphere
    T = float(Atmosphere(11000).temperature[0])
    assert abs(T - 216.65) < 0.01, T
    return f"T(11 km) = {T:.2f} K"


def t_neuralfoil():
    # NACA 0012, α = 0°: CL ≈ 0 do đối xứng
    import neuralfoil as nf
    r = nf.get_aero_from_kulfan_parameters(
        __import__("aerosandbox").Airfoil("naca0012").to_kulfan_airfoil().kulfan_parameters,
        alpha=0, Re=1e6)
    cl = float(r["CL"])
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
    r = subprocess.run([exe], input="naca 2412\noper\nvisc 1e6\nalfa 4\n\nquit\n",
                       capture_output=True, text=True, timeout=60,
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
    exe = "/opt/ardupilot/build/sitl/bin/arducopter"
    assert os.path.exists(exe), "chưa build ArduPilot SITL"
    return "arducopter, arduplane có sẵn"


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
check("ParaView (pvpython)", t_bin("pvpython", ["--version"]))
check("ArduPilot SITL", t_ardupilot)

w = max(len(n) for n, _, _ in results)
for name, ok, info in results:
    print(f"{'✅' if ok else '❌'} {name:<{w}}  {info}")
bad = [n for n, ok, _ in results if not ok]
print(f"\n{len(results) - len(bad)}/{len(results)} đạt" + (f"; lỗi: {', '.join(bad)}" if bad else ""))
sys.exit(1 if bad else 0)
