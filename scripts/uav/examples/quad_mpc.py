"""NMPC bám quỹ đạo tròn cho quadrotor bằng acados (SQP-RTI).

Mô hình 10 trạng thái: vị trí p, vận tốc v (hệ quán tính, trục z hướng lên), quaternion q = (w, x, y, z).
Đầu vào: lực đẩy tổng T [N] và vận tốc góc thân ω [rad/s] (giả sử vòng trong bám ω lý tưởng).
    ṗ = v,   v̇ = R(q)·[0, 0, T/m] − [0, 0, g],   q̇ = ½ q ⊗ [0, ω]
Chạy: /opt/uav-venv/bin/python scripts/uav/examples/quad_mpc.py
Tự kiểm (mã thoát ≠ 0 nếu sai): RMS sai số vị trí, ràng buộc đầu vào, thời gian giải.
"""
import os
import sys
import tempfile

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver

# --- Thông số (SI) ---
M, G = 1.0, 9.81                     # khối lượng [kg], gia tốc trọng trường [m/s²]
T_MAX, W_MAX = 2 * M * G, 3.0        # lực đẩy tối đa [N], vận tốc góc tối đa [rad/s]
N, TF = 20, 1.0                      # số bước dự báo, chân trời [s]
DT, T_SIM = TF / N, 10.0             # chu kỳ điều khiển 50 ms, thời gian mô phỏng [s]
R_C, Z_C, W_C = 2.0, 1.0, 2 * np.pi / 8   # quỹ đạo tròn: bán kính [m], độ cao [m], tốc độ góc [rad/s]


def quad_model():
    p, v, q = ca.SX.sym("p", 3), ca.SX.sym("v", 3), ca.SX.sym("q", 4)
    T, w = ca.SX.sym("T"), ca.SX.sym("w", 3)
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    # Cột thứ 3 của ma trận quay R(q): hướng lực đẩy trong hệ quán tính
    z_b = ca.vertcat(2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx), 1 - 2 * (qx**2 + qy**2))
    q_dot = 0.5 * ca.vertcat(
        -qx * w[0] - qy * w[1] - qz * w[2],
        qw * w[0] + qy * w[2] - qz * w[1],
        qw * w[1] - qx * w[2] + qz * w[0],
        qw * w[2] + qx * w[1] - qy * w[0])
    x, u = ca.vertcat(p, v, q), ca.vertcat(T, w)
    m = AcadosModel()
    m.name = "quad"
    m.x, m.u = x, u
    m.f_expl_expr = ca.vertcat(v, z_b * T / M - ca.vertcat(0, 0, G), q_dot)
    m.xdot = ca.SX.sym("xdot", 10)
    m.f_impl_expr = m.xdot - m.f_expl_expr
    return m


def reference(t):
    """Trạng thái + đầu vào tham chiếu trên quỹ đạo tròn (q, ω gần đúng ở hover)."""
    p = [R_C * np.cos(W_C * t), R_C * np.sin(W_C * t), Z_C]
    v = [-R_C * W_C * np.sin(W_C * t), R_C * W_C * np.cos(W_C * t), 0.0]
    return np.array(p + v + [1, 0, 0, 0, M * G, 0, 0, 0])


def build(workdir):
    ocp = AcadosOcp()
    ocp.model = quad_model()
    x, u = ocp.model.x, ocp.model.u
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = TF
    # Chi phí bình phương tối thiểu phi tuyến: y = [x; u]
    ocp.cost.cost_type = ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.model.cost_y_expr, ocp.model.cost_y_expr_e = ca.vertcat(x, u), x
    w_x = [200] * 3 + [10] * 3 + [5] * 4   # p, v, q
    w_u = [0.5] + [1] * 3                   # T, ω
    ocp.cost.W = np.diag(w_x + w_u)
    ocp.cost.W_e = np.diag(w_x) * 5
    ocp.cost.yref, ocp.cost.yref_e = reference(0), reference(0)[:10]
    # Ràng buộc đầu vào
    ocp.constraints.idxbu = np.arange(4)
    ocp.constraints.lbu = np.array([0, -W_MAX, -W_MAX, -W_MAX])
    ocp.constraints.ubu = np.array([T_MAX, W_MAX, W_MAX, W_MAX])
    ocp.constraints.x0 = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0], dtype=float)
    so = ocp.solver_options
    so.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    so.hessian_approx = "GAUSS_NEWTON"
    so.integrator_type = "ERK"
    so.nlp_solver_type = "SQP_RTI"
    ocp.code_export_directory = os.path.join(workdir, "c_generated_code")
    solver = AcadosOcpSolver(ocp, json_file=os.path.join(workdir, "quad_ocp.json"), verbose=False)
    sim = AcadosSim.from_ocp(ocp)
    sim.code_export_directory = os.path.join(workdir, "c_generated_sim")
    return solver, AcadosSimSolver(sim, json_file=os.path.join(workdir, "quad_sim.json"), verbose=False)


def main():
    workdir = tempfile.mkdtemp(prefix="quad_mpc_")
    cwd = os.getcwd()
    os.chdir(workdir)   # acados sinh file build vào thư mục hiện tại -> giữ repo sạch
    try:
        solver, sim = build(workdir)
    finally:
        os.chdir(cwd)

    steps = int(T_SIM / DT)
    x = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 0], dtype=float)
    X, U, E, t_solve = [x.copy()], [], [], []
    for k in range(steps):
        t = k * DT
        for j in range(N):
            solver.set(j, "yref", reference(t + j * DT))
        solver.set(N, "yref", reference(t + N * DT)[:10])
        solver.set(0, "lbx", x)
        solver.set(0, "ubx", x)
        status = solver.solve()
        # status 2 = đạt số vòng lặp tối đa — bình thường với SQP-RTI (1 vòng/bước)
        assert status in (0, 2), f"acados lỗi status={status} tại t={t:.2f} s"
        t_solve.append(solver.get_stats("time_tot"))
        u = solver.get(0, "u")
        sim.set("x", x)
        sim.set("u", u)
        sim.solve()
        x = sim.get("x")
        x[6:] /= np.linalg.norm(x[6:])   # giữ quaternion đơn vị
        X.append(x.copy())
        U.append(u)
        E.append(np.linalg.norm(x[:3] - reference(t + DT)[:3]))

    X, U, E = np.array(X), np.array(U), np.array(E)
    t_axis = np.arange(steps) * DT + DT
    settle = t_axis > 3.0                 # bỏ giai đoạn quá độ (xuất phát từ gốc toạ độ)
    rms = float(np.sqrt(np.mean(E[settle] ** 2)))
    t_ms = 1000 * float(np.mean(t_solve))
    print(f"RMS sai số vị trí (t > 3 s): {rms * 100:.2f} cm")
    print(f"Sai số lớn nhất (t > 3 s):  {E[settle].max() * 100:.2f} cm")
    print(f"T ∈ [{U[:, 0].min():.2f}, {U[:, 0].max():.2f}] N  (giới hạn 0 … {T_MAX:.2f} N)")
    print(f"|ω|max = {np.abs(U[:, 1:]).max():.2f} rad/s  (giới hạn {W_MAX} rad/s)")
    print(f"Thời gian giải trung bình: {t_ms:.3f} ms/bước (chu kỳ {DT * 1000:.0f} ms)")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        ref = np.array([reference(t)[:3] for t in np.r_[0, t_axis]])
        fig = plt.figure(figsize=(10, 4))
        ax = fig.add_subplot(121, projection="3d")
        ax.plot(*ref.T, "k--", label="tham chiếu")
        ax.plot(*X[:, :3].T, label="NMPC")
        ax.set(xlabel="x [m]", ylabel="y [m]", zlabel="z [m]")
        ax.legend(loc="upper left")
        ax2 = fig.add_subplot(122)
        ax2.plot(t_axis, U[:, 0], label="T [N]")
        ax2.axhline(T_MAX, color="r", ls=":", label="T max")
        ax2.plot(t_axis, E * 10, label="sai số ×10 [m]")
        ax2.set_xlabel("t [s]")
        ax2.legend()
        ax2.grid(True)
        fig.tight_layout()
        png = os.path.join(workdir, "quad_mpc.png")
        fig.savefig(png, dpi=120)
        print(f"Đồ thị: {png}")
    except ImportError:
        pass

    tol = 1e-6
    assert rms < 0.10, f"RMS {rms:.3f} m ≥ 0.10 m"
    assert U[:, 0].min() >= -tol and U[:, 0].max() <= T_MAX + tol, "vi phạm ràng buộc lực đẩy"
    assert np.abs(U[:, 1:]).max() <= W_MAX + tol, "vi phạm ràng buộc vận tốc góc"
    assert t_ms < 10.0, f"giải chậm: {t_ms:.2f} ms"
    print("OK")


if __name__ == "__main__":
    sys.exit(main())
