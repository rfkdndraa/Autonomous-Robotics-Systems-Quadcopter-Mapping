"""
UTS - 3D mapping dengan drone (CoppeliaSim + Python ZMQ Remote API)

NAMA: OKTAVIAN RIFKI DANENDRA
NRP: 5022231107
SRO FALL SEASON
"""
import time
import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ---------------- KONFIGURASI ----------------
TARGET_PATH = "/target"                  # dummy target yang diikuti quadcopter
DRONE_PATH = "/Quadcopter"               # badan drone (untuk merekam lintasan)
SCANNER_PATHS = ["/Quadcopter/fast3DLaserScanner", "/fast3DLaserScanner"]

PATH_MODE = "lawnmower"   # Pilihan navigasi, "figure8" = navigasi quadcopter membentuk angka 8, "lawnmower" = menyapu area
FIG8_LAPS = 3           # berapa kali memutar angka 8
FIG8_A = 2.0            # setengah panjang angka 8 pada sumbu x (m)
FIG8_B = 1.2            # setengah lebar angka 8 pada sumbu y (m)
FIG8_POINTS_PER_LAP = 40  # jumlah waypoint per putaran (jarak antar waypoint harus > WP_TOL)

ARENA_HALF = 2.0     # setengah lebar area yang discan (m); juga dipakai untuk batas peta 2D
ALTITUDE = 0.8       # ketinggian terbang (m)
LANE_SPACING = 0.30   # jarak antar jalur mode "lawnmower" (m)
STEP_DIST = 0.05     # jarak gerak target per langkah simulasi (m)

# --- Obstacle avoidance ---
AVOID_OBSTACLES = True
EXCLUDE_ALIASES = ("floor", "grid")  # shape yang namanya mengandung kata ini TIDAK dianggap penghalang
SAFE_DIST = 0.9      # mulai menghindar kalau jarak badan drone ke penghalang < ini (m)
AVOID_GAIN = 1.5     # kekuatan dorongan menjauh
SLIDE_GAIN = 0.5     # kekuatan gerak menyamping menyusuri penghalang
MAX_PUSH = 0.10      # batas dorongan per langkah (m)
LOOKAHEAD = 0.5      # target diarahkan ke titik segini (m) di depan pada garis lajur; 0 = langsung ke waypoint
MAX_LEAD = 0.6       # target tidak boleh lebih jauh dari ini di depan drone (m)
WP_TOL = 0.2         # waypoint dianggap tercapai kalau jarak < ini (m)
MAX_STEPS_PER_WP = 800  # batas langkah per waypoint sebelum dilewati

# --- Scan & voxel ---
SCAN_EVERY = 6       # ambil depth setiap N langkah simulasi
VOXEL_SIZE = 0.10    # ukuran voxel (m)
CAMERA_CONVENTION = "auto"  # "auto" = pilih otomatis; atau paksa "flip_x" / "flip_y"
DEPTH_IN_METERS = True  # True: depth dari getVisionSensorDepth sudah dalam meter
MAX_RANGE = 6.0      # titik lebih jauh dari ini (m) dari sensor dibuang; None = tidak difilter
MIN_RANGE = 0.3      # titik lebih dekat dari ini (m) dari sensor dibuang (badan/baling-baling drone); None = mati
MIN_NEIGHBORS = 2    # voxel dengan tetangga (dari 26 voxel sekitarnya) kurang dari ini dibuang; 0 = mati
DEBUG = True         # cetak info tiap sensor sekali (resolusi, depth, arah pandang, bounding box)

# --- Tampilan peta ---
GROUND_Z = 0.15      # voxel dengan z <= ini dianggap tanah/lantai; di atasnya dianggap objek
GROUND_COLOR = "#fb8af2"  # warna tanah (satu warna tetap)
OBJECT_CMAP = "plasma"    # colormap objek menurut tinggi

CONVENTIONS = {"flip_x": (-1.0, 1.0), "flip_y": (1.0, -1.0)}
_debugged = set()


# ---------------- LINTASAN ----------------
def figure8_waypoints(a, b, z, laps, per_lap):
    "Lintasan angka 8 (lemniscate Gerono): x = a sin t, y = b sin 2t. Mulai & selesai di tengah."
    t = np.linspace(0, 2 * np.pi * laps, per_lap * laps + 1)
    return [(a * np.sin(ti), b * np.sin(2 * ti), z) for ti in t]


def lawnmower_waypoints(half, lane, z):
    xs = np.arange(-half, half + 1e-6, lane)
    wps = []
    for i, x in enumerate(xs):
        ys = (-half, half) if i % 2 == 0 else (half, -half)
        wps += [(x, ys[0], z), (x, ys[1], z)]
    return wps


def make_waypoints():
    if PATH_MODE == "figure8":
        return figure8_waypoints(FIG8_A, FIG8_B, ALTITUDE, FIG8_LAPS, FIG8_POINTS_PER_LAP)
    return lawnmower_waypoints(ARENA_HALF, LANE_SPACING, ALTITUDE)


# ---------------- SENSOR ----------------
def frame_to_world(L, m, conv):
    sx, sy = CONVENTIONS[conv]
    local = L * np.array([sx, sy, 1.0])
    return local @ m[:, :3].T + m[:, 3]


def capture(sim, sensor, near, far, fov):
    "Ambil 1 frame depth -> (titik di koordinat sensor, matriks pose sensor), atau None kalau kosong."
    try:
        sim.handleVisionSensor(sensor)
    except Exception:
        pass                             
    buf, (w, h) = sim.getVisionSensorDepth(sensor, 1)
    d = np.array(sim.unpackFloatTable(buf), dtype=np.float32).reshape(h, w)
    z = d if DEPTH_IN_METERS else near + d * (far - near)   # meter, sepanjang sumbu optik
    f = (max(w, h) / 2.0) / np.tan(fov / 2.0)               # FOV berlaku untuk sisi terpanjang
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    ux = (u - (w - 1) / 2.0) / f * z                        # kolom piksel -> meter
    vy = (v - (h - 1) / 2.0) / f * z                        # baris piksel -> meter
    valid = (z < far * 0.999) & (z > near)                  # buang piksel "tak kena apa-apa"
    L = np.stack([ux, vy, z], axis=-1)[valid]
    dist = np.linalg.norm(L, axis=1)
    if MAX_RANGE is not None:
        L = L[dist <= MAX_RANGE]                            # buang titik terlalu jauh
        dist = dist[dist <= MAX_RANGE]
    if MIN_RANGE is not None:
        L = L[dist >= MIN_RANGE]                            # buang titik terlalu dekat (bagian drone sendiri)
    if len(L) == 0:
        return None
    m = np.array(sim.getObjectMatrix(sensor, -1)).reshape(3, 4)
    if DEBUG and sensor not in _debugged:
        _debugged.add(sensor)
        W = frame_to_world(L, m, "flip_x")
        print(f"[debug] {sim.getObjectAlias(sensor)}: res={w}x{h} depth=[{d.min():.3f},{d.max():.3f}] "
              f"valid={valid.mean():.0%} near/far/fov=({near:.2f},{far:.2f},{np.rad2deg(fov):.0f}) "
              f"pos={np.round(m[:, 3], 2)} arah_pandang={np.round(m[:, 2], 2)} "
              f"world_min={np.round(W.min(0), 2)} world_max={np.round(W.max(0), 2)}")
    return L, m


def choose_convention(frames):
    """Pilih konvensi sumbu yang membuat objek dari banyak sudut pandang menumpuk di tempat yang sama
    (jumlah voxel objek paling sedikit). Konvensi yang salah menyebarkan objek jadi lebih banyak voxel."""
    if CAMERA_CONVENTION in CONVENTIONS:
        return CAMERA_CONVENTION
    scores = {}
    for conv in CONVENTIONS:
        P = np.vstack([frame_to_world(L, m, conv) for L, m in frames])
        P = P[P[:, 2] > GROUND_Z + 0.1]
        scores[conv] = len(np.unique(np.floor(P / VOXEL_SIZE).astype(np.int64), axis=0)) if len(P) else 0
    best = min(scores, key=scores.get)
    print(f"Pemilihan konvensi sumbu kamera (jumlah voxel objek, makin kecil makin konsisten): {scores} -> {best}")
    lo, hi = sorted(scores.values())
    if hi == 0 or (hi - lo) / hi < 0.05:
        print("PERINGATAN: skor kedua konvensi hampir sama (objek terlalu sedikit?). "
              "Cek manual posisi kotak, lalu set CAMERA_CONVENTION = 'flip_x' atau 'flip_y'.")
    return best


def find_scanner(sim, drone):
    """Cari model fast3DLaserScanner; pasang ke drone kalau belum jadi anaknya."""
    for path in SCANNER_PATHS:
        try:
            root = sim.getObject(path)
        except Exception:
            continue
        if sim.getObjectParent(root) != drone:
            sim.setObjectParent(root, drone, True)
            sim.setObjectPosition(root, drone, [0, 0, -0.1])
            print(f"fast3DLaserScanner ({path}) dipasang ke drone.")
        return root
    raise RuntimeError(f"fast3DLaserScanner tidak ditemukan di scene. Path yang dicoba: {SCANNER_PATHS}")


def get_sensors(sim, drone):
    """Kembalikan daftar vision sensor (handle, near, far, fov) di dalam model scanner."""
    root = find_scanner(sim, drone)
    sensors = []
    for h in sim.getObjectsInTree(root, sim.object_visionsensor_type, 0):
        near = sim.getObjectFloatParam(h, sim.visionfloatparam_near_clipping)
        far = sim.getObjectFloatParam(h, sim.visionfloatparam_far_clipping)
        fov = sim.getObjectFloatParam(h, sim.visionfloatparam_perspective_angle)
        print(f"Sensor dipakai: {sim.getObjectAlias(h)} | near={near:.2f} far={far:.2f} fov={np.rad2deg(fov):.0f} deg")
        sensors.append((h, near, far, fov))
    if not sensors:
        raise RuntimeError("Tidak ada vision sensor di dalam model fast3DLaserScanner.")
    return sensors


# ---------------- OBSTACLE AVOIDANCE & NAVIGASI ----------------
def find_obstacles(sim, drone):
    """Semua shape di scene kecuali drone sendiri dan lantai -> dianggap penghalang."""
    mine = set(sim.getObjectsInTree(drone, sim.object_shape_type, 0))
    obstacles = []
    for h in sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0):
        if h in mine:
            continue
        name = sim.getObjectAlias(h)
        if any(s in name.lower() for s in EXCLUDE_ALIASES):
            continue
        obstacles.append(h)
        print(f"Penghalang: {name}")
    return obstacles


def avoidance(sim, drone, obstacles, goal_dir):
    """Potential field: dorong menjauh dari penghalang yang lebih dekat dari SAFE_DIST,
    ditambah komponen menyamping (tangen) ke arah tujuan supaya drone menyusuri sisi penghalang."""
    push = np.zeros(3)
    for ob in obstacles:
        res, data, _ = sim.checkDistance(drone, ob, SAFE_DIST)
        if res != 1:
            continue                                   # penghalang ini lebih jauh dari SAFE_DIST
        p_drone, p_obs, d = np.array(data[0:3]), np.array(data[3:6]), max(data[6], 1e-3)
        away = (p_drone - p_obs) / d
        w = (SAFE_DIST - d) / SAFE_DIST                # 0 (di batas aman) .. 1 (menempel)
        push += away * w
        tang = np.array([-away[1], away[0], 0.0])
        if np.linalg.norm(tang) > 1e-6:
            tang /= np.linalg.norm(tang)
            if np.dot(tang, goal_dir) < 0:
                tang = -tang
            push += tang * w * SLIDE_GAIN
    push *= AVOID_GAIN * STEP_DIST
    n = np.linalg.norm(push)
    return push if n <= MAX_PUSH else push / n * MAX_PUSH


def goto(sim, client, target, drone, wp, obstacles, on_step, start):
    """Terbang dari `start` ke `wp` sambil menghindar. Target diarahkan ke titik di GARIS start->wp
    (sedikit di depannya, LOOKAHEAD), jadi setelah menghindar target kembali ke garis lajur.
    Berhenti kalau sampai atau kehabisan langkah."""
    tgt = np.array(sim.getObjectPosition(target, -1), dtype=float)
    seg = wp - start
    seg_len = np.linalg.norm(seg)
    seg_dir = seg / seg_len if seg_len > 1e-6 else np.zeros(3)
    for _ in range(MAX_STEPS_PER_WP):
        pos = np.array(sim.getObjectPosition(drone, -1), dtype=float)
        if np.linalg.norm(wp - pos) < WP_TOL:
            return
        if LOOKAHEAD > 0 and seg_len > 1e-6:
            s = np.clip(np.dot(tgt - start, seg_dir), 0.0, seg_len)      # posisi target sepanjang garis
            carrot = start + seg_dir * min(seg_len, s + LOOKAHEAD)       # titik tujuan di garis
        else:
            carrot = wp
        to_goal = carrot - tgt
        dist = np.linalg.norm(to_goal)
        goal_dir = to_goal / dist if dist > 1e-6 else np.zeros(3)
        step = goal_dir * min(STEP_DIST, dist)
        if np.linalg.norm(tgt - pos) > MAX_LEAD:
            step = np.zeros(3)                         # tunggu drone mengejar target
        push = avoidance(sim, drone, obstacles, goal_dir) if obstacles else 0.0
        tgt = tgt + step + push
        sim.setObjectPosition(target, -1, tgt.tolist())
        client.step()
        on_step(pos)
    print(f"Waypoint {np.round(wp, 2)} tidak tercapai (terhalang) -> dilewati.")


def fly_and_scan(sim, client):
    target = sim.getObject(TARGET_PATH)
    drone = sim.getObject(DRONE_PATH)
    sensors = get_sensors(sim, drone)
    obstacles = find_obstacles(sim, drone) if AVOID_OBSTACLES else []

    frames, path = [], []
    counter = [0]

    def on_step(pos):
        path.append(pos.tolist())
        counter[0] += 1
        if counter[0] % SCAN_EVERY == 0:
            for h, near, far, fov in sensors:
                fr = capture(sim, h, near, far, fov)
                if fr is not None:
                    frames.append(fr)

    prev = np.array(sim.getObjectPosition(target, -1), dtype=float)
    for wp in make_waypoints():
        wp = np.array(wp, dtype=float)
        goto(sim, client, target, drone, wp, obstacles, on_step, prev)
        prev = wp
    return frames, np.array(path)


# ---------------- POST PROCESS & PLOT ----------------
def voxel_downsample(P, size):
    """Satu titik per voxel (pusat voxel) -> jumlah titik berkurang drastis."""
    idx = np.floor(P / size).astype(np.int64)
    _, first = np.unique(idx, axis=0, return_index=True)
    return (idx[first] + 0.5) * size


def remove_isolated(vox, size, min_neighbors):
    """Buang voxel terpencil (noise): voxel yang punya kurang dari min_neighbors tetangga
    di 26 voxel sekelilingnya. Permukaan lantai dan badan objek punya banyak tetangga, jadi aman."""
    if min_neighbors <= 0 or len(vox) == 0:
        return vox
    idx = np.floor(vox / size).astype(np.int64)
    idx -= idx.min(axis=0)
    dims = idx.max(axis=0) + 3                       # +3 supaya offset -1..+1 tidak menabrak batas
    def key(a):
        a = a + 1
        return (a[:, 0] * dims[1] + a[:, 1]) * dims[2] + a[:, 2]
    keys = np.sort(key(idx))
    count = np.zeros(len(idx), dtype=np.int32)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == dy == dz == 0:
                    continue
                k = key(idx + np.array([dx, dy, dz]))
                pos = np.searchsorted(keys, k)
                pos[pos == len(keys)] = len(keys) - 1
                count += (keys[pos] == k)
    return vox[count >= min_neighbors]


def occupancy_2d(P, size):
    """Scan 2D: proyeksi top-down semua titik di atas lantai."""
    obj = P[P[:, 2] > GROUND_Z]
    edges = np.arange(-ARENA_HALF - 1, ARENA_HALF + 1 + size, size)
    grid, _, _ = np.histogram2d(obj[:, 0], obj[:, 1], bins=[edges, edges])
    return (grid > 0).T, edges


def plot_all(vox, path, occ, edges):
    # ---- Peta 3D: voxel kotak (tanah satu warna, objek berwarna menurut tinggi) + jalur drone ----
    lim = float(np.ceil(np.abs(np.r_[vox[:, :2].ravel(), path[:, :2].ravel()]).max()))
    zmax = float(max(vox[:, 2].max(), path[:, 2].max())) + 0.5

    fig = plt.figure(figsize=(9, 7.5))
    ax = fig.add_subplot(111, projection="3d")
    is_ground = vox[:, 2] <= GROUND_Z
    ground, objs = vox[is_ground], vox[~is_ground]
    ax.scatter(ground[:, 0], ground[:, 1], ground[:, 2], color=GROUND_COLOR,
               marker="s", s=14, linewidths=0, depthshade=False)
    if len(objs):
        sc = ax.scatter(objs[:, 0], objs[:, 1], objs[:, 2], c=objs[:, 2], cmap=OBJECT_CMAP,
                        vmin=GROUND_Z, vmax=objs[:, 2].max(), marker="s", s=14,
                        linewidths=0, depthshade=False)
        fig.colorbar(sc, ax=ax, shrink=0.5, pad=0.08, label="Tinggi objek (m)")
    ax.scatter([], [], color=GROUND_COLOR, marker="s", label="Tanah")
    ax.scatter([], [], color=plt.get_cmap(OBJECT_CMAP)(0.7), marker="s", label="Objek")
    ax.plot(path[:, 0], path[:, 1], path[:, 2], color="blue", lw=2)
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(0, zmax)
    ax.set_box_aspect((1, 1, 0.45))
    ax.view_init(elev=45, azim=-60)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.set_title("3D Map")
    ax.legend(loc="upper left")
    plt.tight_layout()
    plt.savefig("hasil_3d_map.png", dpi=150)

    # ---- Peta 2D (top-down) terpisah ----
    fig2, ax2 = plt.subplots(figsize=(6, 6))
    ax2.imshow(occ, origin="lower", cmap="gray_r",
               extent=[edges[0], edges[-1], edges[0], edges[-1]])
    ax2.plot(path[:, 0], path[:, 1], color="blue", lw=1)
    ax2.set_title("Peta 2D (top-down) + jalur drone")
    ax2.set_xlabel("X"); ax2.set_ylabel("Y")
    plt.tight_layout()
    plt.savefig("hasil_2d_map.png", dpi=150)
    plt.show()


def main():
    client = RemoteAPIClient()
    sim = client.require("sim")
    client.setStepping(True)
    sim.startSimulation()
    try:
        frames, path = fly_and_scan(sim, client)
    finally:
        sim.stopSimulation()
        time.sleep(0.5)

    conv = choose_convention(frames)
    raw = np.vstack([frame_to_world(L, m, conv) for L, m in frames])
    vox = voxel_downsample(raw, VOXEL_SIZE)
    n_before = len(vox)
    vox = remove_isolated(vox, VOXEL_SIZE, MIN_NEIGHBORS)
    if MIN_NEIGHBORS > 0:
        print(f"Voxel terpencil dibuang: {n_before - len(vox)} dari {n_before}")
    occ, edges = occupancy_2d(vox, VOXEL_SIZE)
    np.save("pointcloud_raw.npy", raw)
    np.save("pointcloud_voxel.npy", vox)
    print(f"Titik mentah: {len(raw)} | setelah voxel: {len(vox)}")
    print(f"Voxel tanah (z<={GROUND_Z}): {(vox[:, 2] <= GROUND_Z).sum()} | objek: {(vox[:, 2] > GROUND_Z).sum()}")
    print(f"Bounding box voxel: min={np.round(vox.min(0), 2)} max={np.round(vox.max(0), 2)}")
    print(f"Jalur drone: min={np.round(path.min(0), 2)} max={np.round(path.max(0), 2)}")
    plot_all(vox, path, occ, edges)


if __name__ == "__main__":
    main()
