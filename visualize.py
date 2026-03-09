import csv
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ─────────────────────────────────────────────
#  CSV okuyucular
# ─────────────────────────────────────────────

def load_environment_csv(path="build/environment_data.csv"):
    """environment_data.csv → world bounds, obstacles, agents"""
    data = {"world_min": None, "world_max": None, "obstacles": {}, "starts": {}, "goals": {}}
    with open(path) as f:
        for row in csv.DictReader(f):
            t = row["type"]
            p = (float(row["x"]), float(row["y"]), float(row["z"]))
            idx = int(row["extra"])
            if   t == "world_min": data["world_min"] = p
            elif t == "world_max": data["world_max"] = p
            elif t == "obs_min":
                data["obstacles"].setdefault(idx, {})["min"] = p
            elif t == "obs_max":
                data["obstacles"].setdefault(idx, {})["max"] = p
            elif t == "start":  data["starts"][idx] = p
            elif t == "goal":   data["goals"][idx]  = p
    return data


def load_octree_csv(path="build/octree_voxels.csv"):
    """octree_voxels.csv → Nx3 numpy array"""
    pts = []
    with open(path) as f:
        for row in csv.DictReader(f):
            pts.append((float(row["x"]), float(row["y"]), float(row["z"])))
    return np.array(pts) if pts else np.empty((0, 3))


def load_vertices_csv(path="build/vertices.csv"):
    """vertices.csv → {id: (x,y,z)}"""
    verts = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            verts[int(row["id"])] = (float(row["x"]), float(row["y"]), float(row["z"]))
    return verts


def load_edges_csv(path="build/edges.csv"):
    """edges.csv → list of (from_id, to_id, cost)"""
    edges = []
    with open(path) as f:
        for row in csv.DictReader(f):
            edges.append((int(row["from"]), int(row["to"]), float(row["cost"])))
    return edges

# ─────────────────────────────────────────────
#  Çizim yardımcıları
# ─────────────────────────────────────────────

def draw_box_3d(ax, mn, mx, facecolor, alpha=0.35, edgecolor="white", lw=0.6):
    x0,y0,z0 = mn;  x1,y1,z1 = mx
    faces = [
        [[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0]],
        [[x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]],
        [[x0,y0,z0],[x0,y0,z1],[x0,y1,z1],[x0,y1,z0]],
        [[x1,y0,z0],[x1,y0,z1],[x1,y1,z1],[x1,y1,z0]],
        [[x0,y0,z0],[x1,y0,z0],[x1,y0,z1],[x0,y0,z1]],
        [[x0,y1,z0],[x1,y1,z0],[x1,y1,z1],[x0,y1,z1]],
    ]
    poly = Poly3DCollection(faces, alpha=alpha, linewidths=lw)
    poly.set_facecolor(facecolor)
    poly.set_edgecolor(edgecolor)
    ax.add_collection3d(poly)


def draw_ellipsoid_3d(ax, center, rx, ry, rz, color, alpha=0.35):
    u = np.linspace(0, 2*np.pi, 18)
    v = np.linspace(0, np.pi,    9)
    x = rx * np.outer(np.cos(u), np.sin(v)) + center[0]
    y = ry * np.outer(np.sin(u), np.sin(v)) + center[1]
    z = rz * np.outer(np.ones_like(u), np.cos(v)) + center[2]
    ax.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0)


PALETTE = {
    "bg_fig":   "#1a1a2e",
    "bg_ax":    "#16213e",
    "world":    "#5566aa",
    "obs":      ["#e74c3c", "#e67e22", "#f39c12", "#9b59b6", "#1abc9c"],
    "agents":   ["#2ecc71", "#3498db", "#f1c40f", "#e91e63"],
    "edge":     "#aaaacc",
    "vertex":   "#ffffff",
}

# ─────────────────────────────────────────────
#  1. 3D Görünüm
# ─────────────────────────────────────────────

def plot_3d(env, voxels, vertices, edges, out="environment_3d.png"):
    fig = plt.figure(figsize=(14, 10))
    fig.patch.set_facecolor(PALETTE["bg_fig"])
    ax  = fig.add_subplot(111, projection="3d")
    ax.set_facecolor(PALETTE["bg_ax"])

    wmin = env["world_min"]
    wmax = env["world_max"]

    # Dünya kutusu
    draw_box_3d(ax, wmin, wmax, facecolor=PALETTE["world"], alpha=0.04, edgecolor="#7788bb")

    # Engeller
    for i, obs in env["obstacles"].items():
        col = PALETTE["obs"][i % len(PALETTE["obs"])]
        draw_box_3d(ax, obs["min"], obs["max"], facecolor=col, alpha=0.55)
        cx = (obs["min"][0] + obs["max"][0]) / 2
        cy = (obs["min"][1] + obs["max"][1]) / 2
        cz =  obs["max"][2] + 0.3
        ax.text(cx, cy, cz, f"Obs {i+1}", color="white", fontsize=7, ha="center")

    # Voxellar (nokta bulutu)
    if len(voxels) > 0:
        ax.scatter(voxels[:,0], voxels[:,1], voxels[:,2],
                   c="#ff6666", s=1.5, alpha=0.08, depthshade=False)

    # Graph — kenarlar
    for (fr, to, _) in edges:
        if fr in vertices and to in vertices:
            a, b = vertices[fr], vertices[to]
            ax.plot([a[0],b[0]], [a[1],b[1]], [a[2],b[2]],
                    color=PALETTE["edge"], lw=1.0, alpha=0.7)

    # Graph — düğümler
    if vertices:
        vx = np.array(list(vertices.values()))
        ax.scatter(vx[:,0], vx[:,1], vx[:,2],
                   c=PALETTE["vertex"], s=25, zorder=8)

    # Agentlar
    all_ids = set(env["starts"]) | set(env["goals"])
    legend_items = []
    for ag_id in sorted(all_ids):
        col = PALETTE["agents"][ag_id % len(PALETTE["agents"])]
        s = env["starts"].get(ag_id)
        g = env["goals"].get(ag_id)
        if s:
            ax.scatter(*s, c=col, s=180, marker="*", zorder=12)
            ax.text(s[0], s[1], s[2]+0.5, f"S{ag_id}", color=col, fontsize=8)
            draw_ellipsoid_3d(ax, s, rx=0.15, ry=0.15, rz=0.5, color=col, alpha=0.4)
        if g:
            ax.scatter(*g, c=col, s=140, marker="s", zorder=12)
            ax.text(g[0], g[1], g[2]+0.5, f"G{ag_id}", color=col, fontsize=8)
        if s and g:
            ax.plot([s[0],g[0]], [s[1],g[1]], [s[2],g[2]],
                    "--", color=col, alpha=0.35, lw=1.2)
        legend_items.append(mpatches.Patch(color=col, label=f"Robot {ag_id}"))

    for i in range(len(env["obstacles"])):
        legend_items.append(mpatches.Patch(
            color=PALETTE["obs"][i % len(PALETTE["obs"])], label=f"Engel {i+1}"))

    # Eksenler
    margin = 1.0
    ax.set_xlim(wmin[0]-margin, wmax[0]+margin)
    ax.set_ylim(wmin[1]-margin, wmax[1]+margin)
    ax.set_zlim(wmin[2]-margin, wmax[2]+margin)
    for label, setter in [("X", ax.set_xlabel), ("Y", ax.set_ylabel), ("Z", ax.set_zlabel)]:
        setter(label, color="white", labelpad=8)
    ax.tick_params(colors="white", labelsize=7)
    ax.set_title("3D Environment — Engeller + Robotlar + Graph",
                 color="white", fontsize=12, pad=12)
    ax.legend(handles=legend_items, loc="upper left",
              facecolor=PALETTE["bg_fig"], labelcolor="white", fontsize=8)
    ax.view_init(elev=22, azim=-55)

    plt.tight_layout()
    plt.savefig(out, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close()
    print(f"Kaydedildi: {out}")

# ─────────────────────────────────────────────
#  2. Üstten görünüm (X-Y)
# ─────────────────────────────────────────────

def plot_topdown(env, vertices, edges, out="environment_topdown.png"):
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.patch.set_facecolor(PALETTE["bg_fig"])
    ax.set_facecolor(PALETTE["bg_ax"])

    wmin, wmax = env["world_min"], env["world_max"]

    # Dünya sınırı
    ax.add_patch(plt.Rectangle(
        (wmin[0], wmin[1]), wmax[0]-wmin[0], wmax[1]-wmin[1],
        fill=False, edgecolor=PALETTE["world"], lw=1.5, linestyle="--"))

    # Engeller
    for i, obs in env["obstacles"].items():
        col = PALETTE["obs"][i % len(PALETTE["obs"])]
        mn, mx = obs["min"], obs["max"]
        ax.add_patch(plt.Rectangle(
            (mn[0], mn[1]), mx[0]-mn[0], mx[1]-mn[1],
            color=col, alpha=0.6, label=f"Engel {i+1}"))
        cx = (mn[0]+mx[0])/2;  cy = (mn[1]+mx[1])/2
        ax.text(cx, cy, f"Obs {i+1}\nz:[{mn[2]:.1f},{mx[2]:.1f}]",
                color="white", ha="center", va="center", fontsize=8)

    # Graph — kenarlar
    for (fr, to, cost) in edges:
        if fr in vertices and to in vertices:
            a, b = vertices[fr], vertices[to]
            ax.plot([a[0],b[0]], [a[1],b[1]],
                    color=PALETTE["edge"], lw=1.2, alpha=0.75, zorder=4)
            mx2 = (a[0]+b[0])/2;  my2 = (a[1]+b[1])/2
            ax.text(mx2, my2, f"{cost:.1f}", color="#aaaacc",
                    fontsize=6, ha="center", alpha=0.7)

    # Graph — düğümler
    if vertices:
        vx = np.array(list(vertices.values()))
        ax.scatter(vx[:,0], vx[:,1], c=PALETTE["vertex"],
                   s=30, zorder=6, label="Waypoint")
        for vid, (x,y,z) in vertices.items():
            ax.text(x+0.15, y+0.15, f"v{vid}", color="white", fontsize=6)

    # Agentlar
    all_ids = set(env["starts"]) | set(env["goals"])
    for ag_id in sorted(all_ids):
        col = PALETTE["agents"][ag_id % len(PALETTE["agents"])]
        s = env["starts"].get(ag_id)
        g = env["goals"].get(ag_id)
        if s and g:
            ax.plot([s[0],g[0]], [s[1],g[1]], "--", color=col, alpha=0.45, lw=1.5)
        if s:
            ax.scatter(s[0], s[1], c=col, s=220, marker="*", zorder=10, label=f"Robot {ag_id} Start")
            ax.annotate(f" S{ag_id}(z={s[2]:.1f})", (s[0], s[1]), color=col, fontsize=8)
            ax.add_patch(plt.Circle((s[0],s[1]), 0.15, color=col, fill=False, linestyle=":", lw=1))
        if g:
            ax.scatter(g[0], g[1], c=col, s=160, marker="s", zorder=10, label=f"Robot {ag_id} Goal")
            ax.annotate(f" G{ag_id}(z={g[2]:.1f})", (g[0], g[1]), color=col, fontsize=8)

    ax.set_xlim(wmin[0]-1, wmax[0]+1)
    ax.set_ylim(wmin[1]-1, wmax[1]+1)
    ax.set_xlabel("X", color="white");  ax.set_ylabel("Y", color="white")
    ax.tick_params(colors="white")
    ax.set_title("Üstten Görünüm (X-Y) — Engeller + Graph",
                 color="white", fontsize=12)
    ax.legend(facecolor=PALETTE["bg_fig"], labelcolor="white", fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.12, color="white")

    plt.tight_layout()
    plt.savefig(out, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close()
    print(f"Kaydedildi: {out}")

# ─────────────────────────────────────────────
#  3. Yan görünüm (X-Z)
# ─────────────────────────────────────────────

def plot_sideview(env, vertices, edges, out="environment_side.png"):
    fig, ax = plt.subplots(figsize=(12, 7))
    fig.patch.set_facecolor(PALETTE["bg_fig"])
    ax.set_facecolor(PALETTE["bg_ax"])

    wmin, wmax = env["world_min"], env["world_max"]
    ax.add_patch(plt.Rectangle(
        (wmin[0], wmin[2]), wmax[0]-wmin[0], wmax[2]-wmin[2],
        fill=False, edgecolor=PALETTE["world"], lw=1.5, linestyle="--"))

    for i, obs in env["obstacles"].items():
        col = PALETTE["obs"][i % len(PALETTE["obs"])]
        mn, mx = obs["min"], obs["max"]
        ax.add_patch(plt.Rectangle(
            (mn[0], mn[2]), mx[0]-mn[0], mx[2]-mn[2],
            color=col, alpha=0.6, label=f"Engel {i+1}"))
        ax.text((mn[0]+mx[0])/2, (mn[2]+mx[2])/2,
                f"Obs {i+1}", color="white", ha="center", va="center", fontsize=8)

    for (fr, to, _) in edges:
        if fr in vertices and to in vertices:
            a, b = vertices[fr], vertices[to]
            ax.plot([a[0],b[0]], [a[2],b[2]], color=PALETTE["edge"], lw=1.2, alpha=0.75)

    if vertices:
        vx = np.array(list(vertices.values()))
        ax.scatter(vx[:,0], vx[:,2], c=PALETTE["vertex"], s=30, zorder=6)

    all_ids = set(env["starts"]) | set(env["goals"])
    for ag_id in sorted(all_ids):
        col = PALETTE["agents"][ag_id % len(PALETTE["agents"])]
        s = env["starts"].get(ag_id)
        g = env["goals"].get(ag_id)
        if s: ax.scatter(s[0], s[2], c=col, s=200, marker="*", zorder=10)
        if g: ax.scatter(g[0], g[2], c=col, s=150, marker="s", zorder=10)
        if s and g:
            ax.plot([s[0],g[0]], [s[2],g[2]], "--", color=col, alpha=0.4, lw=1.5)

    ax.set_xlim(wmin[0]-1, wmax[0]+1)
    ax.set_ylim(wmin[2]-0.5, wmax[2]+1)
    ax.set_xlabel("X", color="white");  ax.set_ylabel("Z (Yükseklik)", color="white")
    ax.tick_params(colors="white")
    ax.set_title("Yan Görünüm (X-Z) — Yükseklik Profili", color="white", fontsize=12)
    ax.legend(facecolor=PALETTE["bg_fig"], labelcolor="white", fontsize=8)
    ax.grid(True, alpha=0.12, color="white")
    ax.axhline(y=0, color="#aaaaaa", lw=0.8, linestyle=":")
    ax.text(wmin[0]+0.2, 0.05, "Zemin", color="#aaaaaa", fontsize=8)

    plt.tight_layout()
    plt.savefig(out, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close()
    print(f"Kaydedildi: {out}")

# ─────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────

def main():
    # CSV'lerin yolu (build/ dizini scriptle aynı seviyede)
    base = os.path.dirname(os.path.abspath(__file__))
    build = os.path.join(base, "build")

    env_csv  = os.path.join(build, "environment_data.csv")
    oct_csv  = os.path.join(build, "octree_voxels.csv")
    vert_csv = os.path.join(build, "vertices.csv")
    edge_csv = os.path.join(build, "edges.csv")

    # Dosya varlık kontrolü
    missing = [p for p in [env_csv, oct_csv] if not os.path.exists(p)]
    if missing:
        print("HATA: Önce C++ programını derleyip çalıştır:")
        print("  cd build && cmake .. && make && ./env_test")
        for m in missing: print(f"  Eksik: {m}")
        return

    print("CSV dosyaları yükleniyor...")
    env      = load_environment_csv(env_csv)
    voxels   = load_octree_csv(oct_csv)
    vertices = load_vertices_csv(vert_csv)  if os.path.exists(vert_csv) else {}
    edges    = load_edges_csv(edge_csv)     if os.path.exists(edge_csv) else []

    print(f"  Engel sayısı : {len(env['obstacles'])}")
    print(f"  Agent sayısı : {len(env['starts'])}")
    print(f"  Voxel sayısı : {len(voxels)}")
    print(f"  Vertex sayısı: {len(vertices)}")
    print(f"  Edge sayısı  : {len(edges)}")
    print()

    out_dir = base  # görseller scriptle aynı yerde
    plot_3d      (env, voxels, vertices, edges, os.path.join(out_dir, "environment_3d.png"))
    plot_topdown (env, vertices, edges,         os.path.join(out_dir, "environment_topdown.png"))
    plot_sideview(env, vertices, edges,         os.path.join(out_dir, "environment_side.png"))

    print("\nTüm görseller oluşturuldu.")

if __name__ == "__main__":
    main()