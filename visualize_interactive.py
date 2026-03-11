import csv
import os
import numpy as np
import plotly.graph_objects as go
from scipy.spatial import ConvexHull

# ─────────────────────────────────────────────
#  CSV okuyucular
# ─────────────────────────────────────────────

def load_environment_csv(path):
    data = {"world_min": None, "world_max": None, "obstacles": {}, "starts": {}, "goals": {}}
    with open(path) as f:
        for row in csv.DictReader(f):
            t   = row["type"]
            p   = (float(row["x"]), float(row["y"]), float(row["z"]))
            idx = int(row["extra"])
            if   t == "world_min": data["world_min"] = p
            elif t == "world_max": data["world_max"] = p
            elif t == "obs_min":   data["obstacles"].setdefault(idx, {})["min"] = p
            elif t == "obs_max":   data["obstacles"].setdefault(idx, {})["max"] = p
            elif t == "start":     data["starts"][idx] = p
            elif t == "goal":      data["goals"][idx]  = p
    return data


def load_vertices_csv(path):
    verts = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            verts[int(row["id"])] = (float(row["x"]), float(row["y"]), float(row["z"]))
    return verts


def load_edges_csv(path):
    edges = []
    with open(path) as f:
        for row in csv.DictReader(f):
            edges.append((int(row["from"]), int(row["to"]), float(row["cost"])))
    return edges


# ─────────────────────────────────────────────
#  AABB → 8 köşe noktası
# ─────────────────────────────────────────────
def aabb_vertices(mn, mx):
    x0,y0,z0 = mn
    x1,y1,z1 = mx
    return np.array([
        [x0,y0,z0], [x1,y0,z0], [x1,y1,z0], [x0,y1,z0],
        [x0,y0,z1], [x1,y0,z1], [x1,y1,z1], [x0,y1,z1],
    ])


# ─────────────────────────────────────────────
#  ConvexHull'dan Mesh3d trace — arkadaşının
#  yöntemiyle aynı, temiz mesh garantili
# ─────────────────────────────────────────────
def mesh_from_hull(mn, mx, color, opacity, name, showlegend=True):
    verts = aabb_vertices(mn, mx)
    hull  = ConvexHull(verts)
    x, y, z = verts[:,0], verts[:,1], verts[:,2]
    i_f = hull.simplices[:,0]
    j_f = hull.simplices[:,1]
    k_f = hull.simplices[:,2]
    return go.Mesh3d(
        x=x, y=y, z=z,
        i=i_f, j=j_f, k=k_f,
        color=color,
        opacity=opacity,
        flatshading=True,
        name=name,
        showlegend=showlegend,
        hoverinfo="name",
        legendgroup="obstacles",
        legendgrouptitle_text="Obstacles" if showlegend else None,
    )


# ─────────────────────────────────────────────
#  Wireframe (12 kenar)
# ─────────────────────────────────────────────
def wireframe_trace(mn, mx, color="white", width=1.5):
    x0,y0,z0 = mn; x1,y1,z1 = mx
    segs = [
        [(x0,y0,z0),(x1,y0,z0)], [(x1,y0,z0),(x1,y1,z0)],
        [(x1,y1,z0),(x0,y1,z0)], [(x0,y1,z0),(x0,y0,z0)],
        [(x0,y0,z1),(x1,y0,z1)], [(x1,y0,z1),(x1,y1,z1)],
        [(x1,y1,z1),(x0,y1,z1)], [(x0,y1,z1),(x0,y0,z1)],
        [(x0,y0,z0),(x0,y0,z1)], [(x1,y0,z0),(x1,y0,z1)],
        [(x1,y1,z0),(x1,y1,z1)], [(x0,y1,z0),(x0,y1,z1)],
    ]
    xs, ys, zs = [], [], []
    for (a,b) in segs:
        xs += [a[0], b[0], None]
        ys += [a[1], b[1], None]
        zs += [a[2], b[2], None]
    return go.Scatter3d(
        x=xs, y=ys, z=zs,
        mode="lines",
        line=dict(color=color, width=width),
        showlegend=False,
        hoverinfo="skip",
    )


# ─────────────────────────────────────────────
#  Renk paleti
# ─────────────────────────────────────────────
OBS_COLORS   = [
    "rgba(231,76,60,0.7)",
    "rgba(230,126,22,0.7)",
    "rgba(243,156,18,0.7)",
    "rgba(155,89,182,0.7)",
    "rgba(26,188,156,0.7)",
]
AGENT_COLORS = ["#2ecc71", "#3498db", "#f1c40f", "#e91e63"]
BG_COLOR     = "#1a1a2e"
GRID_COLOR   = "#2a2a4a"


# ─────────────────────────────────────────────
#  Ana figür
# ─────────────────────────────────────────────
def build_figure(env, vertices, edges):
    fig  = go.Figure()
    wmin = env["world_min"]
    wmax = env["world_max"]

    # ── Dünya kutusu wireframe ────────────────────────────────────────
    fig.add_trace(wireframe_trace(wmin, wmax, color="rgba(100,120,200,0.5)", width=1.5))

    # ── Engeller ─────────────────────────────────────────────────────
    for i, obs in env["obstacles"].items():
        col = OBS_COLORS[i % len(OBS_COLORS)]
        fig.add_trace(mesh_from_hull(
            obs["min"], obs["max"],
            color=col, opacity=0.7,
            name=f"Engel {i+1}",
            showlegend=True,
        ))
        fig.add_trace(wireframe_trace(obs["min"], obs["max"], color="rgba(255,255,255,0.5)"))
        # Etiket
        cx = (obs["min"][0] + obs["max"][0]) / 2
        cy = (obs["min"][1] + obs["max"][1]) / 2
        cz =  obs["max"][2] + 0.2
        fig.add_trace(go.Scatter3d(
            x=[cx], y=[cy], z=[cz],
            mode="text", text=[f"Obs {i+1}"],
            textfont=dict(color="white", size=11),
            showlegend=False, hoverinfo="skip",
        ))

    # ── Graph kenarları ───────────────────────────────────────────────
    if edges and vertices:
        ex, ey, ez = [], [], []
        for (fr, to, _) in edges:
            if fr in vertices and to in vertices:
                a, b = vertices[fr], vertices[to]
                ex += [a[0], b[0], None]
                ey += [a[1], b[1], None]
                ez += [a[2], b[2], None]
        fig.add_trace(go.Scatter3d(
            x=ex, y=ey, z=ez,
            mode="lines",
            line=dict(color="rgba(180,180,220,0.4)", width=1.5),
            name="Graph Edges",
            hoverinfo="skip",
        ))

    # ── Graph vertex'leri ─────────────────────────────────────────────
    if vertices:
        vx  = np.array(list(vertices.values()))
        ids = list(vertices.keys())
        fig.add_trace(go.Scatter3d(
            x=vx[:,0], y=vx[:,1], z=vx[:,2],
            mode="markers",
            marker=dict(size=3, color="white", opacity=0.85),
            name="Waypoints",
            text=[f"v{i}" for i in ids],
            hovertemplate="<b>%{text}</b><br>x:%{x:.2f} y:%{y:.2f} z:%{z:.2f}<extra></extra>",
        ))

    # ── Agentlar ─────────────────────────────────────────────────────
    for ag_id in sorted(set(env["starts"]) | set(env["goals"])):
        col = AGENT_COLORS[ag_id % len(AGENT_COLORS)]
        s   = env["starts"].get(ag_id)
        g   = env["goals"].get(ag_id)

        if s:
            fig.add_trace(go.Scatter3d(
                x=[s[0]], y=[s[1]], z=[s[2]],
                mode="markers+text",
                marker=dict(size=10, color=col, symbol="diamond"),
                text=[f"  S{ag_id}"],
                textfont=dict(color=col, size=12),
                textposition="middle right",
                name=f"Robot {ag_id} Start",
                hovertemplate=(
                    f"<b>Robot {ag_id} Start</b>"
                    f"<br>x:{s[0]:.2f} y:{s[1]:.2f} z:{s[2]:.2f}<extra></extra>"
                ),
            ))
        if g:
            fig.add_trace(go.Scatter3d(
                x=[g[0]], y=[g[1]], z=[g[2]],
                mode="markers+text",
                marker=dict(size=10, color=col, symbol="square"),
                text=[f"  G{ag_id}"],
                textfont=dict(color=col, size=12),
                textposition="middle right",
                name=f"Robot {ag_id} Goal",
                hovertemplate=(
                    f"<b>Robot {ag_id} Goal</b>"
                    f"<br>x:{g[0]:.2f} y:{g[1]:.2f} z:{g[2]:.2f}<extra></extra>"
                ),
            ))
        if s and g:
            fig.add_trace(go.Scatter3d(
                x=[s[0], g[0]], y=[s[1], g[1]], z=[s[2], g[2]],
                mode="lines",
                line=dict(color=col, width=2, dash="dash"),
                showlegend=False,
                hoverinfo="skip",
            ))

    # ── Layout ───────────────────────────────────────────────────────
    margin = 1.0
    fig.update_layout(
        title=dict(
            text="3D Environment — İnteraktif Görselleştirme",
            font=dict(color="white", size=16),
            x=0.5,
        ),
        paper_bgcolor=BG_COLOR,
        height=800,
        width=1200,
        hovermode="closest",
        legend=dict(
            x=1.02, y=1,
            bgcolor="rgba(30,30,60,0.85)",
            font=dict(color="white", size=10),
            bordercolor="rgba(100,100,180,0.4)",
            borderwidth=1,
        ),
        scene=dict(
            bgcolor=BG_COLOR,
            aspectmode="data",
            xaxis=dict(
                title="X (m)", gridcolor=GRID_COLOR,
                range=[wmin[0]-margin, wmax[0]+margin],
                tickfont=dict(color="white"),
            ),
            yaxis=dict(
                title="Y (m)", gridcolor=GRID_COLOR,
                range=[wmin[1]-margin, wmax[1]+margin],
                tickfont=dict(color="white"),
            ),
            zaxis=dict(
                title="Z (m)", gridcolor=GRID_COLOR,
                range=[wmin[2]-margin, wmax[2]+margin],
                tickfont=dict(color="white"),
            ),
            camera=dict(eye=dict(x=1.5, y=-1.8, z=1.2)),
        ),
    )

    return fig


# ─────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────
def main():
    base  = os.path.dirname(os.path.abspath(__file__))
    build = os.path.join(base, "build")

    env_csv  = os.path.join(build, "environment_data.csv")
    vert_csv = os.path.join(build, "vertices.csv")
    edge_csv = os.path.join(build, "edges.csv")

    if not os.path.exists(env_csv):
        print("HATA: environment_data.csv bulunamadı — önce C++ programını çalıştır.")
        return

    print("CSV dosyaları yükleniyor...")
    env      = load_environment_csv(env_csv)
    vertices = load_vertices_csv(vert_csv) if os.path.exists(vert_csv) else {}
    edges    = load_edges_csv(edge_csv)    if os.path.exists(edge_csv) else []

    print(f"  Engel  : {len(env['obstacles'])}")
    print(f"  Agent  : {len(env['starts'])}")
    print(f"  Vertex : {len(vertices)}")
    print(f"  Edge   : {len(edges)}")

    fig = build_figure(env, vertices, edges)
    out = os.path.join(base, "environment_interactive.html")
    fig.write_html(out, include_plotlyjs="cdn")
    print(f"\nKaydedildi: {out}")
    print("Tarayıcıda aç — döndür / zoom / legend'dan katman gizle.")


if __name__ == "__main__":
    main()