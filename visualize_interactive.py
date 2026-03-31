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


def load_paths_csv(path):
    """paths.csv -> {agent_id: [(x,y,z), ...]}"""
    paths = {}
    if not os.path.exists(path):
        return paths
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            agent_id = int(row["agent_id"])
            pos = (float(row["x"]), float(row["y"]), float(row["z"]))
            paths.setdefault(agent_id, []).append(pos)
    return paths


def load_hyperplanes_csv(path):
    """hyperplanes.csv -> [{'robot_id':.., 'timestep':.., 'n':.., 'd':..}, ...]"""
    planes = []
    if not os.path.exists(path):
        return planes
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            # ellipsoid_offset'in C++ tarafında hesaplanıp CSV'ye eklendiğini varsayıyoruz.
            planes.append({
                "robot_id": int(row["robot_id"]),
                "timestep": int(row["timestep"]),
                "n": np.array([float(row["nx"]), float(row["ny"]), float(row["nz"])]),
                "d": float(row["d"]),
                "ellipsoid_offset": float(row["ellipsoid_offset"]) if "ellipsoid_offset" in row else 0.0,
                "separated_from_type": int(row["separated_from_type"]) if "separated_from_type" in row else -1,
                "separated_from_id": int(row["separated_from_id"]) if "separated_from_id" in row else -1,
            })
    return planes
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
#  ConvexHull'dan Mesh3d trace
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
def wireframe_trace(mn, mx, color="gray", width=1.5):
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
#  Hyperplane (n,d) → Surface trace
# ─────────────────────────────────────────────
def hyperplane_trace(plane_data, name, scene_center=None, size=2.0, color='cyan', opacity=0.3, visible=False):
    n = plane_data['n']
    d_raw = plane_data['d']
    offset = plane_data.get('ellipsoid_offset', 0.0)

    # Ham SVM düzlemi: n^T x = d_raw
    # Robota teğet olan gerçek kısıt düzlemi: n^T x = d_raw - offset
    d = d_raw - offset

    # Sahne merkezini düzlem üzerine iz düşür → robotlara yakın merkez
    if scene_center is None:
        scene_center = np.zeros(3)
    p0 = scene_center - (np.dot(n, scene_center) - d) * n  # düzlem üzerindeki en yakın nokta


    # Düzlem için bir ortonormal baz oluştur
    if abs(n[0]) < 0.9:
        a = np.array([1, 0, 0])
    else:
        a = np.array([0, 1, 0])

    b1 = np.cross(n, a)
    b1 = b1 / np.linalg.norm(b1)
    b2 = np.cross(n, b1)

    # Bir grid oluştur
    u = np.linspace(-size, size, 5)
    v = np.linspace(-size, size, 5)
    u, v = np.meshgrid(u, v)

    # Düzlem üzerindeki noktaları hesapla: P(u,v) = p0 + u*b1 + v*b2
    x = p0[0] + u * b1[0] + v * b2[0]
    y = p0[1] + u * b1[1] + v * b2[1]
    z = p0[2] + u * b1[2] + v * b2[2]

    return go.Surface(
        x=x, y=y, z=z,
        colorscale=[[0, color], [1, color]],
        showscale=False,
        opacity=opacity,
        hovertemplate=f"<b>{name}</b><extra></extra>",
        visible=visible,
        lighting=dict(ambient=0.8, diffuse=0.2)
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

BG_COLOR   = "white"
GRID_COLOR = "rgba(200,200,200,0.3)"


# ─────────────────────────────────────────────
#  Eksen ayarları — grid çizgisi yok, tick yok
# ─────────────────────────────────────────────
def axis_style(title, axis_range):
    return dict(
        title=dict(
            text=title,
            font=dict(color="black")
        ),
        range=axis_range,
        showgrid=False,        # ızgara çizgileri kaldırıldı
        showticklabels=False,  # sayısal aralıklandırma kaldırıldı
        zeroline=False,
        showspikes=False,
        backgroundcolor=BG_COLOR,
        gridcolor=GRID_COLOR,
        tickfont=dict(color="black")
    )


# ─────────────────────────────────────────────
#  Ana figür
# ─────────────────────────────────────────────
def build_figure(env, vertices, edges, paths, hyperplanes):
    fig  = go.Figure()
    wmin = np.array(env["world_min"])
    wmax = np.array(env["world_max"])
    scene_center = (wmin + wmax) / 2.0


    # ── Dünya kutusu wireframe ────────────────────────────────────────
    fig.add_trace(wireframe_trace(wmin, wmax, color="rgba(80,80,80,0.6)", width=1.5))

    # ── Engeller ─────────────────────────────────────────────────────
    for i, obs in env["obstacles"].items():
        col = OBS_COLORS[i % len(OBS_COLORS)]
        fig.add_trace(mesh_from_hull(
            obs["min"], obs["max"],
            color=col, opacity=0.7,
            name=f"Engel {i+1}",
            showlegend=True,
        ))
        fig.add_trace(wireframe_trace(obs["min"], obs["max"], color="rgba(50,50,50,0.6)"))
        cx = (obs["min"][0] + obs["max"][0]) / 2
        cy = (obs["min"][1] + obs["max"][1]) / 2
        cz =  obs["max"][2] + 0.2
        fig.add_trace(go.Scatter3d(
            x=[cx], y=[cy], z=[cz],
            mode="text", text=[f"Obs {i+1}"],
            textfont=dict(color="black", size=11),
            showlegend=False, hoverinfo="skip",
        ))

    # ── Graph kenarları — siyah ───────────────────────────────────────
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
            line=dict(color="black", width=1.5),
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
            marker=dict(size=3, color="black", opacity=0.85),
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

    # ── Agent Yolları (Paths) ────────────────────────────────────────
    for agent_id, path_points in paths.items():
        if not path_points:
            continue
        col = AGENT_COLORS[agent_id % len(AGENT_COLORS)]
        path_np = np.array(path_points)
        fig.add_trace(go.Scatter3d(
            x=path_np[:, 0], y=path_np[:, 1], z=path_np[:, 2],
            mode="lines+markers",
            line=dict(color=col, width=5),
            marker=dict(size=2.5, color=col),
            name=f"Robot {agent_id} Yolu",
            hovertemplate=(
                f"<b>Robot {agent_id} Yolu (t=%{{customdata[0]}})</b>"
                f"<br>x:%{{x:.2f}} y:%{{y:.2f}} z:%{{z:.2f}}<extra></extra>"
            ),
            customdata=np.arange(len(path_points)).reshape(-1,1)
        ))

    # ── Hiper Düzlemler (Hyperplanes) ────────────────────────────────
    # Bu liste, her bir hiper düzlemin robot_id ve timestep bilgilerini saklar.
    # Bu sayede filtreleme butonları için görünürlük listeleri oluşturulabilir.
    plane_properties = []
    if hyperplanes:
        is_first_plane = True # Legend'da tek bir girdi olması için
        for p_data in hyperplanes:
            # Varsayılan olarak tüm düzlemleri gizli başlat
            is_visible = False

            # Düzlem için açıklayıcı bir isim oluştur
            from_type = p_data.get('separated_from_type', -1)
            from_id = p_data.get('separated_from_id', -1)
            opponent_str = ""
            if from_type == 0: # Robot
                opponent_str = f"-R{from_id}"
            elif from_type == 1: # Obstacle
                # Engel ID'leri 0'dan başladığı için görselde 1'den başlatıyoruz
                opponent_str = f"-Obs{from_id + 1}"
            
            # Hover text için kullanılacak bireysel isim
            individual_plane_name = f"R{p_data['robot_id']}{opponent_str} T{p_data['timestep']}"

            trace = hyperplane_trace(p_data, name=individual_plane_name, scene_center=scene_center, size=10.0,
                color='rgba(142, 68, 173, 0.5)', # Mor tonu (varsayılan)
                opacity=0.4,
                visible=is_visible
            )

            # Legend'da gruplama için ayarlar
            trace.name = "Hyperplanes"
            trace.legendgroup = "hyperplanes"
            trace.showlegend = is_first_plane
            fig.add_trace(trace)
            is_first_plane = False

            # Filtreleme için düzlem özelliklerini sakla
            plane_properties.append({'robot_id': p_data['robot_id'], 'timestep': p_data['timestep']})

    # ── İnteraktif Dropdown Menü (Hiper düzlemler için) ───────────────────
    updatemenus = None
    if hyperplanes:
        # Sadece hiper düzlem izlerinin indekslerini al
        surface_indices = [i for i, trace in enumerate(fig.data) if isinstance(trace, go.Surface)]

        buttons = []

        # Genel kontrol butonları
        buttons.append(dict(label="Düzlemleri Gizle",
                             method="restyle",
                             args=[{"visible": [False] * len(surface_indices)}, surface_indices]))
        buttons.append(dict(label="Tüm Düzlemleri Göster",
                             method="restyle",
                             args=[{"visible": [True] * len(surface_indices)}, surface_indices]))
        
        # Timestep'e göre filtreleme butonları
        all_timesteps = sorted(list(set(p['timestep'] for p in plane_properties)))
        for t in all_timesteps:
            visibility = [p['timestep'] == t for p in plane_properties]
            buttons.append(dict(label=f"Sadece Timestep T={t}",
                                 method="restyle",
                                 args=[{"visible": visibility}, surface_indices]))
        
        # Robot ID'ye göre filtreleme butonları
        all_robot_ids = sorted(list(set(p['robot_id'] for p in plane_properties)))
        for r_id in all_robot_ids:
            visibility = [p['robot_id'] == r_id for p in plane_properties]
            buttons.append(dict(label=f"Sadece Robot R{r_id}",
                                 method="restyle",
                                 args=[{"visible": visibility}, surface_indices]))

        # Robot ID ve Timestep'e göre kombine filtreleme butonları
        unique_combinations = sorted(list(set((p['robot_id'], p['timestep']) for p in plane_properties)))
        for r_id, t in unique_combinations:
            visibility = [(p['robot_id'] == r_id and p['timestep'] == t) for p in plane_properties]
            buttons.append(dict(label=f"Sadece Robot R{r_id}, Timestep T={t}",
                                 method="restyle",
                                 args=[{"visible": visibility}, surface_indices]))

        # Artık tüm düzlemler varsayılan olarak gizli olduğu için ilk buton ("Gizle") aktif olmalı.
        active_button_index = 0
        updatemenus = [
            dict(
                type="dropdown",
                direction="down",
                active=active_button_index,
                x=0.0, xanchor="left",
                y=1.0, yanchor="top",
                pad={"r": 10, "t": 10},
                buttons=buttons,
                showactive=True,
            )
        ]

    # ── Layout ───────────────────────────────────────────────────────
    margin = 1.0
    fig.update_layout(
        title=dict(
            text="3D Environment — İnteraktif Görselleştirme",
            font=dict(color="black", size=16),
            x=0.5,
        ),
        paper_bgcolor=BG_COLOR,
        height=800,
        width=1200,
        hovermode="closest",
        legend=dict(
            x=1.02, y=1,
            bgcolor="rgba(240,240,240,0.9)",
            font=dict(color="black", size=10),
            bordercolor="rgba(100,100,100,0.4)",
            borderwidth=1,
        ),
        scene=dict(
            bgcolor=BG_COLOR,
            aspectmode="data",
            xaxis=axis_style("X (m)", [wmin[0]-margin, wmax[0]+margin]),
            yaxis=axis_style("Y (m)", [wmin[1]-margin, wmax[1]+margin]),
            zaxis=axis_style("Z (m)", [wmin[2]-margin, wmax[2]+margin]),
            camera=dict(eye=dict(x=1.5, y=-1.8, z=1.2)),
        ),
        updatemenus=updatemenus
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
    paths_csv = os.path.join(build, "paths.csv")
    hyperplanes_csv = os.path.join(build, "hyperplanes.csv")

    if not os.path.exists(env_csv):
        print("HATA: environment_data.csv bulunamadı — önce C++ programını çalıştır.")
        return

    print("CSV dosyaları yükleniyor...")
    env      = load_environment_csv(env_csv) 
    vertices = load_vertices_csv(vert_csv) if os.path.exists(vert_csv) else {} 
    edges    = load_edges_csv(edge_csv)    if os.path.exists(edge_csv) else [] 
    paths    = load_paths_csv(paths_csv)
    hyperplanes = load_hyperplanes_csv(hyperplanes_csv)

    print(f"  Engel  : {len(env['obstacles'])}")
    print(f"  Agent  : {len(env['starts'])}")
    print(f"  Vertex : {len(vertices)}")
    print(f"  Edge   : {len(edges)}")
    if hyperplanes:
        print(f"  Düzlem : {len(hyperplanes)}")
    if paths:
        print(f"  Yol    : {len(paths)}")

    fig = build_figure(env, vertices, edges, paths, hyperplanes)
    out_filename = "solution_with_planes.html" if paths else "environment_interactive.html"
    out = os.path.join(base, out_filename)
    fig.write_html(out, include_plotlyjs="cdn")

    print(f"\nKaydedildi: {out}")
    print("Tarayıcıda aç — döndür / zoom / legend'dan katman gizle.")


if __name__ == "__main__":
    main()