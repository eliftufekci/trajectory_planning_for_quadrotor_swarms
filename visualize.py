import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

def draw_box(ax, min_c, max_c, color='red', alpha=0.3):
    """3D kutu çiz (engel için)"""
    x0, y0, z0 = min_c
    x1, y1, z1 = max_c

    # 6 yüz
    faces = [
        [[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0]],  # alt
        [[x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]],  # üst
        [[x0,y0,z0],[x1,y0,z0],[x1,y0,z1],[x0,y0,z1]],  # ön
        [[x0,y1,z0],[x1,y1,z0],[x1,y1,z1],[x0,y1,z1]],  # arka
        [[x0,y0,z0],[x0,y1,z0],[x0,y1,z1],[x0,y0,z1]],  # sol
        [[x1,y0,z0],[x1,y1,z0],[x1,y1,z1],[x1,y0,z1]],  # sağ
    ]
    poly = Poly3DCollection(faces, alpha=alpha, facecolor=color, edgecolor='darkred', linewidth=0.3)
    ax.add_collection3d(poly)

# Verileri oku
vertices = pd.read_csv('vertices.csv')
edges    = pd.read_csv('edges.csv')
obstacles = pd.read_csv('obstacles.csv')
sg       = pd.read_csv('start_goals.csv')

print(f"Vertex sayısı: {len(vertices)}")
print(f"Edge sayısı:   {len(edges)}")

fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111, projection='3d')

# --- Engelleri çiz ---
for _, obs in obstacles.iterrows():
    draw_box(ax,
             [obs.min_x, obs.min_y, obs.min_z],
             [obs.max_x, obs.max_y, obs.max_z])

# --- Edge'leri çiz (önce — vertex'lerin altında kalsın) ---
for _, e in edges.iterrows():
    ax.plot([e.from_x, e.to_x],
            [e.from_y, e.to_y],
            [e.from_z, e.to_z],
            color='lightblue', alpha=0.4, linewidth=0.5)

# --- Vertex'leri çiz ---
ax.scatter(vertices.x, vertices.y, vertices.z,
           c='steelblue', s=8, alpha=0.6, label='Vertices')

# --- Start ve goal'ları çiz ---
starts = sg[sg.type == 'start']
goals  = sg[sg.type == 'goal']
ax.scatter(starts.x, starts.y, starts.z,
           c='green', s=100, marker='^', zorder=5, label='Start')
ax.scatter(goals.x, goals.y, goals.z,
           c='red', s=100, marker='*', zorder=5, label='Goal')

# Etiket ekle
for _, s in starts.iterrows():
    ax.text(s.x, s.y, s.z + 0.5, 'S', color='green', fontsize=9)
for _, g in goals.iterrows():
    ax.text(g.x, g.y, g.z + 0.5, 'G', color='red', fontsize=9)

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title(f'Grid Roadmap — {len(vertices)} vertices, {len(edges)} edges')
ax.legend()
ax.set_box_aspect([32, 32, 8])

plt.tight_layout()
plt.savefig('roadmap.png', dpi=150, bbox_inches='tight')
plt.show()
print("roadmap.png kaydedildi")