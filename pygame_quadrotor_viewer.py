import argparse
import csv
import math
import os
from collections import defaultdict, deque

import pygame


WIDTH, HEIGHT = 1200, 800
BACKGROUND = (248, 249, 251)
BLACK = (20, 24, 31)
GRAY = (120, 130, 145)
LIGHT_GRAY = (210, 216, 225)
OBSTACLE_FILL = (220, 92, 75)
OBSTACLE_EDGE = (120, 50, 45)
WORLD_EDGE = (70, 78, 90)
START_COLOR = (45, 180, 90)
GOAL_COLOR = (225, 165, 45)
AGENT_COLORS = [
    (46, 204, 113),
    (52, 152, 219),
    (241, 196, 15),
    (233, 30, 99),
    (155, 89, 182),
    (26, 188, 156),
    (230, 126, 34),
    (52, 73, 94),
]


def load_environment(path):
    env = {
        "world_min": None,
        "world_max": None,
        "obstacles": defaultdict(dict),
        "starts": {},
        "goals": {},
    }

    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            item_type = row["type"]
            point = (float(row["x"]), float(row["y"]), float(row["z"]))
            index = int(row["extra"])

            if item_type == "world_min":
                env["world_min"] = point
            elif item_type == "world_max":
                env["world_max"] = point
            elif item_type == "obs_min":
                env["obstacles"][index]["min"] = point
            elif item_type == "obs_max":
                env["obstacles"][index]["max"] = point
            elif item_type == "start":
                env["starts"][index] = point
            elif item_type == "goal":
                env["goals"][index] = point

    env["obstacles"] = dict(env["obstacles"])
    return env


def load_control_points(path):
    pieces = defaultdict(lambda: defaultdict(list))

    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            agent_id = int(row["agent_id"])
            piece_id = int(row["piece_id"])
            point_index = int(row["point_index_in_piece"])
            point = (float(row["x"]), float(row["y"]), float(row["z"]))
            pieces[agent_id][piece_id].append((point_index, point))

    trajectories = {}
    for agent_id, piece_map in pieces.items():
        ordered_pieces = []
        for piece_id in sorted(piece_map):
            ordered = [p for _, p in sorted(piece_map[piece_id], key=lambda item: item[0])]
            if ordered:
                ordered_pieces.append(ordered)
        trajectories[agent_id] = ordered_pieces

    return trajectories


def bezier_point(control_points, t):
    points = [list(p) for p in control_points]
    degree = len(points) - 1

    for r in range(1, degree + 1):
        for i in range(degree - r + 1):
            points[i][0] = (1.0 - t) * points[i][0] + t * points[i + 1][0]
            points[i][1] = (1.0 - t) * points[i][1] + t * points[i + 1][1]
            points[i][2] = (1.0 - t) * points[i][2] + t * points[i + 1][2]

    return tuple(points[0])


def sample_trajectories(trajectories, samples_per_piece):
    sampled = {}

    for agent_id, pieces in trajectories.items():
        points = []
        for piece_index, control_points in enumerate(pieces):
            for sample_index in range(samples_per_piece):
                if piece_index > 0 and sample_index == 0:
                    continue
                t = sample_index / float(samples_per_piece - 1)
                points.append(bezier_point(control_points, t))
        sampled[agent_id] = points

    return sampled


def aabb_vertices(mn, mx):
    x0, y0, z0 = mn
    x1, y1, z1 = mx
    return [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
    ]


BOX_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]


def rotate_point(point, yaw, pitch, center):
    x, y, z = point[0] - center[0], point[1] - center[1], point[2] - center[2]

    cosy, siny = math.cos(yaw), math.sin(yaw)
    x, y = x * cosy - y * siny, x * siny + y * cosy

    cosp, sinp = math.cos(pitch), math.sin(pitch)
    y, z = y * cosp - z * sinp, y * sinp + z * cosp

    return x, y, z


def project(point, camera):
    rx, ry, rz = rotate_point(point, camera["yaw"], camera["pitch"], camera["center"])
    scale = camera["scale"] * camera["zoom"]

    sx = WIDTH / 2 + rx * scale + camera["pan"][0]
    sy = HEIGHT / 2 - rz * scale + ry * scale * 0.35 + camera["pan"][1]
    depth = ry
    return int(sx), int(sy), depth


def draw_box(surface, mn, mx, camera, edge_color, fill_color=None):
    vertices = aabb_vertices(mn, mx)
    projected = [project(v, camera) for v in vertices]

    if fill_color:
        faces = [
            (0, 1, 2, 3), (4, 5, 6, 7),
            (0, 1, 5, 4), (2, 3, 7, 6),
            (1, 2, 6, 5), (0, 3, 7, 4),
        ]
        face_items = []
        for face in faces:
            avg_depth = sum(projected[i][2] for i in face) / len(face)
            points = [(projected[i][0], projected[i][1]) for i in face]
            face_items.append((avg_depth, points))
        for _, points in sorted(face_items):
            pygame.draw.polygon(surface, fill_color, points)

    for a, b in BOX_EDGES:
        pygame.draw.line(
            surface,
            edge_color,
            (projected[a][0], projected[a][1]),
            (projected[b][0], projected[b][1]),
            1,
        )


def draw_polyline(surface, points, camera, color, width=2):
    if len(points) < 2:
        return

    screen_points = [(project(p, camera)[0], project(p, camera)[1]) for p in points]
    pygame.draw.lines(surface, color, False, screen_points, width)


def draw_marker(surface, point, camera, color, radius, label=None, font=None):
    sx, sy, _ = project(point, camera)
    pygame.draw.circle(surface, color, (sx, sy), radius)
    pygame.draw.circle(surface, BLACK, (sx, sy), radius, 1)

    if label and font:
        text = font.render(label, True, BLACK)
        surface.blit(text, (sx + radius + 4, sy - radius - 4))


def get_world_camera(env):
    wmin = env["world_min"]
    wmax = env["world_max"]
    center = (
        (wmin[0] + wmax[0]) / 2.0,
        (wmin[1] + wmax[1]) / 2.0,
        (wmin[2] + wmax[2]) / 2.0,
    )
    span_x = max(1e-6, wmax[0] - wmin[0])
    span_y = max(1e-6, wmax[1] - wmin[1])
    span_z = max(1e-6, wmax[2] - wmin[2])
    max_span = max(span_x, span_y, span_z)

    return {
        "center": center,
        "scale": min(WIDTH, HEIGHT) * 0.65 / max_span,
        "zoom": 1.0,
        "yaw": math.radians(-35),
        "pitch": math.radians(58),
        "pan": [0, 25],
    }


def draw_hud(surface, font, agent_count, speed, paused):
    lines = [
        "Pygame Quadrotor Viewer",
        f"Agents: {agent_count}   Speed: {speed:.1f}x   {'Paused' if paused else 'Playing'}",
        "Space: pause | R: reset | +/-: speed | Arrow keys: rotate | Mouse wheel: zoom | Drag: pan",
    ]
    y = 12
    for line in lines:
        text = font.render(line, True, BLACK)
        surface.blit(text, (12, y))
        y += 22


def run_viewer(env_path, control_points_path, samples_per_piece, speed):
    env = load_environment(env_path)
    control_points = load_control_points(control_points_path)
    sampled = sample_trajectories(control_points, samples_per_piece)

    if not sampled:
        raise RuntimeError("No trajectories found in control_points.csv")

    total_frames = max(len(points) for points in sampled.values())
    trails = {agent_id: deque(maxlen=120) for agent_id in sampled}

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Pygame Quadrotor Viewer")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 16)
    camera = get_world_camera(env)

    frame = 0.0
    paused = False
    dragging = False
    last_mouse = None
    running = True

    while running:
        dt = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    frame = 0.0
                    for trail in trails.values():
                        trail.clear()
                elif event.key in (pygame.K_EQUALS, pygame.K_PLUS, pygame.K_KP_PLUS):
                    speed = min(20.0, speed * 1.25)
                elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    speed = max(0.1, speed / 1.25)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    dragging = True
                    last_mouse = event.pos
                elif event.button == 4:
                    camera["zoom"] = min(5.0, camera["zoom"] * 1.1)
                elif event.button == 5:
                    camera["zoom"] = max(0.2, camera["zoom"] / 1.1)
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    dragging = False
                    last_mouse = None
            elif event.type == pygame.MOUSEMOTION and dragging and last_mouse:
                dx = event.pos[0] - last_mouse[0]
                dy = event.pos[1] - last_mouse[1]
                camera["pan"][0] += dx
                camera["pan"][1] += dy
                last_mouse = event.pos

        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            camera["yaw"] -= 1.8 * dt
        if keys[pygame.K_RIGHT]:
            camera["yaw"] += 1.8 * dt
        if keys[pygame.K_UP]:
            camera["pitch"] = min(math.radians(85), camera["pitch"] + 1.8 * dt)
        if keys[pygame.K_DOWN]:
            camera["pitch"] = max(math.radians(10), camera["pitch"] - 1.8 * dt)

        if not paused:
            frame = (frame + speed * 30.0 * dt) % total_frames

        frame_index = int(frame)

        screen.fill(BACKGROUND)

        draw_box(screen, env["world_min"], env["world_max"], camera, WORLD_EDGE)

        for obs in env["obstacles"].values():
            draw_box(screen, obs["min"], obs["max"], camera, OBSTACLE_EDGE, OBSTACLE_FILL)

        for agent_id, start in env["starts"].items():
            draw_marker(screen, start, camera, START_COLOR, 5, f"S{agent_id}", font)
        for goal_id, goal in env["goals"].items():
            draw_marker(screen, goal, camera, GOAL_COLOR, 5, f"G{goal_id}", font)

        agent_draw_items = []
        for agent_id, points in sampled.items():
            point = points[min(frame_index, len(points) - 1)]
            trails[agent_id].append(point)
            _, _, depth = project(point, camera)
            agent_draw_items.append((depth, agent_id, point))

            color = AGENT_COLORS[agent_id % len(AGENT_COLORS)]
            draw_polyline(screen, points, camera, tuple(max(0, c - 70) for c in color), 1)
            draw_polyline(screen, list(trails[agent_id]), camera, color, 3)

        for _, agent_id, point in sorted(agent_draw_items):
            color = AGENT_COLORS[agent_id % len(AGENT_COLORS)]
            draw_marker(screen, point, camera, color, 9, f"R{agent_id}", font)

        progress_width = 320
        pygame.draw.rect(screen, LIGHT_GRAY, (12, HEIGHT - 26, progress_width, 8))
        progress = frame_index / max(1, total_frames - 1)
        pygame.draw.rect(screen, BLACK, (12, HEIGHT - 26, int(progress_width * progress), 8))

        draw_hud(screen, font, len(sampled), speed, paused)
        pygame.display.flip()

    pygame.quit()


def parse_args():
    parser = argparse.ArgumentParser(description="Simple pygame viewer for quadrotor Bezier trajectories.")
    parser.add_argument("--env", default="build/environment_data.csv", help="Path to environment_data.csv")
    parser.add_argument("--control-points", default="build/control_points.csv", help="Path to control_points.csv")
    parser.add_argument("--samples-per-piece", type=int, default=20, help="Bezier samples per trajectory piece")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    return parser.parse_args()

def main():
    args = parse_args()
    env_path = os.path.abspath(args.env)
    control_points_path = os.path.abspath(args.control_points)

    if not os.path.exists(env_path):
        raise FileNotFoundError(f"Environment CSV not found: {env_path}")
    if not os.path.exists(control_points_path):
        raise FileNotFoundError(f"Control points CSV not found: {control_points_path}")

    run_viewer(env_path, control_points_path, args.samples_per_piece, args.speed)


if __name__ == "__main__":
    main()
