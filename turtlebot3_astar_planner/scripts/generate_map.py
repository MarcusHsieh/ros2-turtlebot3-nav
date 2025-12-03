#!/usr/bin/env python3
"""
Generate a static occupancy grid map for TurtleBot3 hexagon world.

This script creates a PGM map file based on the known obstacle positions
from the turtlebot3_world Gazebo model.
"""

import numpy as np
from PIL import Image
import os

# Map parameters
RESOLUTION = 0.05  # meters per pixel
MAP_SIZE_X = 8.0   # meters (total width)
MAP_SIZE_Y = 8.0   # meters (total height)
ORIGIN_X = -4.0    # map origin x
ORIGIN_Y = -4.0    # map origin y

# Calculate dimensions
WIDTH = int(MAP_SIZE_X / RESOLUTION)   # 160 pixels
HEIGHT = int(MAP_SIZE_Y / RESOLUTION)  # 160 pixels

# Obstacle inflation radius (robot radius + safety margin)
INFLATE_RADIUS = 0.15  # meters

def world_to_pixel(x, y):
    """Convert world coordinates to pixel coordinates."""
    px = int((x - ORIGIN_X) / RESOLUTION)
    py = int((y - ORIGIN_Y) / RESOLUTION)
    return px, py

def draw_circle(grid, cx, cy, radius):
    """Draw a filled circle on the grid."""
    radius_px = int(radius / RESOLUTION)
    px, py = world_to_pixel(cx, cy)

    for dx in range(-radius_px - 1, radius_px + 2):
        for dy in range(-radius_px - 1, radius_px + 2):
            if dx*dx + dy*dy <= radius_px*radius_px:
                nx, ny = px + dx, py + dy
                if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
                    grid[ny, nx] = 0  # 0 = occupied (black)

def draw_hexagon(grid, cx, cy, scale):
    """Draw a hexagonal obstacle on the grid."""
    # Approximate hexagon as a circle with radius based on scale
    # The hexagon.dae mesh has approximate radius of 1.0m at scale 1.0
    radius = scale * 1.0
    draw_circle(grid, cx, cy, radius + INFLATE_RADIUS)

def draw_hexagonal_wall(grid):
    """Draw the outer hexagonal wall boundary."""
    # The wall is approximately a hexagon with vertices at ~4m from center
    # We'll draw it as thick line segments
    wall_radius = 3.8  # approximate outer wall radius
    wall_thickness = 0.1  # wall thickness

    # Hexagon vertices (6 sides)
    angles = np.linspace(0, 2*np.pi, 7)[:-1]  # 6 angles
    vertices = [(wall_radius * np.cos(a), wall_radius * np.sin(a)) for a in angles]

    # Draw each wall segment
    for i in range(6):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % 6]
        draw_thick_line(grid, x1, y1, x2, y2, wall_thickness + INFLATE_RADIUS)

def draw_thick_line(grid, x1, y1, x2, y2, thickness):
    """Draw a thick line between two points."""
    # Number of points along the line
    length = np.sqrt((x2-x1)**2 + (y2-y1)**2)
    num_points = int(length / (RESOLUTION * 0.5)) + 1

    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        draw_circle(grid, x, y, thickness)

def main():
    # Create grid (255 = free/white, 0 = occupied/black)
    grid = np.ones((HEIGHT, WIDTH), dtype=np.uint8) * 255

    # Draw 9 cylindrical obstacles (3x3 grid)
    cylinder_radius = 0.15
    cylinder_positions = [
        (-1.1, -1.1), (-1.1, 0), (-1.1, 1.1),
        (0, -1.1), (0, 0), (0, 1.1),
        (1.1, -1.1), (1.1, 0), (1.1, 1.1)
    ]

    for x, y in cylinder_positions:
        draw_circle(grid, x, y, cylinder_radius + INFLATE_RADIUS)

    # Draw hexagonal obstacles
    # Head (large): scale 0.8 at (3.5, 0)
    draw_hexagon(grid, 3.5, 0, 0.8)

    # Smaller hexagons: scale 0.55
    hex_positions = [
        (1.8, 2.7),   # left_hand
        (1.8, -2.7),  # right_hand
        (-1.8, 2.7),  # left_foot
        (-1.8, -2.7)  # right_foot
    ]
    for x, y in hex_positions:
        draw_hexagon(grid, x, y, 0.55)

    # Draw hexagonal wall boundary
    draw_hexagonal_wall(grid)

    # Flip vertically (PGM origin is top-left, ROS map origin is bottom-left)
    grid = np.flipud(grid)

    # Save as PGM
    script_dir = os.path.dirname(os.path.abspath(__file__))
    maps_dir = os.path.join(script_dir, '..', 'maps')
    os.makedirs(maps_dir, exist_ok=True)

    pgm_path = os.path.join(maps_dir, 'turtlebot3_world.pgm')
    yaml_path = os.path.join(maps_dir, 'turtlebot3_world.yaml')

    # Save PGM
    img = Image.fromarray(grid, mode='L')
    img.save(pgm_path)
    print(f"Saved map image to: {pgm_path}")
    print(f"Map size: {WIDTH}x{HEIGHT} pixels ({MAP_SIZE_X}x{MAP_SIZE_Y} meters)")

    # Save YAML metadata
    yaml_content = f"""image: turtlebot3_world.pgm
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""

    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    print(f"Saved map metadata to: {yaml_path}")

if __name__ == '__main__':
    main()
