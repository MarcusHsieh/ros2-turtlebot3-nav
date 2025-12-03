"""Utilities for loading and processing occupancy grid maps."""

import os
import yaml
import numpy as np
from PIL import Image
from typing import Tuple, Optional

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose


class MapLoader:
    """
    Load and process occupancy grid maps from YAML/PGM files.

    The standard ROS map format consists of:
    - A YAML file with metadata (resolution, origin, thresholds)
    - A PGM image file with occupancy data

    PGM pixel values:
    - 255 (white): Free space
    - 0 (black): Occupied
    - 205 (gray): Unknown

    Attributes:
        map_metadata: Dictionary of map metadata from YAML
        image: Raw image data as numpy array
        resolution: Map resolution in meters per cell
        origin: Map origin as [x, y, theta]
        width: Map width in cells
        height: Map height in cells
    """

    def __init__(self, yaml_path: str):
        """
        Load a map from YAML metadata file.

        Args:
            yaml_path: Path to the map YAML file

        Raises:
            FileNotFoundError: If YAML or image file not found
            ValueError: If YAML format is invalid
        """
        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"Map YAML not found: {yaml_path}")

        with open(yaml_path, 'r') as f:
            self.map_metadata = yaml.safe_load(f)

        # Load the image
        map_dir = os.path.dirname(yaml_path)
        image_name = self.map_metadata.get('image', '')
        image_path = os.path.join(map_dir, image_name)

        if not os.path.exists(image_path):
            raise FileNotFoundError(f"Map image not found: {image_path}")

        # Load as grayscale
        self.image = np.array(Image.open(image_path).convert('L'))

        # Extract metadata
        self.resolution = self.map_metadata.get('resolution', 0.05)
        self.origin = self.map_metadata.get('origin', [0.0, 0.0, 0.0])
        self.negate = self.map_metadata.get('negate', 0)
        self.occupied_thresh = self.map_metadata.get('occupied_thresh', 0.65)
        self.free_thresh = self.map_metadata.get('free_thresh', 0.196)

        self.height, self.width = self.image.shape

        print(f"Loaded map: {self.width}x{self.height} cells, "
              f"resolution: {self.resolution}m/cell")

    def to_occupancy_grid(self, frame_id: str = 'map') -> OccupancyGrid:
        """
        Convert loaded map to ROS2 OccupancyGrid message.

        Args:
            frame_id: Frame ID for the map header

        Returns:
            OccupancyGrid message
        """
        grid = OccupancyGrid()
        grid.header.frame_id = frame_id

        # Set map metadata
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.origin[0]
        grid.info.origin.position.y = self.origin[1]
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # Convert image to occupancy values
        # Note: Image rows are top-to-bottom, but occupancy grid is bottom-to-top
        data = []
        occupied_pixel = int(255 * (1 - self.occupied_thresh))
        free_pixel = int(255 * (1 - self.free_thresh))

        for row in reversed(self.image):  # Flip Y axis
            for pixel in row:
                if self.negate:
                    pixel = 255 - pixel

                if pixel > free_pixel:  # Higher value = more free
                    data.append(0)  # Free
                elif pixel < occupied_pixel:  # Lower value = more occupied
                    data.append(100)  # Occupied
                else:
                    data.append(-1)  # Unknown

        grid.data = data
        return grid

    def get_grid_array(self, inflate_radius: float = 0.0) -> np.ndarray:
        """
        Get the occupancy grid as a numpy boolean array.

        Args:
            inflate_radius: Radius (in meters) to inflate obstacles

        Returns:
            2D boolean array (True = obstacle, False = free)
        """
        occupied_pixel = int(255 * (1 - self.occupied_thresh))

        # Create binary grid
        if self.negate:
            grid = (255 - self.image) < occupied_pixel
        else:
            grid = self.image < occupied_pixel

        # Flip Y axis to match ROS convention
        grid = np.flipud(grid)

        # Inflate obstacles if requested
        if inflate_radius > 0:
            grid = self._inflate_obstacles(grid, inflate_radius)

        return grid

    def _inflate_obstacles(
        self,
        grid: np.ndarray,
        radius: float
    ) -> np.ndarray:
        """
        Inflate obstacles by a given radius.

        Uses morphological dilation to expand obstacle regions.

        Args:
            grid: Binary occupancy grid
            radius: Inflation radius in meters

        Returns:
            Inflated grid
        """
        from scipy import ndimage

        # Convert radius to cells
        radius_cells = int(np.ceil(radius / self.resolution))

        # Create circular structuring element
        y, x = np.ogrid[-radius_cells:radius_cells+1, -radius_cells:radius_cells+1]
        struct = x*x + y*y <= radius_cells*radius_cells

        # Dilate obstacles
        inflated = ndimage.binary_dilation(grid, structure=struct)

        return inflated

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid coordinates.

        Args:
            x: World x coordinate (meters)
            y: World y coordinate (meters)

        Returns:
            Grid position as (row, col)
        """
        col = int((x - self.origin[0]) / self.resolution)
        row = int((y - self.origin[1]) / self.resolution)
        return (row, col)

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """
        Convert grid coordinates to world coordinates.

        Args:
            row: Grid row
            col: Grid column

        Returns:
            World position as (x, y)
        """
        x = self.origin[0] + (col + 0.5) * self.resolution
        y = self.origin[1] + (row + 0.5) * self.resolution
        return (x, y)

    def is_valid(self, row: int, col: int) -> bool:
        """
        Check if grid coordinates are within bounds.

        Args:
            row: Grid row
            col: Grid column

        Returns:
            True if within bounds
        """
        return 0 <= row < self.height and 0 <= col < self.width


def occupancy_grid_to_array(msg: OccupancyGrid) -> Tuple[np.ndarray, dict]:
    """
    Convert a ROS2 OccupancyGrid message to numpy array.

    Args:
        msg: OccupancyGrid message

    Returns:
        Tuple of (grid array, metadata dict)
    """
    # Reshape data to 2D array
    data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    # Convert to binary: True = obstacle (value > 50), False = free
    # Unknown (-1) is treated as obstacle for safety
    grid = (data > 50) | (data < 0)

    metadata = {
        'resolution': msg.info.resolution,
        'origin': [
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            0.0
        ],
        'width': msg.info.width,
        'height': msg.info.height
    }

    return grid, metadata
