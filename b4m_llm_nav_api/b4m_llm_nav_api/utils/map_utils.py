#!/usr/bin/env python3
"""
Map Utilities for B4M LLM Navigation API

Provides functions for processing occupancy grids, converting between
coordinate systems, and analyzing spatial data for LLM consumption.
"""

import numpy as np
from typing import List, Dict, Tuple, Optional, Any
from nav_msgs.msg import OccupancyGrid
import math

class GridAnalyzer:
    """Analyzes occupancy grids and converts them to LLM-friendly format"""
    
    def __init__(self, config: Dict[str, Any]):
        self.grid_resolution = config.get("grid_resolution", 0.5)
        self.obstacle_threshold = config.get("obstacle_threshold", 0.65)
        self.max_range = config.get("max_range", 10.0)
        
    def occupancy_grid_to_symbols(self, occupancy_grid: OccupancyGrid, 
                                 robot_pos: Optional[Tuple[float, float]] = None,
                                 target_pos: Optional[Tuple[float, float]] = None) -> List[List[str]]:
        """
        Convert ROS2 OccupancyGrid to symbol grid for LLM consumption
        
        Symbols:
        - '.' : Free space (occupancy < threshold)
        - '#' : Obstacle/wall (occupancy >= threshold)  
        - '?' : Unknown space (occupancy == -1)
        - '@' : Robot position
        - '*' : Target position
        """
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution
        origin = occupancy_grid.info.origin.position
        
        # Convert occupancy data to 2D array
        occupancy_data = np.array(occupancy_grid.data).reshape((height, width))
        
        # Initialize symbol grid (flip Y-axis for bottom-left origin)
        symbol_grid = []
        for y in range(height):
            row = []
            for x in range(width):
                # Get occupancy value (flipped Y coordinate)
                occupancy_val = occupancy_data[height - 1 - y, x]
                
                if occupancy_val == -1:
                    symbol = '?'  # Unknown
                elif occupancy_val >= self.obstacle_threshold * 100:
                    symbol = '#'  # Obstacle
                else:
                    symbol = '.'  # Free space
                    
                row.append(symbol)
            symbol_grid.append(row)
        
        # Add robot position if provided
        if robot_pos:
            robot_grid_pos = self.world_to_grid(robot_pos, origin, resolution)
            if self._is_valid_grid_position(robot_grid_pos, width, height):
                symbol_grid[robot_grid_pos[1]][robot_grid_pos[0]] = '@'
        
        # Add target position if provided  
        if target_pos:
            target_grid_pos = self.world_to_grid(target_pos, origin, resolution)
            if self._is_valid_grid_position(target_grid_pos, width, height):
                symbol_grid[target_grid_pos[1]][target_grid_pos[0]] = '*'
                
        return symbol_grid
    
    def world_to_grid(self, world_pos: Tuple[float, float], 
                     origin: Any, resolution: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        grid_x = int((world_pos[0] - origin.x) / resolution)
        grid_y = int((world_pos[1] - origin.y) / resolution)
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_pos: Tuple[int, int], 
                     origin: Any, resolution: float) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        world_x = origin.x + grid_pos[0] * resolution
        world_y = origin.y + grid_pos[1] * resolution
        return (world_x, world_y)
    
    def _is_valid_grid_position(self, grid_pos: Tuple[int, int], 
                               width: int, height: int) -> bool:
        """Check if grid position is within bounds"""
        return (0 <= grid_pos[0] < width and 0 <= grid_pos[1] < height)
    
    def analyze_grid(self, symbol_grid: List[List[str]], 
                    robot_pos: Tuple[int, int]) -> Dict[str, Any]:
        """
        Analyze symbol grid and extract useful information for LLM
        """
        height = len(symbol_grid)
        width = len(symbol_grid[0]) if symbol_grid else 0
        
        # Find all positions of different symbols
        unexplored_positions = []
        obstacle_positions = []
        free_positions = []
        target_position = None
        
        for y in range(height):
            for x in range(width):
                symbol = symbol_grid[y][x]
                if symbol == '?':
                    unexplored_positions.append([x, y])
                elif symbol == '#':
                    obstacle_positions.append([x, y])
                elif symbol == '.':
                    free_positions.append([x, y])
                elif symbol == '*':
                    target_position = [x, y]
        
        # Calculate statistics
        total_cells = width * height
        explored_cells = len(free_positions) + len(obstacle_positions)
        unexplored_percentage = (len(unexplored_positions) / total_cells) * 100 if total_cells > 0 else 0
        
        # Analyze path to target if target exists
        path_exists = False
        obstacles_between = []
        if target_position:
            path_exists, obstacles_between = self._analyze_path_to_target(
                symbol_grid, robot_pos, target_position)
        
        # Find large open areas
        large_open_areas = self._find_open_areas(symbol_grid, min_size=9)
        
        # Find narrow passages
        narrow_passages = self._find_narrow_passages(symbol_grid)
        
        return {
            "robot_symbol": "@",
            "robot_grid_position": list(robot_pos),
            "target_symbol": "*" if target_position else None,
            "target_grid_position": target_position,
            "unexplored_positions": unexplored_positions,
            "unexplored_percentage": int(unexplored_percentage),
            "path_exists": path_exists,
            "obstacles_between": obstacles_between,
            "large_open_areas": large_open_areas,
            "narrow_passages": narrow_passages,
            "grid_resolution": self.grid_resolution,
            "grid_origin": {"x": 0.0, "y": 0.0}  # Simplified for LLM
        }
    
    def _analyze_path_to_target(self, symbol_grid: List[List[str]], 
                               robot_pos: Tuple[int, int], 
                               target_pos: List[int]) -> Tuple[bool, List[str]]:
        """
        Analyze if there's a clear path from robot to target
        Returns (path_exists, list_of_obstacles_between)
        """
        obstacles_between = []
        
        # Simple line-of-sight check for straight paths
        if robot_pos[0] == target_pos[0]:  # Vertical path
            start_y = min(robot_pos[1], target_pos[1])
            end_y = max(robot_pos[1], target_pos[1])
            for y in range(start_y + 1, end_y):
                symbol = symbol_grid[y][robot_pos[0]]
                if symbol == '#':
                    obstacles_between.append(f"wall at [{robot_pos[0]},{y}]")
                elif symbol == '?':
                    obstacles_between.append(f"unknown at [{robot_pos[0]},{y}]")
                elif robot_pos[0] > 0 and robot_pos[0] < len(symbol_grid[0])-1:
                    # Check for doorway pattern (single free space between walls)
                    left_symbol = symbol_grid[y][robot_pos[0]-1]
                    right_symbol = symbol_grid[y][robot_pos[0]+1]
                    if left_symbol == '#' and right_symbol == '#':
                        obstacles_between.append(f"doorway at [{robot_pos[0]},{y}]")
        
        elif robot_pos[1] == target_pos[1]:  # Horizontal path
            start_x = min(robot_pos[0], target_pos[0])
            end_x = max(robot_pos[0], target_pos[0])
            for x in range(start_x + 1, end_x):
                symbol = symbol_grid[robot_pos[1]][x]
                if symbol == '#':
                    obstacles_between.append(f"wall at [{x},{robot_pos[1]}]")
                elif symbol == '?':
                    obstacles_between.append(f"unknown at [{x},{robot_pos[1]}]")
        
        # Path exists if no blocking obstacles (walls), unknown areas are navigable
        blocking_obstacles = [obs for obs in obstacles_between if "wall" in obs]
        path_exists = len(blocking_obstacles) == 0
        
        return path_exists, obstacles_between
    
    def _find_open_areas(self, symbol_grid: List[List[str]], 
                        min_size: int = 9) -> List[Dict[str, Any]]:
        """Find large open areas (connected free spaces)"""
        height = len(symbol_grid)
        width = len(symbol_grid[0]) if symbol_grid else 0
        visited = [[False] * width for _ in range(height)]
        open_areas = []
        
        def flood_fill(start_x: int, start_y: int) -> List[Tuple[int, int]]:
            """Flood fill to find connected free spaces"""
            stack = [(start_x, start_y)]
            area_cells = []
            
            while stack:
                x, y = stack.pop()
                if (x < 0 or x >= width or y < 0 or y >= height or 
                    visited[y][x] or symbol_grid[y][x] != '.'):
                    continue
                    
                visited[y][x] = True
                area_cells.append((x, y))
                
                # Add neighbors
                for dx, dy in [(0,1), (0,-1), (1,0), (-1,0)]:
                    stack.append((x + dx, y + dy))
            
            return area_cells
        
        # Find all open areas
        for y in range(height):
            for x in range(width):
                if symbol_grid[y][x] == '.' and not visited[y][x]:
                    area_cells = flood_fill(x, y)
                    if len(area_cells) >= min_size:
                        # Calculate center of area
                        center_x = sum(cell[0] for cell in area_cells) // len(area_cells)
                        center_y = sum(cell[1] for cell in area_cells) // len(area_cells)
                        
                        open_areas.append({
                            "center": [center_x, center_y],
                            "size": len(area_cells),
                            "label": "possible_room" if len(area_cells) > 20 else "open_area"
                        })
        
        return open_areas
    
    def _find_narrow_passages(self, symbol_grid: List[List[str]]) -> List[Dict[str, Any]]:
        """Find narrow passages (corridors)"""
        height = len(symbol_grid)
        width = len(symbol_grid[0]) if symbol_grid else 0
        passages = []
        
        # Look for horizontal corridors (single row of free space between walls)
        for y in range(1, height - 1):
            corridor_start = None
            for x in range(width):
                is_corridor_cell = (symbol_grid[y][x] == '.' and 
                                  symbol_grid[y-1][x] == '#' and 
                                  symbol_grid[y+1][x] == '#')
                
                if is_corridor_cell and corridor_start is None:
                    corridor_start = x
                elif not is_corridor_cell and corridor_start is not None:
                    # End of corridor
                    length = x - corridor_start
                    if length >= 3:  # Minimum corridor length
                        center_x = (corridor_start + x - 1) // 2
                        passages.append({
                            "center": [center_x, y],
                            "length": length,
                            "label": "possible_corridor"
                        })
                    corridor_start = None
        
        # Look for vertical corridors
        for x in range(1, width - 1):
            corridor_start = None
            for y in range(height):
                is_corridor_cell = (symbol_grid[y][x] == '.' and 
                                  symbol_grid[y][x-1] == '#' and 
                                  symbol_grid[y][x+1] == '#')
                
                if is_corridor_cell and corridor_start is None:
                    corridor_start = y
                elif not is_corridor_cell and corridor_start is not None:
                    # End of corridor
                    length = y - corridor_start
                    if length >= 3:  # Minimum corridor length
                        center_y = (corridor_start + y - 1) // 2
                        passages.append({
                            "center": [x, center_y],
                            "length": length,
                            "label": "possible_corridor"
                        })
                    corridor_start = None
        
        return passages
    
    def find_navigable_goals(self, symbol_grid: List[List[str]], 
                           robot_pos: Tuple[int, int],
                           max_goals: int = 5) -> List[Dict[str, Any]]:
        """Find potential navigation goals for the LLM"""
        goals = []
        
        # Add target if it exists
        target_pos = None
        for y in range(len(symbol_grid)):
            for x in range(len(symbol_grid[0])):
                if symbol_grid[y][x] == '*':
                    target_pos = [x, y]
                    break
            if target_pos:
                break
        
        if target_pos:
            distance = math.sqrt((target_pos[0] - robot_pos[0])**2 + 
                               (target_pos[1] - robot_pos[1])**2)
            goals.append({
                "label": "target",
                "grid_position": target_pos,
                "world_position": {
                    "x": target_pos[0] * self.grid_resolution,
                    "y": target_pos[1] * self.grid_resolution
                },
                "distance": round(distance * self.grid_resolution, 2)
            })
        
        # Add nearest unexplored areas
        unexplored_positions = []
        for y in range(len(symbol_grid)):
            for x in range(len(symbol_grid[0])):
                if symbol_grid[y][x] == '?':
                    distance = math.sqrt((x - robot_pos[0])**2 + (y - robot_pos[1])**2)
                    unexplored_positions.append({
                        "position": [x, y],
                        "distance": distance
                    })
        
        # Sort by distance and add closest unexplored areas
        unexplored_positions.sort(key=lambda x: x["distance"])
        for i, unexplored in enumerate(unexplored_positions[:max_goals-len(goals)]):
            pos = unexplored["position"]
            goals.append({
                "label": f"nearest_unexplored" if i == 0 else f"unexplored_{i+1}",
                "grid_position": pos,
                "world_position": {
                    "x": pos[0] * self.grid_resolution,
                    "y": pos[1] * self.grid_resolution
                },
                "distance": round(unexplored["distance"] * self.grid_resolution, 2)
            })
        
        return goals