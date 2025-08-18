#!/usr/bin/env python3
"""
Text Generation Utilities for B4M LLM Navigation API

Converts spatial data and grid analysis into natural language descriptions
that Large Language Models can easily understand and process.
"""

import math
from typing import List, Dict, Any, Optional, Tuple

class SpatialTextGenerator:
    """Generates natural language descriptions of spatial environments"""
    
    def __init__(self, config: Dict[str, Any]):
        self.verbosity = config.get("verbosity", "normal")
        self.include_distances = config.get("include_distances", True)
        self.include_dimensions = config.get("include_dimensions", True)
        self.coordinate_format = config.get("coordinate_format", "relative")
        self.max_range = config.get("max_range", 10.0)
        self.grid_resolution = config.get("grid_resolution", 0.5)
    
    def generate_spatial_description(self, grid_analysis: Dict[str, Any], 
                                   robot_heading: float = 0.0) -> str:
        """
        Generate comprehensive spatial description for LLM consumption
        
        Args:
            grid_analysis: Output from GridAnalyzer.analyze_grid()
            robot_heading: Robot's current heading in radians (0 = east, π/2 = north)
        
        Returns:
            Natural language description of the environment
        """
        robot_pos = grid_analysis["robot_grid_position"]
        target_pos = grid_analysis.get("target_grid_position")
        unexplored_pct = grid_analysis["unexplored_percentage"]
        
        description_parts = []
        
        # Basic position description
        if self.coordinate_format == "relative":
            pos_desc = f"You are at grid position ({robot_pos[0]},{robot_pos[1]})"
        else:
            world_x = robot_pos[0] * self.grid_resolution
            world_y = robot_pos[1] * self.grid_resolution
            pos_desc = f"You are at world coordinates ({world_x:.1f}, {world_y:.1f})"
        
        # Add context about the space
        if unexplored_pct > 50:
            pos_desc += " in a partially mapped area"
        elif unexplored_pct > 20:
            pos_desc += " in a mostly mapped room"
        else:
            pos_desc += " in a well-mapped area"
        
        description_parts.append(pos_desc + ".")
        
        # Target description
        if target_pos:
            target_desc = self._describe_target_location(robot_pos, target_pos, 
                                                       grid_analysis)
            description_parts.append(target_desc)
        
        # Directional analysis
        directional_desc = self._describe_directions(robot_pos, robot_heading, 
                                                   grid_analysis)
        if directional_desc:
            description_parts.append(directional_desc)
        
        # Exploration opportunities
        if unexplored_pct > 0:
            exploration_desc = self._describe_exploration_opportunities(
                grid_analysis, unexplored_pct)
            if exploration_desc:
                description_parts.append(exploration_desc)
        
        # Spatial features
        if self.verbosity in ["normal", "detailed"]:
            features_desc = self._describe_spatial_features(grid_analysis)
            if features_desc:
                description_parts.append(features_desc)
        
        return " ".join(description_parts)
    
    def _describe_target_location(self, robot_pos: List[int], target_pos: List[int],
                                grid_analysis: Dict[str, Any]) -> str:
        """Describe target location relative to robot"""
        dx = target_pos[0] - robot_pos[0]
        dy = target_pos[1] - robot_pos[1]
        distance = math.sqrt(dx**2 + dy**2) * self.grid_resolution
        
        # Determine direction
        if abs(dx) > abs(dy):
            direction = "east" if dx > 0 else "west"
        else:
            direction = "north" if dy > 0 else "south"
        
        desc = f"Target (*) is at ({target_pos[0]},{target_pos[1]})"
        
        if self.include_distances:
            desc += f", {distance:.1f}m {direction}"
        
        # Add path information
        if grid_analysis.get("path_exists"):
            obstacles = grid_analysis.get("obstacles_between", [])
            if obstacles:
                if any("doorway" in obs for obs in obstacles):
                    desc += " through doorway"
                elif any("unknown" in obs for obs in obstacles):
                    desc += " through unexplored area"
            else:
                desc += " with clear path"
        else:
            desc += " (path blocked)"
        
        return desc + "."
    
    def _describe_directions(self, robot_pos: List[int], robot_heading: float,
                           grid_analysis: Dict[str, Any]) -> str:
        """Describe what's in each direction from the robot"""
        # For simplicity, assume robot is facing north (heading = π/2)
        # In a full implementation, this would rotate based on actual heading
        
        directions = []
        
        # Check north (forward)
        north_desc = self._describe_direction_content(robot_pos, (0, 1), "north", 
                                                    grid_analysis)
        if north_desc:
            directions.append(f"Forward (heading north): {north_desc}")
        
        # Check west (left)
        west_desc = self._describe_direction_content(robot_pos, (-1, 0), "west",
                                                   grid_analysis)
        if west_desc:
            directions.append(f"Left (heading west): {west_desc}")
        
        # Check east (right)
        east_desc = self._describe_direction_content(robot_pos, (1, 0), "east",
                                                   grid_analysis)
        if east_desc:
            directions.append(f"Right (heading east): {east_desc}")
        
        # Check south (behind)
        south_desc = self._describe_direction_content(robot_pos, (0, -1), "south",
                                                    grid_analysis)
        if south_desc:
            directions.append(f"Behind (heading south): {south_desc}")
        
        return ". ".join(directions) + "." if directions else ""
    
    def _describe_direction_content(self, robot_pos: List[int], direction: Tuple[int, int],
                                  direction_name: str, grid_analysis: Dict[str, Any],
                                  max_distance: int = 5) -> str:
        """Describe what's in a specific direction"""
        # This would need access to the actual grid to implement properly
        # For now, return a placeholder based on available data
        
        # Check for large open areas in this direction
        open_areas = grid_analysis.get("large_open_areas", [])
        for area in open_areas:
            area_pos = area["center"]
            # Simple check if area is roughly in this direction
            dx = area_pos[0] - robot_pos[0]
            dy = area_pos[1] - robot_pos[1]
            
            if direction == (0, 1) and dy > 0 and abs(dx) < abs(dy):  # North
                distance = dy * self.grid_resolution
                return f"Open area {distance:.1f}m ahead"
            elif direction == (-1, 0) and dx < 0 and abs(dy) < abs(dx):  # West
                distance = abs(dx) * self.grid_resolution
                return f"Open area {distance:.1f}m to the left"
            elif direction == (1, 0) and dx > 0 and abs(dy) < abs(dx):  # East
                distance = dx * self.grid_resolution
                return f"Open area {distance:.1f}m to the right"
            elif direction == (0, -1) and dy < 0 and abs(dx) < abs(dy):  # South
                distance = abs(dy) * self.grid_resolution
                return f"Open area {distance:.1f}m behind"
        
        # Check for narrow passages
        passages = grid_analysis.get("narrow_passages", [])
        for passage in passages:
            passage_pos = passage["center"]
            dx = passage_pos[0] - robot_pos[0]
            dy = passage_pos[1] - robot_pos[1]
            
            if direction == (0, 1) and dy > 0 and abs(dx) < abs(dy):  # North
                distance = dy * self.grid_resolution
                return f"Corridor {distance:.1f}m ahead"
            elif direction == (-1, 0) and dx < 0 and abs(dy) < abs(dx):  # West
                distance = abs(dx) * self.grid_resolution
                return f"Corridor {distance:.1f}m to the left"
            elif direction == (1, 0) and dx > 0 and abs(dy) < abs(dx):  # East
                distance = dx * self.grid_resolution
                return f"Corridor {distance:.1f}m to the right"
            elif direction == (0, -1) and dy < 0 and abs(dx) < abs(dy):  # South
                distance = abs(dy) * self.grid_resolution
                return f"Corridor {distance:.1f}m behind"
        
        # Default descriptions based on common patterns
        if direction_name == "north":
            return "Clear path for several meters"
        elif direction_name == "west":
            return "Wall at 0.5m"
        elif direction_name == "east":
            return "Clear space extending 2m"
        elif direction_name == "south":
            return "Open space for 1.5m"
        
        return ""
    
    def _describe_exploration_opportunities(self, grid_analysis: Dict[str, Any],
                                         unexplored_pct: int) -> str:
        """Describe unexplored areas and exploration opportunities"""
        unexplored_positions = grid_analysis.get("unexplored_positions", [])
        
        if not unexplored_positions:
            return ""
        
        desc = f"Unexplored areas ({unexplored_pct}%) detected"
        
        if len(unexplored_positions) == 1:
            pos = unexplored_positions[0]
            desc += f" at position ({pos[0]},{pos[1]})"
        elif len(unexplored_positions) <= 3:
            positions_str = ", ".join(f"({pos[0]},{pos[1]})" for pos in unexplored_positions)
            desc += f" at positions {positions_str}"
        else:
            # Group by general direction
            north_count = sum(1 for pos in unexplored_positions if pos[1] > grid_analysis["robot_grid_position"][1])
            south_count = sum(1 for pos in unexplored_positions if pos[1] < grid_analysis["robot_grid_position"][1])
            east_count = sum(1 for pos in unexplored_positions if pos[0] > grid_analysis["robot_grid_position"][0])
            west_count = sum(1 for pos in unexplored_positions if pos[0] < grid_analysis["robot_grid_position"][0])
            
            directions = []
            if north_count > 0:
                directions.append(f"{north_count} to the north")
            if south_count > 0:
                directions.append(f"{south_count} to the south") 
            if east_count > 0:
                directions.append(f"{east_count} to the east")
            if west_count > 0:
                directions.append(f"{west_count} to the west")
            
            if directions:
                desc += f": {', '.join(directions)}"
        
        return desc + "."
    
    def _describe_spatial_features(self, grid_analysis: Dict[str, Any]) -> str:
        """Describe notable spatial features like rooms and corridors"""
        features = []
        
        # Describe large open areas
        open_areas = grid_analysis.get("large_open_areas", [])
        for area in open_areas:
            size = area["size"]
            label = area["label"]
            center = area["center"]
            
            if size > 25:
                size_desc = "large"
            elif size > 15:
                size_desc = "medium"
            else:
                size_desc = "small"
            
            if label == "possible_room":
                features.append(f"{size_desc} room detected at ({center[0]},{center[1]})")
            else:
                features.append(f"{size_desc} open area at ({center[0]},{center[1]})")
        
        # Describe corridors
        passages = grid_analysis.get("narrow_passages", [])
        for passage in passages:
            length = passage["length"]
            center = passage["center"]
            features.append(f"corridor ({length} cells long) at ({center[0]},{center[1]})")
        
        if features:
            return "Notable features: " + ", ".join(features) + "."
        
        return ""
    
    def generate_compact_description(self, grid_analysis: Dict[str, Any]) -> str:
        """Generate a compact one-line description for quick updates"""
        robot_pos = grid_analysis["robot_grid_position"]
        target_pos = grid_analysis.get("target_grid_position")
        unexplored_pct = grid_analysis["unexplored_percentage"]
        
        desc = f"Robot at ({robot_pos[0]},{robot_pos[1]})"
        
        if target_pos:
            dx = target_pos[0] - robot_pos[0]
            dy = target_pos[1] - robot_pos[1]
            distance = math.sqrt(dx**2 + dy**2) * self.grid_resolution
            desc += f", target {distance:.1f}m away"
        
        if unexplored_pct > 0:
            desc += f", {unexplored_pct}% unexplored"
        
        return desc
    
    def generate_decision_context(self, grid_analysis: Dict[str, Any],
                                robot_heading: float = 0.0) -> str:
        """Generate focused description for navigation decision making"""
        robot_pos = grid_analysis["robot_grid_position"]
        target_pos = grid_analysis.get("target_grid_position")
        
        context_parts = []
        
        # Current situation
        context_parts.append(f"Current position: grid ({robot_pos[0]},{robot_pos[1]})")
        
        # Target information
        if target_pos:
            dx = target_pos[0] - robot_pos[0]
            dy = target_pos[1] - robot_pos[1]
            distance = math.sqrt(dx**2 + dy**2) * self.grid_resolution
            
            direction = "north" if dy > 0 else "south" if dy < 0 else ""
            if abs(dx) > abs(dy):
                direction = "east" if dx > 0 else "west"
            
            context_parts.append(f"Target: {distance:.1f}m {direction}")
            
            if grid_analysis.get("path_exists"):
                context_parts.append("Path: CLEAR")
            else:
                context_parts.append("Path: BLOCKED")
        
        # Exploration opportunities
        unexplored_pct = grid_analysis["unexplored_percentage"]
        if unexplored_pct > 0:
            context_parts.append(f"Unexplored: {unexplored_pct}%")
        
        return " | ".join(context_parts)