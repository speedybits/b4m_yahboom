#!/usr/bin/env python3
"""
Console Output Utilities for B4M LLM Navigation API

Provides comprehensive, color-coded console logging for monitoring
LLM communications and navigation decisions in real-time.
"""

import json
import time
from datetime import datetime
from typing import Dict, Any, Optional, List
from enum import Enum

class MessageType(Enum):
    """Console message types with color codes"""
    API_REQUEST = ("→ REQ", "\033[94m")     # Blue
    API_RESPONSE = ("← RES", "\033[92m")    # Green  
    GRID_UPDATE = ("GRID", "\033[93m")      # Yellow
    NAVIGATION = ("NAV", "\033[95m")        # Purple
    DECISION = ("DECISION", "\033[91m")     # Orange/Red
    STATUS = ("STATUS", "\033[97m")         # White
    ERROR = ("ERROR", "\033[91m")           # Red

class GridDisplayMode(Enum):
    """Grid visualization modes"""
    COMPACT = "compact"
    STANDARD = "standard" 
    DETAILED = "detailed"

class ConsoleLogger:
    """Enhanced console logger for B4M navigation system"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.use_colors = config.get("use_colors", True)
        self.timestamp_format = config.get("timestamp_format", "HH:MM:SS.mmm")
        self.log_level = config.get("log_level", "INFO")
        self.grid_mode = GridDisplayMode(config.get("grid_display_mode", "standard"))
        
        # Message type filters
        self.show_api_requests = config.get("show_api_requests", True)
        self.show_api_responses = config.get("show_api_responses", True)
        self.show_grid_visualization = config.get("show_grid_visualization", True)
        self.show_navigation_commands = config.get("show_navigation_commands", True)
        self.show_decision_reasoning = config.get("show_decision_reasoning", True)
        
        # Color codes
        self.RESET = "\033[0m" if self.use_colors else ""
        
    def _get_timestamp(self) -> str:
        """Generate formatted timestamp"""
        now = datetime.now()
        if self.timestamp_format == "HH:MM:SS.mmm":
            return now.strftime("%H:%M:%S.") + f"{now.microsecond // 1000:03d}"
        return now.strftime(self.timestamp_format)
    
    def _format_message(self, msg_type: MessageType, content: str, indent: bool = True) -> str:
        """Format console message with timestamp, color, and prefix"""
        timestamp = self._get_timestamp()
        prefix, color = msg_type.value
        
        if self.use_colors:
            formatted = f"[{timestamp}] {color}[{prefix}]{self.RESET} {content}"
        else:
            formatted = f"[{timestamp}] [{prefix}] {content}"
        
        if indent and "\n" in content:
            # Add proper indentation for multi-line content
            lines = content.split("\n")
            if len(lines) > 1:
                indent_str = " " * (len(timestamp) + len(prefix) + 6)
                formatted = formatted.replace("\n", f"\n{indent_str}")
        
        return formatted
    
    def _should_log(self, msg_type: MessageType) -> bool:
        """Check if message type should be logged based on filters"""
        if msg_type == MessageType.API_REQUEST:
            return self.show_api_requests
        elif msg_type == MessageType.API_RESPONSE:
            return self.show_api_responses
        elif msg_type == MessageType.GRID_UPDATE:
            return self.show_grid_visualization
        elif msg_type == MessageType.NAVIGATION:
            return self.show_navigation_commands
        elif msg_type == MessageType.DECISION:
            return self.show_decision_reasoning
        return True  # Always show STATUS and ERROR
    
    def log_api_request(self, method: str, endpoint: str, data: Optional[Dict] = None):
        """Log incoming API request"""
        if not self._should_log(MessageType.API_REQUEST):
            return
            
        message = f"{method} {endpoint}"
        if data:
            message += f"\n{json.dumps(data, indent=2)}"
        
        print(self._format_message(MessageType.API_REQUEST, message))
    
    def log_api_response(self, status_code: int, endpoint: str, data: Dict, 
                        duration_ms: float):
        """Log outgoing API response"""
        if not self._should_log(MessageType.API_RESPONSE):
            return
            
        message = f"{status_code} OK {endpoint} ({duration_ms:.0f}ms)\n"
        message += json.dumps(data, indent=2)
        
        print(self._format_message(MessageType.API_RESPONSE, message))
    
    def log_grid_analysis(self, grid_data: List[List[str]], robot_pos: List[int], 
                         target_pos: Optional[List[int]] = None, 
                         unexplored_pct: int = 0, clear_path: bool = False):
        """Log grid visualization and analysis"""
        if not self._should_log(MessageType.GRID_UPDATE):
            return
        
        if self.grid_mode == GridDisplayMode.COMPACT:
            self._log_grid_compact(robot_pos, target_pos, unexplored_pct, clear_path)
        elif self.grid_mode == GridDisplayMode.STANDARD:
            self._log_grid_standard(grid_data, robot_pos, target_pos, unexplored_pct, clear_path)
        elif self.grid_mode == GridDisplayMode.DETAILED:
            self._log_grid_detailed(grid_data, robot_pos, target_pos, unexplored_pct)
    
    def _log_grid_compact(self, robot_pos: List[int], target_pos: Optional[List[int]],
                         unexplored_pct: int, clear_path: bool):
        """Log compact grid summary"""
        target_str = f" → * at ({target_pos[0]},{target_pos[1]})" if target_pos else ""
        path_str = "CLEAR" if clear_path else "BLOCKED"
        message = f"@ at ({robot_pos[0]},{robot_pos[1]}){target_str} | Path: {path_str} | Unknown: {unexplored_pct}%"
        print(self._format_message(MessageType.GRID_UPDATE, message))
    
    def _log_grid_standard(self, grid_data: List[List[str]], robot_pos: List[int],
                          target_pos: Optional[List[int]], unexplored_pct: int, clear_path: bool):
        """Log standard ASCII grid visualization"""
        message = f"Grid analysis:\n"
        
        # Add column headers
        message += "                       " + " ".join(f"{i}" for i in range(10)) + "\n"
        
        # Add grid rows (flipped for bottom-left origin)
        for y in range(len(grid_data) - 1, -1, -1):
            row = grid_data[y]
            message += f"                    {y}  " + "  ".join(row) + "\n"
        
        # Add summary
        target_str = f" | Target: * at ({target_pos[0]},{target_pos[1]})" if target_pos else ""
        path_str = "YES" if clear_path else "NO"
        message += f"                    Robot: @ at ({robot_pos[0]},{robot_pos[1]}){target_str}\n"
        message += f"                    Unexplored: {unexplored_pct}% | Clear path: {path_str}"
        
        print(self._format_message(MessageType.GRID_UPDATE, message))
    
    def _log_grid_detailed(self, grid_data: List[List[str]], robot_pos: List[int],
                          target_pos: Optional[List[int]], unexplored_pct: int):
        """Log detailed grid analysis with statistics"""
        message = "Occupancy Grid Analysis\n"
        message += f"       Resolution: 0.5m/cell | Origin: (0.0, 0.0)\n"
        message += f"       Dimensions: {len(grid_data[0])}x{len(grid_data)} cells ({len(grid_data[0])*0.5:.1f}m x {len(grid_data)*0.5:.1f}m)\n\n"
        message += "       Legend: @ Robot | * Target | # Wall | . Free | ? Unknown\n\n"
        message += "       Grid State:\n"
        
        # Add the actual grid
        for y in range(len(grid_data) - 1, -1, -1):
            row = grid_data[y]
            message += f"       {y:2}  " + "  ".join(row) + "\n"
        
        # Calculate statistics
        total_cells = len(grid_data) * len(grid_data[0])
        free_cells = sum(row.count('.') for row in grid_data)
        obstacle_cells = sum(row.count('#') for row in grid_data)
        unknown_cells = sum(row.count('?') for row in grid_data)
        explored_cells = free_cells + obstacle_cells
        
        message += f"\n       Statistics:\n"
        message += f"       - Explored: {(explored_cells/total_cells)*100:.0f}% ({explored_cells}/{total_cells} cells)\n"
        message += f"       - Free space: {free_cells} cells\n"
        message += f"       - Obstacles: {obstacle_cells} cells\n"
        message += f"       - Unknown: {unknown_cells} cells\n"
        
        if target_pos:
            distance = ((target_pos[0] - robot_pos[0])**2 + (target_pos[1] - robot_pos[1])**2)**0.5
            message += f"       - Robot→Target: {distance*0.5:.1f}m ({int(distance)} cells)"
        
        print(self._format_message(MessageType.GRID_UPDATE, message))
    
    def log_navigation_command(self, target_pos: Dict[str, float], frame: str = "map",
                              planner: str = "NavFn/A*"):
        """Log navigation command sent to Nav2"""
        if not self._should_log(MessageType.NAVIGATION):
            return
            
        x, y = target_pos.get("x", 0), target_pos.get("y", 0)
        orientation = target_pos.get("orientation", 0)
        
        message = f"Sending goal to Navigation2:\n"
        message += f"Target: ({x:.1f}, {y:.1f}) | Orientation: {orientation:.2f} rad ({orientation*180/3.14159:.0f}°)\n"
        message += f"Frame: {frame} | Planner: {planner}"
        
        print(self._format_message(MessageType.NAVIGATION, message))
    
    def log_decision_analysis(self, current_pos: Dict[str, float], delta: Dict[str, float],
                             target_pos: Dict[str, float], path_clear: bool, 
                             estimated_time: float):
        """Log navigation decision analysis"""
        if not self._should_log(MessageType.DECISION):
            return
            
        message = f"Navigation request analysis:\n"
        message += f"- Current position: ({current_pos['x']:.1f}, {current_pos['y']:.1f})\n"
        message += f"- Delta movement: {delta['x']:.1f}m east, {delta['y']:.1f}m {'north' if delta['y'] >= 0 else 'south'}\n"
        message += f"- Calculated target: ({target_pos['x']:.1f}, {target_pos['y']:.1f})\n"
        message += f"- Path validation: {'CLEAR' if path_clear else 'BLOCKED'}\n"
        message += f"- Estimated time: {estimated_time:.1f} seconds"
        
        print(self._format_message(MessageType.DECISION, message))
    
    def log_simulated_reasoning(self, robot_pos: List[int], unexplored_areas: List,
                               target_pos: Optional[List[int]], clear_areas: List,
                               decision: Dict[str, Any], confidence: float):
        """Log detailed simulated B4M reasoning"""
        if not self._should_log(MessageType.DECISION):
            return
            
        message = "=== SIMULATED B4M REASONING ===\n"
        message += "Grid Analysis:\n"
        message += f"- Robot position: [{robot_pos[0]}, {robot_pos[1]}]\n"
        message += f"- Unexplored areas: {len(unexplored_areas)}% at positions {unexplored_areas[:5]}\n"
        if target_pos:
            distance = ((target_pos[0] - robot_pos[0])**2 + (target_pos[1] - robot_pos[1])**2)**0.5 * 0.5
            message += f"- Target marker: [{target_pos[0]}, {target_pos[1]}] ({distance:.1f}m {'north' if target_pos[1] > robot_pos[1] else 'south'})\n"
        message += f"- Clear areas: {len(clear_areas)}% of known space\n\n"
        
        message += "Decision Process:\n"
        if target_pos:
            message += "1. Priority check: Target marker exists\n"
            message += "2. Path analysis: Clear path analysis\n"
            message += "3. Safety check: No obstacles within safety margin\n"
            message += "4. Strategy: Direct navigation to target\n"
            action = f"Navigate to target at ({target_pos[0]},{target_pos[1]})"
        else:
            message += "1. Priority check: No target marker found\n"
            message += "2. Exploration check: Unexplored areas available\n"
            message += "3. Safety check: Path validation\n"
            message += "4. Strategy: Exploration navigation\n"
            action = "Navigate to unexplored area"
        
        message += f"\nSelected Action: {action}\n"
        message += f"Confidence: {confidence:.0f}%\n"
        message += "================================"
        
        print(self._format_message(MessageType.DECISION, message))
    
    def log_status(self, message: str):
        """Log general status message"""
        print(self._format_message(MessageType.STATUS, message))
    
    def log_error(self, message: str):
        """Log error message"""
        print(self._format_message(MessageType.ERROR, message))
    
    def log_navigation_progress(self, progress_pct: int, distance_remaining: float):
        """Log navigation progress update"""
        if progress_pct >= 100:
            self.log_status("✓ Goal reached successfully!")
        else:
            self.log_status(f"Navigation progress: {progress_pct}% ({distance_remaining:.1f}m remaining)")