#!/usr/bin/env python3
"""
B4M LLM Navigation API Server

FastAPI server that bridges LLMs with ROS2 Navigation2 stack, providing
spatial awareness through occupancy grid analysis and natural language
descriptions with comprehensive console logging.
"""

import json
import time
import asyncio
from typing import Dict, Any, Optional, List
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel
import uvicorn

from .utils.console_output import ConsoleLogger, MessageType
from .utils.map_utils import GridAnalyzer
from .utils.text_generator import SpatialTextGenerator


class NavigationRequest(BaseModel):
    """Navigation request model"""
    x: float
    y: float
    orientation: Optional[float] = 0.0
    speed: Optional[str] = "normal"  # slow, normal, fast


class SpatialQuery(BaseModel):
    """Spatial environment query model"""
    include_grid: Optional[bool] = False
    verbosity: Optional[str] = "normal"  # minimal, normal, detailed
    analysis_range: Optional[float] = 10.0


class SimulatedDecision(BaseModel):
    """Simulated B4M decision response"""
    action: str
    target_position: Dict[str, float]
    confidence: float
    reasoning: str


class B4MLLMNavigationAPI(Node):
    """Main ROS2 node providing FastAPI server for LLM navigation"""
    
    def __init__(self, config: Dict[str, Any]):
        super().__init__('b4m_llm_nav_api')
        
        self.config = config
        self.console = ConsoleLogger(config.get("console_output", {}))
        self.grid_analyzer = GridAnalyzer(config.get("spatial_interpreter", {}))
        self.text_generator = SpatialTextGenerator(config.get("text_generation", {}))
        
        # Current state
        self.current_map: Optional[OccupancyGrid] = None
        self.current_pose: Optional[Dict[str, float]] = None
        self.last_grid_analysis: Optional[Dict[str, Any]] = None
        self.simulated_mode = config.get("simulated_b4m", {}).get("enabled", False)
        
        # ROS2 subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile
        )
        
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # FastAPI setup
        self.app = FastAPI(
            title="B4M LLM Navigation API",
            description="Bridge between LLMs and ROS2 Navigation2 stack",
            version="1.0.0"
        )
        
        self._setup_fastapi_routes()
        self._setup_cors()
        
        self.console.log_status("B4M LLM Navigation API initialized")
        if self.simulated_mode:
            self.console.log_status("Running in SIMULATED B4M mode")
    
    def _setup_cors(self):
        """Configure CORS for FastAPI"""
        if self.config.get("api_server", {}).get("cors_enabled", True):
            self.app.add_middleware(
                CORSMiddleware,
                allow_origins=["*"],
                allow_credentials=True,
                allow_methods=["*"],
                allow_headers=["*"],
            )
    
    def _setup_fastapi_routes(self):
        """Setup FastAPI route handlers"""
        
        @self.app.middleware("http")
        async def log_requests(request: Request, call_next):
            """Log all incoming requests"""
            start_time = time.time()
            
            # Log request
            body = await request.body()
            data = None
            if body:
                try:
                    data = json.loads(body.decode())
                except json.JSONDecodeError:
                    data = {"raw_body": body.decode()[:200]}
            
            self.console.log_api_request(
                method=request.method,
                endpoint=str(request.url.path),
                data=data
            )
            
            # Process request
            response = await call_next(request)
            
            # Log response timing
            duration_ms = (time.time() - start_time) * 1000
            
            return response
        
        @self.app.get("/")
        async def root():
            """API health check"""
            return {
                "status": "operational",
                "service": "B4M LLM Navigation API",
                "mode": "simulated" if self.simulated_mode else "real",
                "timestamp": datetime.now().isoformat(),
                "map_available": self.current_map is not None,
                "pose_available": self.current_pose is not None
            }
        
        @self.app.get("/spatial/environment")
        async def get_spatial_environment(
            include_grid: bool = False,
            verbosity: str = "normal",
            analysis_range: float = 10.0
        ):
            """Get spatial environment description for LLM"""
            start_time = time.time()
            
            if not self.current_map or not self.current_pose:
                error_msg = "Map or pose data not available"
                self.console.log_error(error_msg)
                raise HTTPException(status_code=503, detail=error_msg)
            
            # Convert map to symbol grid
            robot_world_pos = (self.current_pose["x"], self.current_pose["y"])
            symbol_grid = self.grid_analyzer.occupancy_grid_to_symbols(
                self.current_map, robot_world_pos
            )
            
            # Analyze grid
            robot_grid_pos = self.grid_analyzer.world_to_grid(
                robot_world_pos, 
                self.current_map.info.origin.position,
                self.current_map.info.resolution
            )
            
            self.last_grid_analysis = self.grid_analyzer.analyze_grid(
                symbol_grid, robot_grid_pos
            )
            
            # Generate text description
            if verbosity == "minimal":
                description = self.text_generator.generate_compact_description(
                    self.last_grid_analysis
                )
            elif verbosity == "detailed":
                description = self.text_generator.generate_spatial_description(
                    self.last_grid_analysis, self.current_pose.get("orientation", 0.0)
                )
            else:  # normal
                description = self.text_generator.generate_decision_context(
                    self.last_grid_analysis, self.current_pose.get("orientation", 0.0)
                )
            
            # Log grid analysis
            self.console.log_grid_analysis(
                grid_data=symbol_grid,
                robot_pos=list(robot_grid_pos),
                target_pos=self.last_grid_analysis.get("target_grid_position"),
                unexplored_pct=self.last_grid_analysis["unexplored_percentage"],
                clear_path=self.last_grid_analysis.get("path_exists", False)
            )
            
            # Prepare response
            response_data = {
                "description": description,
                "robot_position": {
                    "x": self.current_pose["x"],
                    "y": self.current_pose["y"],
                    "orientation": self.current_pose.get("orientation", 0.0)
                },
                "grid_analysis": {
                    "unexplored_percentage": self.last_grid_analysis["unexplored_percentage"],
                    "navigable_goals": self.grid_analyzer.find_navigable_goals(
                        symbol_grid, robot_grid_pos, max_goals=3
                    )
                }
            }
            
            if include_grid:
                response_data["symbol_grid"] = symbol_grid
            
            # Log response
            duration_ms = (time.time() - start_time) * 1000
            self.console.log_api_response(
                status_code=200,
                endpoint="/spatial/environment",
                data=response_data,
                duration_ms=duration_ms
            )
            
            return response_data
        
        @self.app.post("/navigation/goto")
        async def navigate_to_position(request: NavigationRequest):
            """Navigate to specified position"""
            start_time = time.time()
            
            if not self.current_pose:
                error_msg = "Current pose not available"
                self.console.log_error(error_msg)
                raise HTTPException(status_code=503, detail=error_msg)
            
            # Calculate delta movement for analysis
            delta = {
                "x": request.x - self.current_pose["x"],
                "y": request.y - self.current_pose["y"]
            }
            
            # Estimate path clear status and travel time
            path_clear = True  # Simplified for now
            distance = (delta["x"]**2 + delta["y"]**2)**0.5
            speed_config = self.config.get("navigation", {})
            base_speed = speed_config.get(f"linear_speed_{request.speed}", 0.2)
            estimated_time = distance / base_speed
            
            # Log decision analysis
            self.console.log_decision_analysis(
                current_pos=self.current_pose,
                delta=delta,
                target_pos={"x": request.x, "y": request.y},
                path_clear=path_clear,
                estimated_time=estimated_time
            )
            
            # Log navigation command
            self.console.log_navigation_command(
                target_pos={
                    "x": request.x, 
                    "y": request.y, 
                    "orientation": request.orientation
                }
            )
            
            # Send navigation goal to Nav2
            success = await self._send_navigation_goal(request)
            
            response_data = {
                "status": "accepted" if success else "failed",
                "target": {
                    "x": request.x,
                    "y": request.y,
                    "orientation": request.orientation
                },
                "estimated_duration": estimated_time,
                "movement_delta": delta
            }
            
            # Log response
            duration_ms = (time.time() - start_time) * 1000
            self.console.log_api_response(
                status_code=200 if success else 500,
                endpoint="/navigation/goto",
                data=response_data,
                duration_ms=duration_ms
            )
            
            if not success:
                raise HTTPException(status_code=500, detail="Navigation goal failed")
            
            return response_data
        
        @self.app.post("/navigation/delta")
        async def navigate_delta(delta_x: float, delta_y: float, 
                               orientation: Optional[float] = None):
            """Navigate using relative delta movement"""
            if not self.current_pose:
                error_msg = "Current pose not available"
                self.console.log_error(error_msg)
                raise HTTPException(status_code=503, detail=error_msg)
            
            # Calculate absolute target
            target_x = self.current_pose["x"] + delta_x
            target_y = self.current_pose["y"] + delta_y
            target_orientation = orientation or self.current_pose.get("orientation", 0.0)
            
            # Use existing goto endpoint
            nav_request = NavigationRequest(
                x=target_x,
                y=target_y,
                orientation=target_orientation
            )
            
            return await navigate_to_position(nav_request)
        
        @self.app.get("/navigation/simulated_decision")
        async def get_simulated_decision():
            """Get simulated B4M navigation decision"""
            start_time = time.time()
            
            if not self.simulated_mode:
                error_msg = "Simulated B4M mode not enabled"
                self.console.log_error(error_msg)
                raise HTTPException(status_code=400, detail=error_msg)
            
            if not self.last_grid_analysis or not self.current_pose:
                error_msg = "Grid analysis or pose not available"
                self.console.log_error(error_msg)
                raise HTTPException(status_code=503, detail=error_msg)
            
            # Simulate B4M decision making
            decision = self._simulate_b4m_decision()
            
            # Log simulated reasoning
            self.console.log_simulated_reasoning(
                robot_pos=self.last_grid_analysis["robot_grid_position"],
                unexplored_areas=self.last_grid_analysis["unexplored_positions"][:3],
                target_pos=self.last_grid_analysis.get("target_grid_position"),
                clear_areas=[],  # Simplified
                decision=decision,
                confidence=decision["confidence"]
            )
            
            response_data = {
                "decision": decision["action"],
                "target": decision["target_position"],
                "confidence": decision["confidence"],
                "reasoning": decision["reasoning"],
                "timestamp": datetime.now().isoformat()
            }
            
            # Log response
            duration_ms = (time.time() - start_time) * 1000
            self.console.log_api_response(
                status_code=200,
                endpoint="/navigation/simulated_decision",
                data=response_data,
                duration_ms=duration_ms
            )
            
            return response_data
    
    def _simulate_b4m_decision(self) -> Dict[str, Any]:
        """Simulate B4M decision making process"""
        robot_pos = self.last_grid_analysis["robot_grid_position"]
        target_pos = self.last_grid_analysis.get("target_grid_position")
        unexplored_positions = self.last_grid_analysis["unexplored_positions"]
        
        # Simple decision logic
        if target_pos:
            # Navigate to target
            world_target = self.grid_analyzer.grid_to_world(
                (target_pos[0], target_pos[1]),
                self.current_map.info.origin.position,
                self.current_map.info.resolution
            )
            return {
                "action": "navigate_to_target",
                "target_position": {"x": world_target[0], "y": world_target[1]},
                "confidence": 85.0,
                "reasoning": "Target marker detected. Direct navigation path appears clear."
            }
        elif unexplored_positions:
            # Explore nearest unexplored area
            nearest = min(unexplored_positions, 
                         key=lambda p: (p[0] - robot_pos[0])**2 + (p[1] - robot_pos[1])**2)
            world_target = self.grid_analyzer.grid_to_world(
                (nearest[0], nearest[1]),
                self.current_map.info.origin.position,
                self.current_map.info.resolution
            )
            return {
                "action": "explore_unknown",
                "target_position": {"x": world_target[0], "y": world_target[1]},
                "confidence": 70.0,
                "reasoning": "No target found. Exploring nearest unknown area for mapping."
            }
        else:
            # Stay in place
            return {
                "action": "hold_position",
                "target_position": {"x": self.current_pose["x"], "y": self.current_pose["y"]},
                "confidence": 60.0,
                "reasoning": "Area fully explored. No specific target. Maintaining position."
            }
    
    async def _send_navigation_goal(self, request: NavigationRequest) -> bool:
        """Send navigation goal to Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.console.log_error("Navigation server not available")
            return False
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = request.x
        goal_msg.pose.pose.position.y = request.y
        goal_msg.pose.pose.position.z = 0.0
        
        # Set orientation (simplified quaternion from yaw)
        yaw = request.orientation
        goal_msg.pose.pose.orientation.z = (yaw / 2.0) if yaw else 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        try:
            # Send goal asynchronously
            future = self.nav_client.send_goal_async(goal_msg)
            return True
        except Exception as e:
            self.console.log_error(f"Failed to send navigation goal: {str(e)}")
            return False
    
    def map_callback(self, msg: OccupancyGrid):
        """Handle incoming map updates"""
        self.current_map = msg
        self.console.log_status(f"Map updated: {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m/cell")
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle robot pose updates"""
        pose = msg.pose.pose
        self.current_pose = {
            "x": pose.position.x,
            "y": pose.position.y,
            "orientation": 2.0 * pose.orientation.z  # Simplified from quaternion
        }
    
    async def run_server(self):
        """Run the FastAPI server"""
        server_config = self.config.get("api_server", {})
        host = server_config.get("host", "0.0.0.0")
        port = server_config.get("port", 8080)
        
        self.console.log_status(f"Starting FastAPI server on {host}:{port}")
        
        config = uvicorn.Config(
            app=self.app,
            host=host,
            port=port,
            log_level="warning"  # Reduce uvicorn logs, we handle our own
        )
        server = uvicorn.Server(config)
        await server.serve()


def main():
    """Main entry point"""
    # Load configuration
    import yaml
    import os
    
    config_path = os.path.join(
        os.path.dirname(__file__), 
        "..", "config", "llm_nav_api.yaml"
    )
    
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Configuration file not found: {config_path}")
        config = {}
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create and run API server
        api_node = B4MLLMNavigationAPI(config)
        
        # Run both ROS2 spinning and FastAPI server
        async def run_both():
            import threading
            
            # Start ROS2 spinning in separate thread
            def spin_ros():
                rclpy.spin(api_node)
            
            ros_thread = threading.Thread(target=spin_ros, daemon=True)
            ros_thread.start()
            
            # Run FastAPI server
            await api_node.run_server()
        
        # Run the async main loop
        asyncio.run(run_both())
        
    except KeyboardInterrupt:
        print("\nShutting down B4M LLM Navigation API...")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()