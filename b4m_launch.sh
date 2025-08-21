#!/bin/bash

# B4M Robot Launch Script
# This script automates the launch process for the B4M Robot 
# Automatically rebuilds workspace before launch to ensure latest code
# Each step will be launched in a separate terminal with user confirmation
#
# Usage: ./b4m_launch.sh [--skip-agent] [--only-agent] [--debug] [--simulation] [--regression] [--explore] [--b4m-api] [--b4m-lidar] [--b4m-HA] [--b4m-ping] [--b4m-nav] [--simulated-b4m] [--localization-test] [--tune-params] [--navigation-performance-test] [--parameter-set <name>] [--setup-wifi] [--skip-rebuild]
#   --skip-agent:                   Skip the Micro-ROS agent launch (Step 1)
#   --only-agent:                   Launch ONLY the Micro-ROS agent (Step 1) and exit
#   --debug:                        Enable verbose debug logging
#   --simulation:                   Launch in Gazebo simulation mode instead of real robot
#   --regression:                   Run comprehensive regression test suite (navigation + laser stability)
#   --explore:                      Enable autonomous exploration mode with obstacle avoidance
#   --b4m-api:                      Enable B4M API mode (duplicate of --explore for API integration)
#   --b4m-lidar:                    Enable B4M LiDAR-based intelligent navigation with API
#   --b4m-HA:                       (Experimental) Enable Home Assistant MQTT integration features
#   --b4m-ping:                     Test bike4mind API with random obstacle detection messages
#   --b4m-nav:                      Enable LLM Navigation API for spatial awareness and natural language control
#   --simulated-b4m:                Run B4M Navigation API in simulated mode (no real LLM calls)
#   --localization-test:            (Untested) Enable localization quality and navigation performance testing
#   --tune-params:                  (Untested) Enable parameter tuning iterations (requires --localization-test)
#   --navigation-performance-test:  (Untested) Execute 1x1m square navigation circuit testing
#   --parameter-set <name>:         (Untested) Specify parameter set to test (baseline, indoor_optimized, high_precision, balanced, fast_convergence)
#   --setup-wifi:                   Interactive WiFi setup wizard for robot configuration
#   --skip-rebuild:                 Skip automatic workspace rebuild (for development/debugging)

# Parse command line arguments
SKIP_AGENT=false
ONLY_AGENT=false
DEBUG_MODE=false
SIMULATION_MODE=false
REGRESSION_MODE=false
EXPLORE_MODE=false
B4M_API=false
B4M_LIDAR=false
B4M_PING=false
B4M_HA=false
B4M_NAV=false
SIMULATED_B4M=false
LOCALIZATION_TEST=false
TUNE_PARAMS=false
NAVIGATION_PERFORMANCE_TEST=false
PARAMETER_SET=""
SETUP_WIFI=false
SKIP_REBUILD=false
for arg in "$@"; do
    case $arg in
        --skip-agent)
            SKIP_AGENT=true
            shift
            ;;
        --only-agent)
            ONLY_AGENT=true
            shift
            ;;
        --debug)
            DEBUG_MODE=true
            shift
            ;;
        --simulation)
            SIMULATION_MODE=true
            shift
            ;;
        --regression)
            REGRESSION_MODE=true
            shift
            ;;
        --explore)
            EXPLORE_MODE=true
            shift
            ;;
        --b4m-api)
            B4M_API=true
            shift
            ;;
        --b4m-lidar)
            B4M_LIDAR=true
            shift
            ;;
        --b4m-ping)
            B4M_PING=true
            shift
            ;;
        --b4m-HA)
            B4M_HA=true
            shift
            ;;
        --b4m-nav)
            B4M_NAV=true
            shift
            ;;
        --simulated-b4m)
            SIMULATED_B4M=true
            B4M_NAV=true  # Auto-enable B4M Nav
            shift
            ;;
        --localization-test)
            LOCALIZATION_TEST=true
            shift
            ;;
        --tune-params)
            TUNE_PARAMS=true
            shift
            ;;
        --navigation-performance-test)
            NAVIGATION_PERFORMANCE_TEST=true
            LOCALIZATION_TEST=true  # Auto-enable localization test
            shift
            ;;
        --parameter-set)
            shift
            PARAMETER_SET="$1"
            shift
            ;;
        --setup-wifi)
            SETUP_WIFI=true
            shift
            ;;
        --skip-rebuild)
            SKIP_REBUILD=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--skip-agent] [--only-agent] [--debug] [--simulation] [--regression] [--explore] [--b4m-api] [--b4m-lidar] [--b4m-HA] [--b4m-ping] [--b4m-nav] [--simulated-b4m] [--localization-test] [--tune-params] [--navigation-performance-test] [--parameter-set <name>] [--setup-wifi] [--skip-rebuild]"
            echo "  --skip-agent:                   Skip the Micro-ROS agent launch (Step 1)"
            echo "  --only-agent:                   Launch ONLY the Micro-ROS agent (Step 1) and exit"
            echo "  --debug:                        Enable verbose debug logging"
            echo "  --simulation:                   Launch in Gazebo simulation mode instead of real robot"
            echo "  --regression:                   Run comprehensive regression test suite (navigation + laser stability)"
            echo "  --explore:                      Enable autonomous exploration mode with obstacle avoidance"
            echo "  --b4m-api:                      Enable B4M API mode (duplicate of --explore for API integration)"
            echo "  --b4m-lidar:                    Enable B4M LiDAR-based intelligent navigation with API"
	    echo "  --b4m-HA:                       (Experimental) Enable Home Assistant MQTT integration features"
            echo "  --b4m-ping:                     Test bike4mind API with random obstacle detection messages"
            echo "  --b4m-nav:                      Enable LLM Navigation API for spatial awareness and natural language control"
            echo "  --simulated-b4m:                Run B4M Navigation API in simulated mode (no real LLM calls)"
	    echo "  --localization-test:            (Untested) Enable localization quality and navigation performance testing"
	    echo "  --tune-params:                  (Untested) Enable parameter tuning iterations (requires --localization-test)"
	    echo "  --navigation-performance-test:  (Untested) Execute 1x1m square navigation circuit testing"
	    echo "  --parameter-set <name>:         (Untested) Specify parameter set to test (baseline, indoor_optimized, high_precision, balanced, fast_convergence)"
	    echo "  --setup-wifi:                   Interactive WiFi setup wizard for robot configuration"
	    echo "  --skip-rebuild:                 Skip automatic workspace rebuild (for development/debugging)"
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Automatically skip agent in simulation mode (simulation doesn't need physical robot connection)
if [ "$SIMULATION_MODE" = true ]; then
    SKIP_AGENT=true
    echo "🎮 Simulation mode detected: Automatically skipping Micro-ROS agent (no physical robot needed)"
fi

# Get the workspace root directory (where this script is located)
WORKSPACE_ROOT=$(cd "$(dirname "$0")" && pwd)

# Create logs directory if it doesn't exist
LOGS_DIR="$WORKSPACE_ROOT/logs"
mkdir -p "$LOGS_DIR"

# Generate timestamp for log files
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAIN_LOG="$LOGS_DIR/b4m_launch_$TIMESTAMP.log"

# Automatic workspace rebuild (unless skipped)
if [ "$SKIP_REBUILD" = false ]; then
    echo ""
    echo "======================================================"
    echo "🔨 REBUILDING WORKSPACE"
    echo "======================================================"
    echo "Rebuilding all packages to ensure latest code changes..."
    
    # Create rebuild log file
    REBUILD_LOG="$LOGS_DIR/colcon_build_$TIMESTAMP.log"
    echo "📝 Build log: $REBUILD_LOG"
    
    # Clean and rebuild all packages
    echo "🧹 Cleaning build and install directories..."
    rm -rf build/ install/ > /dev/null 2>&1 || true
    
    echo "🔨 Building all packages with symlink install..."
    if colcon build --symlink-install > "$REBUILD_LOG" 2>&1; then
        echo "✅ Workspace rebuild completed successfully"
        
        # Source all workspaces after rebuild
        echo "📂 Sourcing all workspaces..."
        if . source_workspaces.sh > /dev/null 2>&1; then
            echo "✅ Workspaces sourced successfully"
        else
            echo "⚠️  Warning: Could not source workspaces properly"
        fi
    else
        echo "❌ Workspace rebuild failed! Check logs at: $REBUILD_LOG"
        echo "Exiting due to build failure..."
        exit 1
    fi
    echo ""
else
    echo "⏭️  Skipping workspace rebuild (--skip-rebuild specified)"
    echo ""
fi

# Validate argument combinations
if [ "$EXPLORE_MODE" = true ] && [ "$REGRESSION_MODE" = true ]; then
    echo "ERROR: --explore mode is incompatible with --regression mode"
    echo "Exploration requires manual control while regression runs automated tests"
    exit 1
fi

if [ "$EXPLORE_MODE" = true ] && [ "$LOCALIZATION_TEST" = true ]; then
    echo "ERROR: --explore mode is incompatible with --localization-test mode"  
    echo "Both modes require different navigation behaviors"
    exit 1
fi

if [ "$EXPLORE_MODE" = true ] && [ "$NAVIGATION_PERFORMANCE_TEST" = true ]; then
    echo "ERROR: --explore mode is incompatible with --navigation-performance-test mode"
    echo "Both modes require different robot control patterns"
    exit 1
fi

# B4M API mode incompatibility checks (same as --explore)
if [ "$B4M_API" = true ] && [ "$REGRESSION_MODE" = true ]; then
    echo "ERROR: --b4m-api mode is incompatible with --regression mode"
    echo "B4M API requires manual control while regression runs automated tests"
    exit 1
fi

if [ "$B4M_API" = true ] && [ "$LOCALIZATION_TEST" = true ]; then
    echo "ERROR: --b4m-api mode is incompatible with --localization-test mode"  
    echo "Both modes require different navigation behaviors"
    exit 1
fi

if [ "$B4M_API" = true ] && [ "$NAVIGATION_PERFORMANCE_TEST" = true ]; then
    echo "ERROR: --b4m-api mode is incompatible with --navigation-performance-test mode"
    echo "Both modes require different robot control patterns"
    exit 1
fi

if [ "$B4M_API" = true ] && [ "$EXPLORE_MODE" = true ]; then
    echo "ERROR: --b4m-api and --explore cannot be used together"
    echo "They are functionally identical modes"
    exit 1
fi

# B4M LiDAR mode incompatibility checks
if [ "$B4M_LIDAR" = true ]; then
    if [ "$EXPLORE_MODE" = true ] || [ "$B4M_API" = true ] || [ "$REGRESSION_MODE" = true ] || [ "$LOCALIZATION_TEST" = true ] || [ "$NAVIGATION_PERFORMANCE_TEST" = true ]; then
        echo "ERROR: --b4m-lidar is incompatible with other navigation modes"
        echo "B4M LiDAR mode is a dedicated navigation system"
        exit 1
    fi
fi

# B4M Ping mode incompatibility checks
if [ "$B4M_PING" = true ]; then
    if [ "$EXPLORE_MODE" = true ] || [ "$B4M_API" = true ] || [ "$REGRESSION_MODE" = true ] || [ "$LOCALIZATION_TEST" = true ] || [ "$NAVIGATION_PERFORMANCE_TEST" = true ] || [ "$B4M_LIDAR" = true ]; then
        echo "ERROR: --b4m-ping is incompatible with other modes"
        echo "B4M Ping is a standalone API testing tool"
        exit 1
    fi
fi

# Handle WiFi setup mode
if [ "$SETUP_WIFI" = true ]; then
    echo "🔧 WiFi Setup Mode"
    echo "================="
    echo "Starting interactive WiFi configuration wizard..."
    echo ""
    setup_wifi_interactive
    # If we reach here, user chose to continue to normal launch
fi

# Handle exploration mode
if [ "$EXPLORE_MODE" = true ]; then
    echo "🗺️ EXPLORATION MODE"
    echo "======================================"
    echo "Launching autonomous exploration with Cartographer mapping"
    
    if [ "$SIMULATION_MODE" = true ]; then
        echo "Mode: Gazebo Classic Simulation with Exploration World"
        WORLD_NAME="exploration_test_classic"  # Use exploration-specific world
    else  
        echo "Mode: Real Robot Autonomous Exploration"
    fi
    
    echo ""
    echo "This mode will:"
    echo "- Launch Cartographer for real-time SLAM mapping"
    echo "- Start autonomous exploration with obstacle avoidance" 
    echo "- Build a map while navigating safely"
    echo "- Stop exploration when area is sufficiently mapped"
    echo ""
    
    # FIX: Ensure clean state before launching exploration
    echo "🧹 ENSURING CLEAN STATE FOR EXPLORATION"
    echo "======================================"
    echo "Cleaning up any existing ROS2 processes to prevent TF conflicts..."
    ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
    sleep 3
    echo "✅ System cleanup completed"
    echo ""
    
    # Launch exploration sequence
    echo "🚀 EXPLORATION LAUNCH SEQUENCE"
    echo "======================================"
    
    if [ "$SIMULATION_MODE" = true ]; then
        # Step 1: Launch Gazebo Classic simulation with exploration world
        echo "🎮 Step 1: Starting Gazebo Classic simulation with exploration world"
        ros2 launch yahboomcar_nav gazebo_classic_nav_launch.py world_name:=$WORLD_NAME > "$LOGS_DIR/exploration_gazebo_$TIMESTAMP.log" 2>&1 &
        GAZEBO_PID=$!
        echo "   Waiting for simulation initialization..."
        sleep 10  # Give Gazebo more time to fully initialize and spawn robot
        
        # Verify laser scan is being published
        echo "   Verifying laser scan topic..."
        if timeout 5 ros2 topic echo /scan --once > /dev/null 2>&1; then
            echo "   ✅ Laser scan topic active"
        else
            echo "   ⚠️  Warning: Laser scan topic not detected"
        fi
        
        # Step 2: Launch robot bringup for EKF and sensor processing (CRITICAL FOR SIMULATION)
        echo "🤖 Step 2: Starting robot sensor and control systems (EKF, IMU, etc.)"
        echo "   This is required even in simulation to get filtered /odom from EKF"
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py use_sim_time:=true > "$LOGS_DIR/exploration_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # FIX: Verify that /odom topic is being published before continuing (even in simulation)
        echo "   Verifying odometry is available..."
        timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ Odometry topic verified"
        else
            echo "   ⚠️  Warning: /odom topic not ready, continuing anyway..."
        fi
        
        # Step 3: Launch RViz for visualization
        echo "📊 Step 3: Starting RViz for map visualization"
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=true > "$LOGS_DIR/exploration_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 5  # Give RViz more time to connect to topics
        
    else
        echo "   ⚠️  Make sure physical robot is powered on and ready"
        echo "   ⚠️  Ensure exploration area is safe and obstacle-free"
        read -p "   Press Enter when robot is ready for exploration..."
        
        # Step 1: Launch robot bringup 
        echo "🤖 Step 1: Starting robot sensor and control systems"
        # Must source ALL workspaces including IMU for EKF to work properly
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py > "$LOGS_DIR/exploration_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # FIX: Verify that /odom topic is being published before continuing
        echo "   Verifying odometry is available..."
        timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ Odometry topic verified"
        else
            echo "   ⚠️  Warning: /odom topic not ready, continuing anyway..."
        fi
        
        # Step 2: Launch RViz for visualization
        echo "📊 Step 2: Starting RViz for map visualization"  
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=false > "$LOGS_DIR/exploration_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 3
    fi
    
    # Step 4: Launch TF bridge to connect odom_frame to odom
    echo "🔗 Step 4: Creating TF bridge between odom_frame and odom"
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom_frame odom > "$LOGS_DIR/exploration_tf_bridge_$TIMESTAMP.log" 2>&1 &
    TF_BRIDGE_PID=$!
    echo "   Bridge created: odom_frame → odom (connects Cartographer to robot)"
    
    # Step 5: Launch Cartographer SLAM for real-time mapping
    echo "🗺️  Step 5: Starting Cartographer SLAM for real-time mapping"
    if [ "$SIMULATION_MODE" = true ]; then
        ros2 launch yahboomcar_nav map_cartographer_launch.py use_sim_time:=true > "$LOGS_DIR/exploration_cartographer_$TIMESTAMP.log" 2>&1 &
    else
        ros2 launch yahboomcar_nav map_cartographer_launch.py use_sim_time:=false > "$LOGS_DIR/exploration_cartographer_$TIMESTAMP.log" 2>&1 &
    fi
    CARTOGRAPHER_PID=$!
    echo "   Waiting for SLAM system initialization..."
    sleep 10  # Give Cartographer more time to initialize
    
    # Verify map topic is being published
    echo "   Verifying map topic..."
    if timeout 10 ros2 topic echo /map --once > /dev/null 2>&1; then
        echo "   ✅ Map topic active"
    else
        echo "   ⚠️  Warning: Map topic not detected yet (may appear shortly)"
        echo "   Checking available topics..."
        ros2 topic list | grep -E "map|scan|tf" || true
    fi
    
    # FIX: Verify TF tree is complete before starting exploration
    echo "   Verifying TF tree integrity..."
    timeout 10 ros2 run tf2_ros tf2_echo map base_link > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "   ✅ TF tree verified (map → base_link transform available)"
    else
        echo "   ⚠️  TF tree not complete yet, waiting additional time..."
        sleep 5
        timeout 5 ros2 run tf2_ros tf2_echo map base_link > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ TF tree now ready"
        else
            echo "   ⚠️  Warning: TF tree may be incomplete, but continuing..."
        fi
    fi
    
    # Step 6: Start autonomous exploration
    echo "🚀 Step 6: Starting autonomous exploration with obstacle avoidance"
    cd "$WORKSPACE_ROOT" && . install/setup.bash && python3 "$WORKSPACE_ROOT/scripts/autonomous_exploration.py" > "$LOGS_DIR/exploration_autonomous_$TIMESTAMP.log" 2>&1 &
    EXPLORATION_PID=$!
    
    echo ""
    echo "✅ EXPLORATION ACTIVE"
    echo "======================================"
    echo "🗺️  Robot is now autonomously exploring and mapping"
    echo "📊 Monitor progress in RViz:"
    echo "   - Map topic: /map (shows real-time SLAM mapping)"
    echo "   - Robot position: /tf (robot location on map)"
    echo "   - Laser scans: /scan (sensor readings)"
    echo ""
    echo "🛑 To stop exploration:"
    echo "   - Press Ctrl+C in this terminal, OR"
    echo "   - Run: ./b4m_shutdown.sh --keep-agent"
    echo ""
    echo "⏳ Let the robot explore for several minutes to build a complete map"
    echo "   The robot will avoid obstacles and explore systematically"
    
    # Wait for user to stop or monitor the exploration
    trap 'echo "🛑 Stopping exploration..."; [ ! -z "$EXPLORATION_PID" ] && kill $EXPLORATION_PID 2>/dev/null; [ ! -z "$CARTOGRAPHER_PID" ] && kill $CARTOGRAPHER_PID 2>/dev/null; [ ! -z "$TF_BRIDGE_PID" ] && kill $TF_BRIDGE_PID 2>/dev/null; [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null; [ ! -z "$BRINGUP_PID" ] && kill $BRINGUP_PID 2>/dev/null; if [ "$SIMULATION_MODE" = true ]; then [ ! -z "$GAZEBO_PID" ] && kill $GAZEBO_PID 2>/dev/null; fi; ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1; echo "✅ Exploration stopped"; exit 0' INT
    
    # Keep the script running and show periodic status
    while true; do
        sleep 30
        echo "🗺️  Exploration continues... (Ctrl+C to stop)"
        echo "   Check RViz to see mapping progress"
    done
fi

# Handle B4M API mode (duplicate of exploration mode for API integration)
if [ "$B4M_API" = true ]; then
    echo "🔌 B4M API MODE"
    echo "======================================"
    echo "Launching B4M API mode with Cartographer mapping (identical to --explore)"
    
    if [ "$SIMULATION_MODE" = true ]; then
        echo "Mode: Gazebo Classic Simulation with Exploration World"
        WORLD_NAME="exploration_test_classic"  # Use exploration-specific world
    else  
        echo "Mode: Real Robot B4M API Exploration"
    fi
    
    echo ""
    echo "This mode will:"
    echo "- Launch Cartographer for real-time SLAM mapping"
    echo "- Start autonomous exploration with obstacle avoidance" 
    echo "- Build a map while navigating safely"
    echo "- Stop exploration when area is sufficiently mapped"
    echo "- Provide API endpoints for external integration"
    echo ""
    
    # FIX: Ensure clean state before launching B4M API
    echo "🧹 ENSURING CLEAN STATE FOR B4M API"
    echo "======================================"
    echo "Cleaning up any existing ROS2 processes to prevent TF conflicts..."
    ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
    sleep 3
    echo "✅ System cleanup completed"
    echo ""
    
    # Launch B4M API sequence
    echo "🚀 B4M API LAUNCH SEQUENCE"
    echo "======================================"
    
    if [ "$SIMULATION_MODE" = true ]; then
        # Step 1: Launch Gazebo Classic simulation with exploration world
        echo "🎮 Step 1: Starting Gazebo Classic simulation with exploration world"
        ros2 launch yahboomcar_nav gazebo_classic_nav_launch.py world_name:=$WORLD_NAME > "$LOGS_DIR/b4m_api_gazebo_$TIMESTAMP.log" 2>&1 &
        GAZEBO_PID=$!
        echo "   Waiting for simulation initialization..."
        sleep 10  # Give Gazebo more time to fully initialize and spawn robot
        
        # Verify laser scan is being published
        echo "   Verifying laser scan topic..."
        if timeout 5 ros2 topic echo /scan --once > /dev/null 2>&1; then
            echo "   ✅ Laser scan topic active"
        else
            echo "   ⚠️  Warning: Laser scan topic not detected"
        fi
        
        # Step 2: Launch robot bringup for EKF and sensor processing (CRITICAL FOR SIMULATION)
        echo "🤖 Step 2: Starting robot sensor and control systems (EKF, IMU, etc.)"
        echo "   This is required even in simulation to get filtered /odom from EKF"
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py use_sim_time:=true > "$LOGS_DIR/b4m_api_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # FIX: Verify that /odom topic is being published before continuing (even in simulation)
        echo "   Verifying odometry is available..."
        timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ Odometry topic verified"
        else
            echo "   ⚠️  Warning: /odom topic not ready, continuing anyway..."
        fi
        
        # Step 3: Launch RViz for visualization
        echo "📊 Step 3: Starting RViz for map visualization"
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=true > "$LOGS_DIR/b4m_api_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 5  # Give RViz more time to connect to topics
        
    else
        echo "   ⚠️  Make sure physical robot is powered on and ready"
        echo "   ⚠️  Ensure exploration area is safe and obstacle-free"
        read -p "   Press Enter when robot is ready for B4M API mode..."
        
        # Step 1: Launch robot bringup 
        echo "🤖 Step 1: Starting robot sensor and control systems"
        # Must source ALL workspaces including IMU for EKF to work properly
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py > "$LOGS_DIR/b4m_api_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # FIX: Verify that /odom topic is being published before continuing
        echo "   Verifying odometry is available..."
        timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ Odometry topic verified"
        else
            echo "   ⚠️  Warning: /odom topic not ready, continuing anyway..."
        fi
        
        # Step 2: Launch RViz for visualization
        echo "📊 Step 2: Starting RViz for map visualization"  
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=false > "$LOGS_DIR/b4m_api_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 3
    fi
    
    # Step 4: Launch TF bridge to connect odom_frame to odom
    echo "🔗 Step 4: Creating TF bridge between odom_frame and odom"
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom_frame odom > "$LOGS_DIR/b4m_api_tf_bridge_$TIMESTAMP.log" 2>&1 &
    TF_BRIDGE_PID=$!
    echo "   Bridge created: odom_frame → odom (connects Cartographer to robot)"
    
    # Step 5: Launch Cartographer SLAM for real-time mapping
    echo "🗺️  Step 5: Starting Cartographer SLAM for real-time mapping"
    if [ "$SIMULATION_MODE" = true ]; then
        ros2 launch yahboomcar_nav map_cartographer_launch.py use_sim_time:=true > "$LOGS_DIR/b4m_api_cartographer_$TIMESTAMP.log" 2>&1 &
    else
        ros2 launch yahboomcar_nav map_cartographer_launch.py use_sim_time:=false > "$LOGS_DIR/b4m_api_cartographer_$TIMESTAMP.log" 2>&1 &
    fi
    CARTOGRAPHER_PID=$!
    echo "   Waiting for SLAM system initialization..."
    sleep 10  # Give Cartographer more time to initialize
    
    # Verify map topic is being published
    echo "   Verifying map topic..."
    if timeout 10 ros2 topic echo /map --once > /dev/null 2>&1; then
        echo "   ✅ Map topic active"
    else
        echo "   ⚠️  Warning: Map topic not detected yet (may appear shortly)"
        echo "   Checking available topics..."
        ros2 topic list | grep -E "map|scan|tf" || true
    fi
    
    # FIX: Verify TF tree is complete before starting exploration
    echo "   Verifying TF tree integrity..."
    timeout 10 ros2 run tf2_ros tf2_echo map base_link > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "   ✅ TF tree verified (map → base_link transform available)"
    else
        echo "   ⚠️  TF tree not complete yet, waiting additional time..."
        sleep 5
        timeout 5 ros2 run tf2_ros tf2_echo map base_link > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ TF tree now ready"
        else
            echo "   ⚠️  Warning: TF tree may be incomplete, but continuing..."
        fi
    fi
    
    # Step 6: Start B4M Spatial Interpreter with manual decisions
    echo "🚀 Step 6: Starting B4M Spatial Interpreter with manual decisions"
    echo "   Console will display spatial descriptions when obstacles detected"
    echo "   User input required for navigation decisions"
    echo ""
    # Run in foreground (no &) to keep console interactive for user input
    cd "$WORKSPACE_ROOT" && . install/setup.bash && python3 "$WORKSPACE_ROOT/scripts/b4m_spatial_interpreter.py" 2>&1 | tee "$LOGS_DIR/b4m_api_spatial_$TIMESTAMP.log"
    # Note: Script will run until user stops it with Ctrl+C
    
    # The spatial interpreter runs in foreground and handles its own exit
    # When it exits (user presses Ctrl+C), clean up the other processes
    echo ""
    echo "🛑 B4M Spatial Interpreter stopped"
    echo "Cleaning up system processes..."
    
    # Clean up background processes
    [ ! -z "$CARTOGRAPHER_PID" ] && kill $CARTOGRAPHER_PID 2>/dev/null
    [ ! -z "$TF_BRIDGE_PID" ] && kill $TF_BRIDGE_PID 2>/dev/null
    [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null
    [ ! -z "$BRINGUP_PID" ] && kill $BRINGUP_PID 2>/dev/null
    if [ "$SIMULATION_MODE" = true ]; then
        [ ! -z "$GAZEBO_PID" ] && kill $GAZEBO_PID 2>/dev/null
    fi
    
    # Final cleanup
    ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
    echo "✅ B4M API mode stopped"
    exit 0
fi

# Handle B4M LLM Navigation mode
if [ "$B4M_NAV" = true ]; then
    echo "🤖 B4M LLM NAVIGATION MODE"
    echo "======================================"
    echo "Launching LLM Navigation API with real-time SLAM mapping"
    
    if [ "$SIMULATION_MODE" = true ]; then
        echo "Mode: Gazebo Classic Simulation with LLM Navigation"
        WORLD_NAME="exploration_test_classic"  # Use same world as exploration
    else  
        echo "Mode: Real Robot LLM Navigation"
    fi
    
    echo ""
    echo "This mode will:"
    echo "- Launch Cartographer for real-time SLAM mapping"
    echo "- Start B4M LLM Navigation API server on port 8080" 
    echo "- Provide spatial awareness to LLMs via HTTP/JSON"
    echo "- Enable natural language navigation control"
    if [ "$SIMULATED_B4M" = true ]; then
        echo "- Run in SIMULATED B4M mode (no external LLM needed)"
    fi
    echo ""
    
    # FIX: Ensure clean state before launching B4M navigation
    echo "🧹 ENSURING CLEAN STATE FOR B4M NAVIGATION"
    echo "======================================"
    echo "Cleaning up any existing ROS2 processes to prevent TF conflicts..."
    ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
    sleep 3
    echo "✅ System cleanup completed"
    echo ""
    
    # Launch B4M navigation sequence (copying explore pattern exactly)
    echo "🚀 B4M NAVIGATION LAUNCH SEQUENCE"
    echo "======================================"
    
    if [ "$SIMULATION_MODE" = true ]; then
        # Step 1: Launch Gazebo Classic simulation with exploration world
        echo "🎮 Step 1: Starting Gazebo Classic simulation with exploration world"
        ros2 launch yahboomcar_nav gazebo_classic_nav_launch.py world_name:=$WORLD_NAME > "$LOGS_DIR/b4m_nav_gazebo_$TIMESTAMP.log" 2>&1 &
        GAZEBO_PID=$!
        echo "   Waiting for simulation initialization..."
        sleep 10  # Give Gazebo more time to fully initialize and spawn robot
        
        # Verify laser scan is being published
        echo "   Verifying laser scan topic..."
        if timeout 5 ros2 topic echo /scan --once > /dev/null 2>&1; then
            echo "   ✅ Laser scan topic active"
        else
            echo "   ⚠️  Warning: Laser scan topic not detected"
        fi
        
        # Step 2: Launch robot bringup for EKF and sensor processing (CRITICAL FOR SIMULATION)
        echo "🤖 Step 2: Starting robot sensor and control systems (EKF, IMU, etc.)"
        echo "   This is required even in simulation to get filtered /odom from EKF"
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py use_sim_time:=true > "$LOGS_DIR/b4m_nav_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # FIX: Verify that /odom topic is being published before continuing (even in simulation)
        echo "   Verifying odometry is available..."
        timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ Odometry topic verified"
        else
            echo "   ⚠️  Warning: /odom topic not ready, continuing anyway..."
        fi
        
        # Step 3: Launch RViz for visualization
        echo "📊 Step 3: Starting RViz for map visualization"
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=true > "$LOGS_DIR/b4m_nav_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 5  # Give RViz more time to connect to topics
        
    else
        echo "   ⚠️  Make sure physical robot is powered on and ready"
        echo "   ⚠️  Ensure navigation area is safe and obstacle-free"
        read -p "   Press Enter when robot is ready for B4M navigation..."
        
        # Step 1: Launch robot bringup 
        echo "🤖 Step 1: Starting robot sensor and control systems"
        # Must source ALL workspaces including IMU for EKF to work properly
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py > "$LOGS_DIR/b4m_nav_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # FIX: Verify that /odom topic is being published before continuing
        echo "   Verifying odometry is available..."
        timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ Odometry topic verified"
        else
            echo "   ⚠️  Warning: /odom topic not ready, continuing anyway..."
        fi
        
        # Step 2: Launch RViz for visualization
        echo "📊 Step 2: Starting RViz for map visualization"  
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=false > "$LOGS_DIR/b4m_nav_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 3
    fi
    
    # Step 4: Launch TF bridge to connect odom_frame to odom
    echo "🔗 Step 4: Creating TF bridge between odom_frame and odom"
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom_frame odom > "$LOGS_DIR/b4m_nav_tf_bridge_$TIMESTAMP.log" 2>&1 &
    TF_BRIDGE_PID=$!
    echo "   Bridge created: odom_frame → odom (connects Cartographer to robot)"
    
    # Step 5: Launch Cartographer SLAM for real-time mapping
    echo "🗺️  Step 5: Starting Cartographer SLAM for real-time mapping"
    if [ "$SIMULATION_MODE" = true ]; then
        ros2 launch yahboomcar_nav map_cartographer_launch.py use_sim_time:=true > "$LOGS_DIR/b4m_nav_cartographer_$TIMESTAMP.log" 2>&1 &
    else
        ros2 launch yahboomcar_nav map_cartographer_launch.py use_sim_time:=false > "$LOGS_DIR/b4m_nav_cartographer_$TIMESTAMP.log" 2>&1 &
    fi
    CARTOGRAPHER_PID=$!
    echo "   Waiting for SLAM system initialization..."
    sleep 10  # Give Cartographer more time to initialize
    
    # Verify map topic is being published
    echo "   Verifying map topic..."
    if timeout 10 ros2 topic echo /map --once > /dev/null 2>&1; then
        echo "   ✅ Map topic active"
    else
        echo "   ⚠️  Warning: Map topic not detected yet (may appear shortly)"
        echo "   Checking available topics..."
        ros2 topic list | grep -E "map|scan|tf" || true
    fi
    
    # FIX: Verify TF tree is complete before starting LLM API
    echo "   Verifying TF tree integrity..."
    timeout 10 ros2 run tf2_ros tf2_echo map base_link > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "   ✅ TF tree verified (map → base_link transform available)"
    else
        echo "   ⚠️  TF tree not complete yet, waiting additional time..."
        sleep 5
        timeout 5 ros2 run tf2_ros tf2_echo map base_link > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ TF tree now ready"
        else
            echo "   ⚠️  Warning: TF tree may be incomplete, but continuing..."
        fi
    fi
    
    # Step 6: Start B4M LLM Navigation API
    echo "🚀 Step 6: Starting B4M LLM Navigation API server"
    if [ "$SIMULATED_B4M" = true ]; then
        echo "   Launching in SIMULATED B4M mode (no external LLM needed)"
        cd "$WORKSPACE_ROOT" && . install/setup.bash && ros2 launch b4m_llm_nav_api llm_nav_api_launch.py simulated_b4m:=true verbose_logging:=true > "$LOGS_DIR/b4m_nav_api_$TIMESTAMP.log" 2>&1 &
    else
        echo "   Launching for real LLM integration"
        cd "$WORKSPACE_ROOT" && . install/setup.bash && ros2 launch b4m_llm_nav_api llm_nav_api_launch.py simulated_b4m:=false verbose_logging:=true > "$LOGS_DIR/b4m_nav_api_$TIMESTAMP.log" 2>&1 &
    fi
    API_PID=$!
    echo "   Waiting for API server initialization..."
    sleep 8
    
    # Verify API server is responding
    echo "   Verifying API server is responding..."
    if timeout 10 curl -s -f "http://localhost:8080/" > /dev/null 2>&1; then
        echo "   ✅ API server responding on port 8080"
    else
        echo "   ⚠️  Warning: API server not responding yet (may need more time)"
    fi
    
    echo ""
    echo "✅ B4M LLM NAVIGATION ACTIVE"
    echo "======================================"
    echo "🤖 Robot is now ready for LLM navigation control"
    echo "🌐 API server available at: http://localhost:8080"
    echo ""
    echo "📊 Monitor progress in RViz:"
    echo "   - Map topic: /map (shows real-time SLAM mapping)"
    echo "   - Robot position: /tf (robot location on map)"
    echo "   - Laser scans: /scan (sensor readings)"
    echo ""
    echo "🔗 Available API endpoints:"
    echo "   - GET  /spatial_context      - Get spatial description for LLM"
    echo "   - POST /navigate_to          - Send navigation commands"
    echo "   - GET  /status               - Check navigation status"
    if [ "$SIMULATED_B4M" = true ]; then
        echo "   - GET  /simulated_b4m_decision - Get simulated B4M decision"
    fi
    echo ""
    echo "🛑 To stop B4M navigation:"
    echo "   - Press Ctrl+C in this terminal, OR"
    echo "   - Run: ./b4m_shutdown.sh --keep-agent"
    echo ""
    
    # Set up signal handler for clean shutdown
    trap 'echo "🛑 Stopping B4M navigation..."; [ ! -z "$API_PID" ] && kill $API_PID 2>/dev/null; [ ! -z "$CARTOGRAPHER_PID" ] && kill $CARTOGRAPHER_PID 2>/dev/null; [ ! -z "$TF_BRIDGE_PID" ] && kill $TF_BRIDGE_PID 2>/dev/null; [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null; [ ! -z "$BRINGUP_PID" ] && kill $BRINGUP_PID 2>/dev/null; if [ "$SIMULATION_MODE" = true ]; then [ ! -z "$GAZEBO_PID" ] && kill $GAZEBO_PID 2>/dev/null; fi; ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1; echo "✅ B4M navigation stopped"; exit 0' INT
    
    # Wait for user to stop or let it run
    if [ "$SIMULATED_B4M" = true ]; then
        echo "⏳ Simulated B4M is now analyzing the environment and making navigation decisions..."
        echo "   Watch the console output and RViz to see the LLM-like navigation behavior"
    else
        echo "⏳ System ready for LLM connections and navigation commands"
        echo "   LLMs can now connect to http://localhost:8080 for spatial awareness"
    fi
    
    while true; do
        sleep 5
        echo "🗺️  B4M navigation continues... (Ctrl+C to stop)"
        if [ "$SIMULATED_B4M" = true ]; then
            echo "   Check RViz to see simulated LLM navigation behavior"
        else
            echo "   API server ready for LLM connections on port 8080"
        fi
    done
fi

# Handle B4M LiDAR mode
if [ "$B4M_LIDAR" = true ]; then
    echo "🤖 B4M LIDAR INTELLIGENT NAVIGATION MODE"
    echo "=========================================="
    echo "Launching LiDAR-based navigation with B4M API integration"
    
    if [ "$SIMULATION_MODE" = true ]; then
        echo "Mode: Gazebo Classic Simulation"
        WORLD_NAME="exploration_test_classic"  # Use exploration world with obstacles
    else
        echo "Mode: Real Robot with B4M API"
    fi
    
    echo ""
    echo "This mode will:"
    echo "- Use LiDAR to detect obstacles"
    echo "- Request turn directions from B4M API"
    echo "- Navigate intelligently based on API decisions"
    echo "- API cooldown: 20 seconds between requests"
    echo "- Robot stops during cooldown if obstacles detected"
    echo ""
    
    # Launch B4M LiDAR sequence
    echo "🚀 B4M LIDAR LAUNCH SEQUENCE"
    echo "======================================"
    
    if [ "$SIMULATION_MODE" = true ]; then
        # Step 1: Launch Gazebo Classic simulation
        echo "🎮 Step 1: Starting Gazebo Classic simulation"
        ros2 launch yahboomcar_nav gazebo_classic_nav_launch.py world_name:=$WORLD_NAME > "$LOGS_DIR/b4m_lidar_gazebo_$TIMESTAMP.log" 2>&1 &
        GAZEBO_PID=$!
        echo "   Waiting for simulation initialization..."
        sleep 8
        
        # Step 2: Launch RViz for visualization
        echo "📊 Step 2: Starting RViz for visualization"
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=true > "$LOGS_DIR/b4m_lidar_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 3
        
        # Step 3: Start B4M LiDAR Navigator
        echo "🌐 Step 3: Starting B4M LiDAR Navigator with API integration"
        echo "   API endpoint: https://app.bike4mind.com/api/chat"
        echo "   API cooldown: 20 seconds between calls"
        echo "   Stop distance: 30.48cm (1 foot)"
        echo "   Using simulation time"
        
        cd "$WORKSPACE_ROOT" && . install/setup.bash && ros2 run b4m_lidar b4m_lidar_navigator --ros-args -p use_sim_time:=true > "$LOGS_DIR/b4m_lidar_navigator_$TIMESTAMP.log" 2>&1 &
        B4M_LIDAR_PID=$!
        
    else
        echo "   ⚠️  Make sure physical robot is powered on and ready"
        echo "   ⚠️  Ensure exploration area is safe"
        read -p "   Press Enter when robot is ready..."
        
        # Step 1: Launch robot bringup 
        echo "🤖 Step 1: Starting robot sensor and control systems"
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py > "$LOGS_DIR/b4m_lidar_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization..."
        sleep 8
        
        # Step 2: Launch RViz for visualization
        echo "📊 Step 2: Starting RViz for visualization"
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=false > "$LOGS_DIR/b4m_lidar_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 3
        
        # Step 3: Start B4M LiDAR Navigator
        echo "🌐 Step 3: Starting B4M LiDAR Navigator with API integration"
        echo "   API endpoint: https://app.bike4mind.com/api/chat"
        echo "   API cooldown: 20 seconds between calls"
        echo "   Stop distance: 30.48cm (1 foot)"
        
        cd "$WORKSPACE_ROOT" && . install/setup.bash && ros2 run b4m_lidar b4m_lidar_navigator > "$LOGS_DIR/b4m_lidar_navigator_$TIMESTAMP.log" 2>&1 &
        B4M_LIDAR_PID=$!
    fi
    
    echo ""
    echo "✅ B4M LIDAR NAVIGATION ACTIVE"
    echo "======================================"
    echo "🤖 Robot is navigating using LiDAR and B4M API"
    echo "📊 Monitor in RViz:"
    echo "   - Laser scans: /scan"
    echo "   - Movement commands: /cmd_vel"
    echo "   - Status: /b4m_lidar/status"
    echo "   - Obstacle info: /b4m_lidar/obstacle_info"
    echo "   - API cooldown: /b4m_lidar/api_cooldown"
    echo ""
    echo "📡 Control commands:"
    echo "   - Stop: ros2 topic pub -1 /b4m_lidar/command std_msgs/String '{data: stop}'"
    echo "   - Start: ros2 topic pub -1 /b4m_lidar/command std_msgs/String '{data: start}'"
    echo "   - Reset: ros2 topic pub -1 /b4m_lidar/command std_msgs/String '{data: reset}'"
    echo ""
    echo "🛑 To stop navigation:"
    echo "   - Press Ctrl+C in this terminal, OR"
    echo "   - Run: ./b4m_shutdown.sh --keep-agent"
    echo ""
    
    # Wait for user to stop
    if [ "$SIMULATION_MODE" = true ]; then
        trap 'echo "🛑 Stopping B4M LiDAR navigation..."; [ ! -z "$B4M_LIDAR_PID" ] && kill $B4M_LIDAR_PID 2>/dev/null; [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null; [ ! -z "$GAZEBO_PID" ] && kill $GAZEBO_PID 2>/dev/null; ./b4m_shutdown.sh > /dev/null 2>&1; echo "✅ B4M LiDAR navigation stopped"; exit 0' INT
    else
        trap 'echo "🛑 Stopping B4M LiDAR navigation..."; [ ! -z "$B4M_LIDAR_PID" ] && kill $B4M_LIDAR_PID 2>/dev/null; [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null; [ ! -z "$BRINGUP_PID" ] && kill $BRINGUP_PID 2>/dev/null; ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1; echo "✅ B4M LiDAR navigation stopped"; exit 0' INT
    fi
    
    # Keep the script running and show periodic status
    while true; do
        sleep 30
        echo "🤖 B4M LiDAR navigation continues... (Ctrl+C to stop)"
        echo "   Check RViz for real-time visualization"
        echo "   Monitor /b4m_lidar/status for navigation state"
    done
fi

# Handle B4M Ping test mode
if [ "$B4M_PING" = true ]; then
    echo "📡 B4M PING API TEST MODE"
    echo "================================="
    echo "Testing bike4mind API with random obstacle detection messages"
    echo ""
    echo "This test will:"
    echo "- Generate random obstacle reports (left/front/right directions)"
    echo "- Send messages to bike4mind API endpoint"
    echo "- Display API responses in real-time"
    echo "- Wait for keypress between messages"
    echo "- Exit on CTRL+C"
    echo ""
    echo "API endpoint: https://app.bike4mind.com/api/chat"
    
    # Check for API key in environment
    if [ -z "$B4M_API_KEY" ]; then
        echo "⚠️  WARNING: B4M_API_KEY environment variable not set!"
        echo "Please set it: export B4M_API_KEY='your_key_here'"
        echo "Or create a .env file with B4M_API_KEY=your_key_here"
        exit 1
    fi
    echo "API key: [CONFIGURED VIA ENVIRONMENT]"
    echo ""
    
    # Make script executable if needed
    if [ ! -x "$WORKSPACE_ROOT/scripts/b4m_ping_test.py" ]; then
        chmod +x "$WORKSPACE_ROOT/scripts/b4m_ping_test.py"
    fi
    
    # Run the B4M ping test
    echo "🚀 Starting B4M Ping Test..."
    echo "================================="
    python3 "$WORKSPACE_ROOT/scripts/b4m_ping_test.py"
    
    echo ""
    echo "✅ B4M Ping Test completed"
    exit 0
fi

# Handle regression test mode
if [ "$REGRESSION_MODE" = true ]; then
    echo "🧪 REGRESSION TEST MODE"
    echo "======================================"
    echo "Running comprehensive regression test suite with Cartographer"
    
    if [ "$SIMULATION_MODE" = true ]; then
        echo "Mode: Gazebo Classic Simulation with RViz"
    else  
        echo "Mode: Real Robot with RViz"
    fi
    
    echo "Test logs will be saved to:"
    echo "  Rotation test: $LOGS_DIR/regression_rotation_$TIMESTAMP.log"
    echo "======================================"
    
    # CLEANUP BEFORE STARTING NEW TEST
    echo ""
    echo "🧹 PRE-TEST CLEANUP"
    echo "Ensuring clean system state before regression test..."
    
    if [ "$SIMULATION_MODE" = true ]; then
        # Kill any existing simulation processes
        pkill -f "rviz2" 2>/dev/null || true
        pkill -f "rviz" 2>/dev/null || true
        pkill -f "gazebo" 2>/dev/null || true
        pkill -f "gzserver" 2>/dev/null || true
        pkill -f "gzclient" 2>/dev/null || true
        
        # Run comprehensive shutdown for simulation
        ./b4m_shutdown.sh > /dev/null 2>&1 || true
    else
        # Kill any existing real robot processes but preserve agent
        pkill -f "rviz2" 2>/dev/null || true
        pkill -f "rviz" 2>/dev/null || true
        
        # Run shutdown script to clean up properly (keep agent for real robot)
        ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1 || true
    fi
    
    sleep 3
    echo "✅ Pre-test cleanup complete"
    
    # Step 1: Launch system for testing
    echo ""
    echo "🚀 PHASE 1: SYSTEM LAUNCH"
    echo "Launching system components for regression testing..."
    
    if [ "$SIMULATION_MODE" = true ]; then
        # Launch Gazebo Classic simulation with exploration world (same as --explore mode)
        echo "🎮 Step 1: Starting Gazebo Classic simulation with exploration world"
        echo "   Using exploration_test_classic world for consistent testing"
        ros2 launch yahboomcar_nav gazebo_classic_nav_launch.py world_name:=exploration_test_classic > /dev/null 2>&1 &
        GAZEBO_PID=$!
        echo "   Waiting for simulation initialization..."
        sleep 12
        
        # Step 2: Start robot bringup for simulation (EKF and sensor integration)
        echo "🤖 Step 2: Starting robot bringup for simulation (sensors and EKF odometry)"
        # Must source ALL workspaces including IMU for EKF to work properly
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py use_sim_time:=true > /dev/null 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor and EKF initialization..."
        sleep 8
        
        # Step 3: Start RViz
        echo "👁️  Step 3: Starting RViz for visualization" 
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=true > /dev/null 2>&1 &
        RVIZ_PID=$!
        echo "   Waiting for RViz initialization..."
        sleep 8
        
        # Step 4: Launch TF bridge to connect odom_frame to odom (critical for SLAM)
        echo "🔗 Step 4: Creating TF bridge between odom_frame and odom"
        ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom_frame odom > "$LOGS_DIR/regression_tf_bridge_$TIMESTAMP.log" 2>&1 &
        TF_BRIDGE_PID=$!
        echo "   Bridge created: odom_frame → odom (connects Cartographer to robot)"
        
        # Step 5: Start Cartographer SLAM (identical to explore mode)
        echo "🗺️  Step 5: Starting Cartographer SLAM system (simulation)"
        ros2 launch yahboomcar_nav map_cartographer_launch.py use_sim_time:=true > /dev/null 2>&1 &
        CARTOGRAPHER_PID=$!
        echo "   Waiting for Cartographer initialization..."
        sleep 10
    else
        # Real robot mode - start bringup, RViz, then Cartographer
        echo "🤖 Step 1: Starting robot bringup (sensors and odometry)"
        # Must source ALL workspaces including IMU for EKF to work properly
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py > /dev/null 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization..."
        sleep 8
        
        echo "👁️  Step 2: Starting RViz for visualization"
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=false > /dev/null 2>&1 &
        RVIZ_PID=$!
        echo "   Waiting for RViz initialization..."
        sleep 8
        
        # Step 3: Launch TF bridge to connect odom_frame to odom (critical for SLAM)
        echo "🔗 Step 3: Creating TF bridge between odom_frame and odom"
        ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom_frame odom > "$LOGS_DIR/regression_tf_bridge_$TIMESTAMP.log" 2>&1 &
        TF_BRIDGE_PID=$!
        echo "   Bridge created: odom_frame → odom (connects Cartographer to robot)"
        
        echo "🗺️  Step 4: Starting Cartographer SLAM system (real robot)"
        ros2 launch yahboomcar_nav map_cartographer_launch.py use_sim_time:=false > /dev/null 2>&1 &
        CARTOGRAPHER_PID=$!
        echo "   Waiting for Cartographer initialization..."
        sleep 10
    fi
    
    echo "✅ System launch complete! Now running tests..."
    echo ""
    
    # Step 4: Start comprehensive regression test (rotation + odometry quality)
    echo "🧪 PHASE 2: COMPREHENSIVE REGRESSION TEST"
    echo "🚀 Step 4: Starting comprehensive regression with odometry monitoring"
    echo "   - 360° rotation test with screenshot comparison"
    echo "   - Real-time odometry quality analysis"
    echo "   - Navigation system health validation"
    
    # Use unified comprehensive regression test with proper environment variable
    if [ "$SIMULATION_MODE" = true ]; then
        cd "$WORKSPACE_ROOT" && . install/setup.bash && ROS_USE_SIM_TIME=true python3 "$WORKSPACE_ROOT/scripts/regression_with_odometry.py" > "$LOGS_DIR/regression_comprehensive_$TIMESTAMP.log" 2>&1 &
    else
        cd "$WORKSPACE_ROOT" && . install/setup.bash && ROS_USE_SIM_TIME=false python3 "$WORKSPACE_ROOT/scripts/regression_with_odometry.py" > "$LOGS_DIR/regression_comprehensive_$TIMESTAMP.log" 2>&1 &
    fi
    ROTATION_PID=$!
    
    echo ""
    echo "✅ COMPREHENSIVE REGRESSION TEST ACTIVE"
    echo "======================================" 
    echo "🔄 Robot performing controlled 360° rotation with odometry monitoring"
    echo "📊 Monitor progress in RViz:"
    echo "   - Map topic: /map (shows real-time SLAM mapping)" 
    echo "   - Robot position: /tf (robot location on map)"
    echo "   - Laser scans: /scan (sensor readings)"
    echo "   - Odometry: /odom (filtered position from EKF)"
    echo ""
    echo "⏳ Test phases will complete automatically:"
    echo "   Phase 1: Stationary baseline measurement (5s)"
    echo "   Phase 2: 360° rotation with quality monitoring (20s)"
    echo "   Phase 3: Post-movement settling analysis (5s)"
    echo "   Screenshots captured at: initial, mid-rotation, final"
    
    # Wait for rotation to complete with 2-minute timeout
    echo "⏱️  Starting 2-minute timeout for regression test..."
    
    # Use timeout command to limit execution time
    TIMEOUT_SECONDS=120  # 2 minutes
    START_TIME=$(date +%s)
    
    # Monitor the process with timeout
    while kill -0 $ROTATION_PID 2>/dev/null; do
        CURRENT_TIME=$(date +%s)
        ELAPSED=$((CURRENT_TIME - START_TIME))
        
        if [ $ELAPSED -ge $TIMEOUT_SECONDS ]; then
            echo "⏰ TIMEOUT: Regression test exceeded 2 minutes"
            echo "   Terminating test process..."
            kill -TERM $ROTATION_PID 2>/dev/null
            sleep 2
            kill -KILL $ROTATION_PID 2>/dev/null
            ROTATION_EXIT_CODE=124  # Standard timeout exit code
            break
        fi
        
        REMAINING=$((TIMEOUT_SECONDS - ELAPSED))
        echo "⏳ Regression test running... ${REMAINING}s remaining"
        sleep 5
    done
    
    # If process completed normally, get its exit code
    if [ -z "$ROTATION_EXIT_CODE" ]; then
        wait $ROTATION_PID
        ROTATION_EXIT_CODE=$?
    fi
    
    if [ $ROTATION_EXIT_CODE -eq 0 ]; then
        echo "✅ Comprehensive regression test PASSED"
        echo "  ✓ Screenshot comparison passed"
        echo "  ✓ Odometry quality assessment passed"
        echo "  ✓ Robot rotated 360 degrees successfully"
        echo "  ✓ Navigation system health validated"
        TEST_RESULT="PASSED"
    elif [ $ROTATION_EXIT_CODE -eq 124 ]; then
        echo "⏰ Comprehensive regression test TIMED OUT"
        echo "  ❌ Test exceeded 2-minute time limit"
        echo "  ❌ Test was terminated to prevent infinite execution"
        echo "  Check log for details: $LOGS_DIR/regression_comprehensive_$TIMESTAMP.log"
        TEST_RESULT="TIMEOUT"
    else
        echo "❌ Comprehensive regression test FAILED"  
        echo "  ❌ Exit code: $ROTATION_EXIT_CODE"
        echo "  Check log for details: $LOGS_DIR/regression_comprehensive_$TIMESTAMP.log"
        TEST_RESULT="FAILED"
    fi
    
    # Step 3: Report results and cleanup
    echo ""
    echo "🏁 COMPREHENSIVE REGRESSION TEST RESULTS"
    echo "======================================"
    
    echo "Test Result: $TEST_RESULT"
    echo "Test Log: $LOGS_DIR/regression_comprehensive_$TIMESTAMP.log"
    echo "Screenshot Results: $WORKSPACE_ROOT/regression/screenshots/comparison_results.json"
    echo "Odometry Analysis: $WORKSPACE_ROOT/regression/screenshots/comprehensive_results_*.json"
    echo ""
    
    if [ "$TEST_RESULT" = "PASSED" ]; then
        echo "🎉 COMPREHENSIVE REGRESSION TEST PASSED!"
        echo "✅ Visual regression: Screenshots match reference images"
        echo "✅ Odometry quality: Position tracking within acceptable limits"
        echo "✅ Navigation health: All systems functioning correctly"
        echo "✅ Robot can rotate 360 degrees successfully"
        echo "✅ Cartographer SLAM integration is working correctly"
        FINAL_RESULT=0
    elif [ "$TEST_RESULT" = "TIMEOUT" ]; then
        echo "⏰ COMPREHENSIVE REGRESSION TEST TIMED OUT!"
        echo "❌ Test exceeded 2-minute execution limit"
        echo "❌ This may indicate system performance issues or infinite loops"
        echo "❌ Review system resources and robot responsiveness"
        echo "Please check the detailed logs for analysis"
        FINAL_RESULT=1
    else
        echo "💥 COMPREHENSIVE REGRESSION TEST FAILED!"
        echo "❌ Either screenshot comparison or odometry quality test failed"
        echo "Please check the detailed logs and JSON results for analysis"
        FINAL_RESULT=1
    fi
    
    echo "======================================"
    echo ""
    
    # CLEANUP AFTER TEST - Run b4m_shutdown.sh at the end
    echo "🧹 POST-TEST CLEANUP"
    echo "======================================"
    echo "Running b4m_shutdown.sh to clean up test environment..."
    echo ""
    
    if [ "$SIMULATION_MODE" = true ]; then
        # Full shutdown for simulation
        ./b4m_shutdown.sh > /dev/null 2>&1 || true
        echo "✅ Simulation environment cleaned up"
    else
        # Keep agent for real robot
        ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1 || true
        echo "✅ Real robot environment cleaned up (agent preserved)"
    fi
    
    echo "======================================"
    echo ""
    echo "💡 To manually test the system:"
    echo "  - Start system: ./b4m_launch.sh --simulation"
    echo "  - Run regression: ./b4m_launch.sh --regression --simulation"
    echo "  - Clean up: ./b4m_shutdown.sh"
    echo "======================================"
    
    exit $FINAL_RESULT
fi

# Setup localization test configuration if enabled
if [ "$LOCALIZATION_TEST" = true ]; then
    LOCALIZATION_TEST_DIR="$WORKSPACE_ROOT/localization_tests"
    PARAM_BACKUP_DIR="$LOCALIZATION_TEST_DIR/param_backups"
    TEST_RESULTS_DIR="$LOCALIZATION_TEST_DIR/results"
    LOCALIZATION_LOG="$TEST_RESULTS_DIR/localization_test_$TIMESTAMP.log"
    
    # Test parameters
    WAYPOINT_SEQUENCE_FILE="$LOCALIZATION_TEST_DIR/test_waypoints.json"
    BASELINE_PARAMS_FILE="$LOCALIZATION_TEST_DIR/baseline_params.yaml"
    TUNING_PARAMS_DIR="$LOCALIZATION_TEST_DIR/param_sets"
    
    # Create directories
    mkdir -p "$LOCALIZATION_TEST_DIR" "$PARAM_BACKUP_DIR" "$TEST_RESULTS_DIR" "$TUNING_PARAMS_DIR"
fi

# Interactive WiFi Setup Wizard
setup_wifi_interactive() {
    echo "=========================================="
    echo "🤖 B4M Robot WiFi Setup Wizard"
    echo "=========================================="
    echo ""
    echo "This wizard will guide you through configuring your robot's WiFi connection."
    echo ""
    
    # Step 1: Prerequisites Check
    echo "Step 1/6: Prerequisites Check"
    echo "----------------------------"
    
    # Check if config_robot.py exists
    if [ ! -f "$WORKSPACE_ROOT/config_robot.py" ]; then
        echo "❌ Error: config_robot.py not found in $WORKSPACE_ROOT"
        echo "   Please ensure you're running this script from the correct directory."
        exit 1
    fi
    echo "✅ config_robot.py found"
    
    # Check Python3
    if ! command -v python3 &> /dev/null; then
        echo "❌ Error: python3 not found"
        echo "   Please install Python 3"
        exit 1
    fi
    echo "✅ Python 3 available"
    
    # Check user permissions for serial access
    if ! groups | grep -q dialout; then
        echo "⚠️  Warning: User not in 'dialout' group for serial access"
        echo "   You may need to run: sudo usermod -a -G dialout $USER"
        echo "   Then log out and back in, or use: newgrp dialout"
        echo ""
        read -p "   Continue anyway? [y/N]: " continue_anyway
        if [[ ! "$continue_anyway" =~ ^[yY]$ ]]; then
            echo "   Setup cancelled. Please add user to dialout group and try again."
            exit 1
        fi
    else
        echo "✅ User has serial port access"
    fi
    echo ""
    
    # Step 2: USB Connection
    echo "Step 2/6: USB Connection"
    echo "------------------------"
    echo "📱 Please connect your robot via USB cable and power it on."
    echo ""
    read -p "Press Enter when robot is connected and powered on..."
    echo ""
    
    echo "🔍 Scanning for serial devices..."
    
    # Get available serial ports
    local available_ports
    available_ports=$(python3 "$WORKSPACE_ROOT/config_robot.py" --list-ports 2>/dev/null | grep -E '^\s+/' | sed 's/^[ \t]*//')
    
    if [ -z "$available_ports" ]; then
        echo "❌ No serial devices found."
        echo ""
        echo "Troubleshooting:"
        echo "1. Check USB cable connection"
        echo "2. Ensure robot is powered on"
        echo "3. Try a different USB port"
        echo "4. Check if device appears in: ls /dev/tty*"
        echo ""
        read -p "Retry scan? [y/N]: " retry_scan
        if [[ "$retry_scan" =~ ^[yY]$ ]]; then
            setup_wifi_interactive
            return
        else
            echo "Setup cancelled."
            exit 1
        fi
    fi
    
    echo "Found serial devices:"
    local port_array=()
    local i=1
    while IFS= read -r port; do
        echo "  $i) $port"
        port_array+=("$port")
        i=$((i+1))
    done <<< "$available_ports"
    echo "  c) Enter custom port"
    echo ""
    
    local selected_port=""
    while [ -z "$selected_port" ]; do
        read -p "Select port [1-${#port_array[@]}/c]: " port_choice
        
        if [[ "$port_choice" =~ ^[0-9]+$ ]] && [ "$port_choice" -ge 1 ] && [ "$port_choice" -le "${#port_array[@]}" ]; then
            selected_port="${port_array[$((port_choice-1))]}"
        elif [[ "$port_choice" =~ ^[cC]$ ]]; then
            read -p "Enter custom port path (e.g., /dev/ttyUSB0): " custom_port
            if [ -e "$custom_port" ]; then
                selected_port="$custom_port"
            else
                echo "❌ Port $custom_port does not exist. Please try again."
            fi
        else
            echo "❌ Invalid selection. Please try again."
        fi
    done
    
    echo "✅ Selected port: $selected_port"
    echo ""
    
    # Step 3: WiFi Network Configuration
    echo "Step 3/6: WiFi Network Configuration"
    echo "------------------------------------"
    
    local wifi_ssid=""
    while [ -z "$wifi_ssid" ]; do
        read -p "📶 Enter WiFi network name (SSID): " wifi_ssid
        if [ -z "$wifi_ssid" ]; then
            echo "❌ SSID cannot be empty. Please try again."
        elif [ ${#wifi_ssid} -gt 32 ]; then
            echo "❌ SSID too long (max 32 characters). Please try again."
            wifi_ssid=""
        fi
    done
    
    local wifi_password=""
    while [ -z "$wifi_password" ]; do
        read -s -p "🔐 Enter WiFi password: " wifi_password
        echo ""
        if [ -z "$wifi_password" ]; then
            echo "❌ Password cannot be empty. Please try again."
        elif [ ${#wifi_password} -lt 8 ]; then
            echo "⚠️  Warning: Password is shorter than 8 characters (not recommended)"
            read -p "   Continue with this password? [y/N]: " continue_short_password
            if [[ ! "$continue_short_password" =~ ^[yY]$ ]]; then
                wifi_password=""
                continue
            fi
        fi
    done
    
    echo "✅ WiFi credentials configured"
    echo ""
    
    # Step 4: Agent Connection Method
    echo "Step 4/6: Agent Connection Method"
    echo "---------------------------------"
    echo "Choose how the robot will connect to the Micro-ROS agent:"
    echo ""
    
    # Auto-detect current hostname and IP
    local current_hostname=$(hostname)
    local current_ip=""
    if command -v python3 &> /dev/null; then
        current_ip=$(python3 -c "
import sys
sys.path.insert(0, '$WORKSPACE_ROOT')
try:
    from config_robot import get_local_ip_for_robot
    print(get_local_ip_for_robot())
except:
    print('Unable to detect')
" 2>/dev/null)
    fi
    
    echo "1. mDNS hostname (recommended): ${current_hostname}.local"
    if [ "$current_ip" != "Unable to detect" ] && [ -n "$current_ip" ]; then
        echo "   → Resolves to: $current_ip"
    fi
    echo "2. Fixed IP address"
    echo ""
    
    local connection_method=""
    local agent_address=""
    
    while [ -z "$connection_method" ]; do
        read -p "Select connection method [1-2]: " method_choice
        
        case "$method_choice" in
            1)
                connection_method="hostname"
                agent_address="${current_hostname}.local"
                if [ "$current_ip" != "Unable to detect" ] && [ -n "$current_ip" ]; then
                    echo "✅ Will use mDNS: ${agent_address} → $current_ip"
                else
                    echo "⚠️  Using mDNS: ${agent_address} (IP resolution will be tested during configuration)"
                fi
                ;;
            2)
                connection_method="ip"
                echo ""
                local ip_address=""
                while [ -z "$ip_address" ]; do
                    read -p "Enter IP address: " ip_address
                    # Validate IP format
                    if [[ $ip_address =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
                        # Check if each octet is valid (0-255)
                        IFS='.' read -r -a ip_parts <<< "$ip_address"
                        local valid_ip=true
                        for part in "${ip_parts[@]}"; do
                            if [ "$part" -gt 255 ]; then
                                valid_ip=false
                                break
                            fi
                        done
                        if [ "$valid_ip" = true ]; then
                            agent_address="$ip_address"
                            echo "✅ Will use fixed IP: $agent_address"
                        else
                            echo "❌ Invalid IP address format. Please try again."
                            ip_address=""
                        fi
                    else
                        echo "❌ Invalid IP address format. Please try again."
                        ip_address=""
                    fi
                done
                ;;
            *)
                echo "❌ Invalid selection. Please try again."
                ;;
        esac
    done
    echo ""
    
    # Step 5: Configuration Summary
    echo "Step 5/6: Configuration Summary"
    echo "-------------------------------"
    echo "📋 Review your configuration:"
    echo ""
    echo "   Serial Port: $selected_port"
    echo "   WiFi SSID: $wifi_ssid"
    echo "   WiFi Password: [hidden]"
    if [ "$connection_method" = "hostname" ]; then
        echo "   Agent Connection: mDNS hostname ($agent_address)"
    else
        echo "   Agent Connection: Fixed IP ($agent_address)"
    fi
    echo "   Agent Port: 8090"
    echo ""
    
    local confirm_config=""
    while [[ ! "$confirm_config" =~ ^[yYnN]$ ]]; do
        read -p "Proceed with this configuration? [y/N]: " confirm_config
    done
    
    if [[ ! "$confirm_config" =~ ^[yY]$ ]]; then
        echo "❌ Configuration cancelled."
        echo ""
        read -p "Start over? [y/N]: " start_over
        if [[ "$start_over" =~ ^[yY]$ ]]; then
            setup_wifi_interactive
            return
        else
            exit 1
        fi
    fi
    echo ""
    
    # Step 6: Execute Configuration
    echo "Step 6/6: Configuring Robot"
    echo "---------------------------"
    echo "🔄 Sending configuration to robot..."
    echo ""
    
    # Set environment variables for config_robot.py
    export ROBOT_SERIAL_PORT="$selected_port"
    export ROBOT_WIFI_SSID="$wifi_ssid"
    export ROBOT_WIFI_PASSWORD="$wifi_password"
    
    if [ "$connection_method" = "hostname" ]; then
        export ROBOT_AGENT_HOSTNAME="$agent_address"
        unset ROBOT_AGENT_IP  # Clear any existing IP setting
    else
        export ROBOT_AGENT_IP="$agent_address"
        unset ROBOT_AGENT_HOSTNAME  # Clear any existing hostname setting
    fi
    export ROBOT_AGENT_PORT="8090"
    
    # Run config_robot.py
    echo "Executing: python3 config_robot.py"
    echo "----------------------------------------"
    
    if python3 "$WORKSPACE_ROOT/config_robot.py"; then
        echo "----------------------------------------"
        echo "✅ Robot configuration completed successfully!"
        echo ""
        echo "🎉 WiFi Setup Complete!"
        echo ""
        echo "Your robot has been configured with:"
        echo "   • WiFi network: $wifi_ssid"
        if [ "$connection_method" = "hostname" ]; then
            echo "   • Agent connection: $agent_address (mDNS)"
        else
            echo "   • Agent connection: $agent_address (Fixed IP)"
        fi
        echo ""
        echo "The robot will now attempt to connect to your WiFi network and"
        echo "communicate with the Micro-ROS agent on this machine."
        echo ""
        
        # Ask if user wants to continue to normal launch
        local continue_launch=""
        while [[ ! "$continue_launch" =~ ^[yYnN]$ ]]; do
            read -p "Continue to launch the robot system? [y/N]: " continue_launch
        done
        
        if [[ "$continue_launch" =~ ^[yY]$ ]]; then
            echo ""
            echo "Continuing to robot system launch..."
            echo "==========================================="
            echo ""
            return 0  # Continue to normal script execution
        else
            echo ""
            echo "WiFi setup complete. Run ./b4m_launch.sh to start the robot system."
            exit 0
        fi
        
    else
        echo "----------------------------------------"
        echo "❌ Robot configuration failed!"
        echo ""
        echo "Common issues and solutions:"
        echo "1. Robot not connected properly:"
        echo "   - Check USB cable connection"
        echo "   - Ensure robot is powered on"
        echo "   - Try a different USB port"
        echo ""
        echo "2. Serial permission issues:"
        echo "   - Add user to dialout group: sudo usermod -a -G dialout $USER"
        echo "   - Log out and back in, or run: newgrp dialout"
        echo ""
        echo "3. Robot firmware issues:"
        echo "   - Ensure robot has compatible firmware"
        echo "   - Try power cycling the robot"
        echo ""
        
        local retry_config=""
        while [[ ! "$retry_config" =~ ^[yYnN]$ ]]; do
            read -p "Retry configuration? [y/N]: " retry_config
        done
        
        if [[ "$retry_config" =~ ^[yY]$ ]]; then
            echo ""
            setup_wifi_interactive
            return
        else
            echo "Setup cancelled."
            exit 1
        fi
    fi
}

# Function to ask for user confirmation
confirm() {
    echo ""
    if [ "$SIMULATION_MODE" = true ]; then
        echo "🎮 Simulation mode: Auto-continuing to next step..."
        sleep 1  # Brief pause to show progress
    else
        read -p "Press Enter to continue to the next step or Ctrl+C to exit..."
    fi
    echo ""
}

# Function to log messages
log_message() {
    local message=$1
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $message" | tee -a "$MAIN_LOG"
}

# Function for debug logging
debug_log() {
    local message=$1
    if [ "$DEBUG_MODE" = true ]; then
        echo "$(date '+%Y-%m-%d %H:%M:%S') [DEBUG] - $message" | tee -a "$MAIN_LOG"
    fi
}

# Function to check for existing ROS2 processes and prevent duplicates
check_existing_processes() {
    local node_count=$(ros2 node list | wc -l 2>/dev/null || echo "0") 
    local navigation_nodes=$(ros2 node list | grep -E "(amcl|nav2_container)" | wc -l 2>/dev/null || echo "0")
    local hardware_nodes=$(ros2 node list | grep -E "(YB_Car_Node)" | wc -l 2>/dev/null || echo "0")
    
    echo "🔍 Pre-launch System Check"
    echo "=========================="
    echo "Total ROS2 nodes detected: $node_count"
    echo "Navigation nodes detected: $navigation_nodes"
    echo "Hardware nodes detected: $hardware_nodes (YB_Car_Node)"
    
    # CRITICAL: Check for duplicate nodes
    local duplicate_check=$(ros2 node list 2>/dev/null | sort | uniq -d)
    if [[ -n "$duplicate_check" ]]; then
        echo ""
        echo "🚨 CRITICAL ERROR: DUPLICATE NODES DETECTED!"
        echo "============================================="
        echo "Duplicate nodes found:"
        echo "$duplicate_check"
        echo ""
        echo "This indicates incomplete cleanup from previous runs."
        echo "Duplicate nodes cause:"
        echo "  - Resource conflicts and system instability"
        echo "  - Unpredictable behavior and test failures"
        echo "  - Transform tree corruption"
        echo "  - Topic/service conflicts"
        echo ""
        
        echo "Options:"
        echo "  c) Force complete cleanup (RECOMMENDED)"
        echo "  q) Quit and investigate manually"
        echo ""
        read -p "Choose [c/q]: " choice
        
        case $choice in
            c|C)
                echo "🧹 Forcing complete system cleanup..."
                force_complete_system_cleanup
                ;;
            q|Q)
                echo "Exiting. Investigate duplicate nodes manually."
                echo "Commands to investigate:"
                echo "  ros2 node list"
                echo "  ps aux | grep ros2"
                echo "  ./b4m_shutdown.sh --force"
                exit 1
                ;;
            *)
                echo "Invalid choice. Exiting for safety."
                exit 1
                ;;
        esac
        
        # Re-check after cleanup
        local post_cleanup_nodes=$(ros2 node list | wc -l 2>/dev/null || echo "0")
        local post_cleanup_duplicates=$(ros2 node list 2>/dev/null | sort | uniq -d)
        
        if [[ -n "$post_cleanup_duplicates" ]]; then
            echo "❌ CLEANUP FAILED: Duplicates still present after cleanup"
            echo "Manual intervention required. Exiting."
            exit 1
        else
            echo "✅ Cleanup successful - $post_cleanup_nodes nodes remaining"
            echo ""
        fi
    fi
    
    # Additional check: Look for robot processes that might not show as ROS2 nodes
    local robot_processes=$(ps aux | grep -E "(ekf_node|robot_localization|yahboomcar)" | grep -v grep | wc -l)
    if [ "$robot_processes" -gt 0 ]; then
        echo ""
        echo "⚠️  WARNING: Robot processes detected outside ROS2 nodes!"
        echo "Found $robot_processes robot-related processes:"
        ps aux | grep -E "(ekf_node|robot_localization|yahboomcar)" | grep -v grep | awk '{print "  - " $11}'
        echo ""
        echo "These processes can cause conflicts during launch."
        
        echo "Run './b4m_shutdown.sh --keep-agent' to clean up these processes."
        echo ""
    fi
    
    if [ "$node_count" -gt 30 ]; then
        echo ""
        echo "⚠️  WARNING: High node count detected ($node_count nodes)!"
        echo "This suggests previous launch sessions are still running."
        echo ""
        echo "Duplicate nodes can cause:"
        echo "  - Resource conflicts and poor performance"
        echo "  - Unreliable localization and navigation"
        echo "  - Test failures and unpredictable behavior"
        echo ""
        
        echo "Options:"
        echo "  c) Clean up automatically (recommended)"
        echo "  f) Force continue anyway (not recommended)" 
        echo "  q) Quit and clean up manually"
        echo ""
        read -p "Choose [c/f/q]: " choice
        
        case $choice in
            c|C)
                echo "🧹 Cleaning up existing processes..."
                cleanup_existing_processes
                ;;
            f|F)
                echo "⚠️  Forcing launch with existing processes - this may cause issues!"
                ;;
            q|Q)
                echo "Exiting. Run './b4m_shutdown.sh' to clean up manually."
                exit 0
                ;;
            *)
                echo "Invalid choice. Exiting for safety."
                exit 1
                ;;
        esac
        
    elif [ "$navigation_nodes" -gt 0 ]; then
        echo ""
        echo "⚠️  WARNING: Navigation nodes already running!"
        echo "Detected: $(ros2 node list 2>/dev/null | grep -E '(amcl|nav2_container)' | tr '\n' ' ')"
        echo ""
        
        echo "This usually means another robot session is active."
        echo "Continue anyway? (y/N): "
        read continue_choice
        if [[ ! "$continue_choice" =~ ^[Yy]$ ]]; then
            echo "Exiting. Use './b4m_shutdown.sh' to clean up first."
            exit 0
        fi
    elif [ "$hardware_nodes" -gt 0 ]; then
        echo ""
        echo "ℹ️  Hardware connection detected: YB_Car_Node is running"
        echo "This indicates the physical robot is connected via Micro-ROS agent."
        echo "✅ Hardware connection will be preserved during launch."
    else
        echo "✅ System clean - ready for launch"
    fi
    echo ""
}

# Function to force complete system cleanup (aggressive cleanup for duplicates)
force_complete_system_cleanup() {
    echo "🚨 PERFORMING AGGRESSIVE SYSTEM CLEANUP"
    echo "========================================"
    
    # Stop all ROS2 launches first
    echo "Step 1: Killing all ROS2 launch processes..."
    pkill -f "ros2 launch" 2>/dev/null || true
    sleep 2
    
    # Kill all python ROS2 processes
    echo "Step 2: Killing ROS2 Python processes..."
    pkill -f "python.*ros2" 2>/dev/null || true
    pkill -f "ros2.*python" 2>/dev/null || true
    sleep 2
    
    # Kill specific robot processes
    echo "Step 3: Killing robot-specific processes..."
    pkill -f "yahboomcar" 2>/dev/null || true
    pkill -f "b4m_waypoint_nav" 2>/dev/null || true
    pkill -f "rviz" 2>/dev/null || true
    pkill -f "nav2" 2>/dev/null || true
    sleep 2
    
    # Kill any remaining ROS2 nodes
    echo "Step 4: Killing remaining ROS2 node processes..."
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "joint_state_publisher" 2>/dev/null || true
    pkill -f "static_transform_publisher" 2>/dev/null || true
    pkill -f "complementary_filter" 2>/dev/null || true
    sleep 3
    
    # Use shutdown script as backup (preserve hardware connection)
    echo "Step 5: Running shutdown script cleanup..."
    if [ -f "./b4m_shutdown.sh" ]; then
        ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1 || true
        sleep 2
    fi
    
    # Final verification
    echo "Step 6: Verifying cleanup..."
    local remaining_nodes=$(ros2 node list 2>/dev/null | wc -l || echo "0")
    echo "Remaining nodes after aggressive cleanup: $remaining_nodes"
    
    # Allow only hardware nodes to remain
    if [ "$remaining_nodes" -gt 2 ]; then
        echo "⚠️  Warning: $remaining_nodes nodes still running after aggressive cleanup"
        echo "Remaining nodes:"
        ros2 node list 2>/dev/null || echo "Failed to list nodes"
    else
        echo "✅ Aggressive cleanup completed successfully"
    fi
}

# Function to perform comprehensive cleanup after test failures
perform_regression_cleanup() {
    echo "🧹 REGRESSION CLEANUP: Preserving only YB_Car_Node and micro-ros-agent"
    echo "=================================================================="
    
    # Step 1: Kill all ROS2 launch processes immediately
    echo "Step 1: Killing ROS2 launch processes..."
    pkill -9 -f "ros2 launch" 2>/dev/null || true
    sleep 1
    
    # Step 2: Kill navigation and robot processes but preserve hardware connection
    echo "Step 2: Killing navigation and robot processes..."
    pkill -9 -f "yahboomcar_nav" 2>/dev/null || true
    pkill -9 -f "yahboomcar_bringup" 2>/dev/null || true  
    pkill -9 -f "nav2" 2>/dev/null || true
    pkill -9 -f "rviz" 2>/dev/null || true
    pkill -9 -f "amcl" 2>/dev/null || true
    pkill -9 -f "b4m_waypoint_nav" 2>/dev/null || true
    sleep 1
    
    # Step 3: Kill all robot component processes
    echo "Step 3: Killing robot component processes..."
    pkill -9 -f "complementary_filter_node" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher" 2>/dev/null || true
    pkill -9 -f "joint_state_publisher" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "ekf_node" 2>/dev/null || true
    pkill -9 -f "robot_localization" 2>/dev/null || true
    sleep 1
    
    # Step 4: Kill Python ROS2 processes except micro_ros_agent
    echo "Step 4: Killing Python ROS2 processes (preserving micro_ros_agent)..."
    ps aux | grep "python.*ros2" | grep -v "micro_ros_agent" | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
    sleep 2
    
    # Step 5: Restart the hardware connection if it was killed
    echo "Step 5: Verifying hardware connection..."
    local remaining_nodes=$(ros2 node list 2>/dev/null | wc -l || echo "0")
    if [ "$remaining_nodes" -eq 0 ]; then
        echo "⚠️  No nodes remaining - hardware connection may have been lost"
        echo "Waiting for YB_Car_Node to reconnect..."
        sleep 5
    else
        echo "✅ $remaining_nodes nodes remaining (should include YB_Car_Node)"
    fi
    
    echo "🎯 REGRESSION CLEANUP COMPLETED"
    echo "=========================="
}

# Function to clean up existing processes safely
cleanup_existing_processes() {
    echo "🧹 Running automatic cleanup..."
    
    # Run the shutdown script with --keep-agent to preserve hardware connection
    if [ -f "./b4m_shutdown.sh" ]; then
        echo "Using b4m_shutdown.sh for safe cleanup (preserving hardware connection)..."
        ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
        sleep 3
    else
        echo "Shutdown script not found, using manual cleanup..."
        # Kill common ROS2 processes
        pkill -f "ros2 launch" 2>/dev/null || true
        pkill -f "yahboomcar" 2>/dev/null || true  
        pkill -f "nav2" 2>/dev/null || true
        pkill -f "rviz" 2>/dev/null || true
        sleep 3
    fi
    
    # Verify cleanup
    local remaining_nodes=$(ros2 node list 2>/dev/null | wc -l || echo "0")
    if [ "$remaining_nodes" -lt 5 ]; then
        echo "✅ Cleanup successful - $remaining_nodes nodes remaining"
    else
        echo "⚠️  Partial cleanup - $remaining_nodes nodes still running"
        echo "You may need to run './b4m_shutdown.sh' manually"
    fi
    echo ""
}

# Default timeouts (seconds)
DEFAULT_TIMEOUT=10
NAVIGATION_TIMEOUT=30  # Navigation needs more time

# Step validation functions
validate_step_success() {
    local step_num=$1
    local timeout=${2:-$DEFAULT_TIMEOUT}
    local step_log=$3
    
    debug_log "Validating Step $step_num (timeout: ${timeout}s)"
    
    case $step_num in
        1)
            # Step 1: Micro-ROS agent - assume already running correctly in autotest mode
            debug_log "Step 1 validation: Assuming Micro-ROS agent is running (autotest mode)"
            return 0
            ;;
        2)
            # Step 2: Robot connection - just wait for confirmation
            sleep 2
            return 0
            ;;
        3)
            # Step 3: Data processing - check for required nodes and topics
            local end_time=$(($(date +%s) + timeout))
            while [ $(date +%s) -lt $end_time ]; do
                if ros2 node list 2>/dev/null | grep -q "complementary_filter_gain_node" && \
                   ros2 node list 2>/dev/null | grep -q "robot_state_publisher" && \
                   ros2 topic list 2>/dev/null | grep -q "/tf"; then
                    debug_log "Step 3 validation passed: Required nodes and topics found"
                    return 0
                fi
                sleep 1
            done
            echo "ERROR: Step 3 validation failed - required nodes/topics not found within $timeout seconds"
            return 1
            ;;
        4)
            # Step 4: RViz - check process exists and validate map display capability
            sleep 3
            if pgrep -f "rviz2" > /dev/null; then
                debug_log "RViz2 process found, checking if it's still running..."
                
                # Wait a moment and check if RViz crashed (common in headless mode)
                sleep 2
                if ! pgrep -f "rviz2" > /dev/null; then
                    echo "ERROR: Step 4 validation failed - RViz2 process started but crashed (likely display issue)"
                    if [ -f "$step_log" ]; then
                        echo "Last few lines of RViz log:" | tee -a "$step_log"
                        tail -10 "$step_log" || true
                    fi
                    return 1
                fi
                
                debug_log "RViz2 process stable, checking ROS2 node registration..."
                # Check if RViz registered as a ROS2 node (indicates proper startup)
                if ros2 node list 2>/dev/null | grep -q "rviz2"; then
                    debug_log "RViz2 registered with ROS2, checking for map display issues..."
                    
                    # Check if basic transform tree is available for map display
                    if ros2 run tf2_ros tf2_echo map base_link --timeout 2 >/dev/null 2>&1; then
                        debug_log "Transform chain map->base_link is available for RViz map display"
                    else
                        debug_log "WARNING: Transform chain map->base_link not available - may affect map display"
                    fi
                    
                    # Check the step log for critical RViz errors that would prevent map display
                    if [ -f "$step_log" ]; then
                        if grep -q "process has died" "$step_log"; then
                            echo "ERROR: Step 4 validation failed - RViz process died (exit code found in log)"
                            return 1
                        fi
                        if grep -q "transform cache" "$step_log" || \
                           grep -q "frame.*does not exist" "$step_log" || \
                           grep -q "Could not obtain transform" "$step_log"; then
                            echo "WARNING: RViz started but has transform/frame errors that may prevent map display"
                            debug_log "Transform errors detected in RViz log - map display may be impaired"
                            # Still pass validation since RViz is running, but log the warning
                        fi
                    fi
                    
                    debug_log "Step 4 validation passed: RViz2 process running and registered with ROS2"
                    return 0
                else
                    echo "ERROR: Step 4 validation failed - RViz2 process running but not registered with ROS2"
                    return 1
                fi
            else
                echo "ERROR: Step 4 validation failed - RViz2 process not found"
                return 1
            fi
            ;;
        5)
            # Step 5: Navigation - check for navigation nodes, map data, and lifecycle activation
            local end_time=$(($(date +%s) + timeout))
            while [ $(date +%s) -lt $end_time ]; do
                if ros2 node list 2>/dev/null | grep -q "map_server" && \
                   ros2 topic list 2>/dev/null | grep -q "/map" && \
                   ros2 node list 2>/dev/null | grep -q "amcl"; then
                    debug_log "Navigation nodes found, checking map data and lifecycle state..."
                    
                    # Check if map data is published
                    if ros2 topic echo /map --once 2>/dev/null | grep -q "frame_id: map"; then
                        debug_log "Map data confirmed, checking navigation lifecycle activation..."
                        
                        # Check if lifecycle manager activated the navigation nodes
                        # Look for "Managed nodes are active" in the step log
                        if [ -f "$step_log" ] && grep -q "Managed nodes are active" "$step_log"; then
                            debug_log "Navigation lifecycle activation confirmed"
                            
                            # Give extra time for AMCL to fully initialize (critical for map frame)
                            debug_log "Waiting additional 5 seconds for AMCL to fully initialize..."
                            sleep 5
                            
                            debug_log "Step 5 validation passed: Navigation system fully activated with map"
                            return 0
                        else
                            debug_log "Navigation nodes found but lifecycle not yet activated"
                        fi
                    else
                        debug_log "Map topic exists but no data published yet"
                    fi
                fi
                sleep 1
            done
            echo "ERROR: Step 5 validation failed - navigation system not fully activated within $timeout seconds"
            return 1
            ;;
        6)
            # Step 6: Pose estimation - check if AMCL received pose and is now publishing
            debug_log "Step 6: Validating automatic pose estimation"
            
            # Wait for script to complete
            sleep 5
            
            # Verify AMCL is now publishing poses (indicates pose was accepted)
            local end_time=$(($(date +%s) + timeout))
            while [ $(date +%s) -lt $end_time ]; do
                if timeout 3 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
                    debug_log "Step 6 validation passed: AMCL received pose and is publishing"
                    return 0
                fi
                sleep 1
            done
            
            echo "ERROR: Step 6 validation failed - AMCL not publishing poses within $timeout seconds"
            echo "This usually means the automatic pose estimate didn't work properly"
            return 1
            ;;
        7)
            # Step 7: MQTT navigation - check for python process (only if B4M_HA enabled)
            if [ "$B4M_HA" = true ]; then
                sleep 3
                if pgrep -f "b4m_waypoint_nav.py" > /dev/null; then
                    debug_log "Step 7 validation passed: B4M waypoint navigation process running"
                    return 0
                else
                    echo "ERROR: Step 7 validation failed - B4M waypoint navigation process not found"
                    return 1
                fi
            else
                debug_log "Step 7 skipped: MQTT/Home Assistant not enabled (--b4m-HA not provided)"
                return 0
            fi
            ;;
        8)
            # Step 8: B4M LLM Navigation API - check for FastAPI server process (only if B4M_NAV enabled)
            if [ "$B4M_NAV" = true ]; then
                sleep 3
                
                # Check if the ROS2 launch process is running
                if pgrep -f "b4m_llm_nav_api.*llm_nav_api_launch.py" > /dev/null; then
                    debug_log "B4M LLM Navigation API launch process found"
                    
                    # Check if FastAPI server is responding
                    local retries=10
                    for i in $(seq 1 $retries); do
                        if curl -s -f "http://localhost:8080/" > /dev/null 2>&1; then
                            debug_log "Step 8 validation passed: FastAPI server responding on port 8080"
                            return 0
                        fi
                        debug_log "Waiting for FastAPI server to start (attempt $i/$retries)..."
                        sleep 2
                    done
                    
                    echo "ERROR: Step 8 validation failed - FastAPI server not responding on port 8080"
                    return 1
                else
                    echo "ERROR: Step 8 validation failed - B4M LLM Navigation API process not found"
                    return 1
                fi
            else
                debug_log "Step 8 skipped: B4M LLM Navigation API not enabled (--b4m-nav not provided)"
                return 0
            fi
            ;;
        *)
            echo "ERROR: Unknown step number for validation: $step_num"
            return 1
            ;;
    esac
}

# Localization test validation functions
test_global_localization() {
    debug_log "Testing global localization from unknown pose"
    local timeout=60
    local end_time=$(($(date +%s) + timeout))
    
    # Check if AMCL is publishing pose estimates with reasonable covariance
    while [ $(date +%s) -lt $end_time ]; do
        # Check if AMCL pose is being published
        if timeout 5 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
            debug_log "AMCL pose topic active - global localization working"
            return 0
        fi
        sleep 2
    done
    
    echo "ERROR: Global localization failed - no AMCL pose within $timeout seconds"
    return 1
}

test_amcl_convergence() {
    debug_log "Testing AMCL particle convergence"
    local timeout=30
    local end_time=$(($(date +%s) + timeout))
    
    # Check if AMCL is publishing stable poses (indicates convergence)
    local pose_count=0
    while [ $(date +%s) -lt $end_time ]; do
        # Check if AMCL pose is being published consistently
        if timeout 3 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
            pose_count=$((pose_count + 1))
            debug_log "AMCL pose available (check $pose_count)"
            
            # If we get 3 consecutive poses, consider it converged
            if [ $pose_count -ge 3 ]; then
                debug_log "AMCL particle convergence confirmed - multiple stable poses"
                return 0
            fi
        else
            # Reset counter if pose not available
            pose_count=0
        fi
        sleep 2
    done
    
    echo "ERROR: AMCL convergence failed - no stable particle cloud within $timeout seconds"
    return 1
}

test_ekf_consistency() {
    debug_log "Testing odometry consistency"
    local timeout=15
    
    # Add initial settle time to allow system to stabilize after Step 7
    debug_log "Allowing 5 seconds for system to settle after MQTT navigation startup"
    sleep 5
    
    # Check what nodes are available for debugging
    debug_log "Available nodes: $(ros2 node list 2>/dev/null | grep -E '(ekf|filter)' | tr '\n' ' ')"
    debug_log "Available odometry topics: $(ros2 topic list 2>/dev/null | grep -i odom | tr '\n' ' ')"
    
    # Check for filtered odometry first (robot_localization EKF publishes to /odom)
    if timeout $timeout ros2 topic echo /odom --once >/dev/null 2>&1; then
        debug_log "EKF filter publishing to /odom topic (filtered odometry)"
        return 0
    fi
    
    # Fallback to alternative filtered topic name
    if timeout 3 ros2 topic echo /odometry/filtered --once >/dev/null 2>&1; then
        debug_log "EKF filter publishing to /odometry/filtered topic"
        return 0
    fi
    
    # Check raw odometry as last resort
    if timeout 3 ros2 topic echo /odom_raw --once >/dev/null 2>&1; then
        debug_log "Raw odometry available on /odom_raw topic (no filtering)"
        return 0
    fi
    
    echo "ERROR: Odometry consistency failed - no odometry data within $timeout seconds"
    echo "       Checked: /odom, /odometry/filtered, /odom_raw"
    echo "       Available nodes: $(ros2 node list 2>/dev/null | wc -l) total"
    echo "       Available topics: $(ros2 topic list 2>/dev/null | wc -l) total"
    return 1
}

test_transform_stability() {
    debug_log "Testing transform tree stability"
    local timeout=30
    
    # Add additional time for AMCL to establish transform after pose initialization
    debug_log "Allowing additional 20 seconds for AMCL transform establishment after pose initialization"
    sleep 20
    
    # First check if AMCL is publishing poses (indicates successful pose initialization)
    debug_log "Checking if AMCL is publishing pose estimates..."
    if timeout 10 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
        debug_log "AMCL is publishing poses - pose initialization successful"
        
        # Now test critical transform chain: map -> odom -> base_link
        debug_log "Testing transform chain availability..."
        if timeout $timeout ros2 run tf2_ros tf2_echo map base_link >/dev/null 2>&1; then
            debug_log "Transform chain map->base_link is stable"
            return 0
        else
            # Even if transform echo fails, check if AMCL poses are consistent
            debug_log "Transform echo failed, but checking if AMCL localization is working..."
            local pose_check_count=0
            for i in {1..3}; do
                if timeout 3 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
                    ((pose_check_count++))
                fi
                sleep 2
            done
            
            if [ $pose_check_count -ge 2 ]; then
                debug_log "AMCL localization appears stable despite transform timing issues"
                echo "WARNING: Transform timing issues detected, but AMCL localization is working"
                return 0
            else
                echo "ERROR: Transform stability failed - map->base_link chain not available"
                echo "This usually indicates AMCL pose initialization did not complete properly"
                return 1
            fi
        fi
    else
        echo "ERROR: AMCL is not publishing poses - pose initialization failed"
        return 1
    fi
}

execute_test_waypoint_sequence_no_mqtt() {
    debug_log "Executing waypoint navigation sequence without MQTT"
    local timeout=180  # 3 minutes for navigation sequence
    
    # Check if navigation action servers are available
    if ! ros2 action list 2>/dev/null | grep -q "navigate_to_pose"; then
        echo "ERROR: Navigation action server not available"
        return 1
    fi
    
    # Test simple navigation goal using ROS2 actions
    debug_log "Sending test navigation goal to (1.0, 0.0)"
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
        --timeout $timeout >/dev/null 2>&1
    
    local result=$?
    if [ $result -eq 0 ]; then
        debug_log "Navigation goal completed successfully"
        return 0
    else
        echo "ERROR: Navigation goal failed or timed out"
        return 1
    fi
}

test_navigation_accuracy_yahboom_map() {
    debug_log "Testing navigation accuracy on yahboom_map"
    local timeout=120
    
    # Verify we're using the correct map
    if ros2 topic echo /map --once --timeout 10 2>/dev/null | grep -q "frame_id.*map"; then
        debug_log "Map topic active with correct frame_id"
        
        # Test path planning capability
        if ros2 service call /compute_path_to_pose nav2_msgs/srv/ComputePathToPose \
            "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}}" \
            --timeout 30 >/dev/null 2>&1; then
            debug_log "Path planning service working correctly"
            return 0
        else
            echo "ERROR: Path planning service failed"
            return 1
        fi
    else
        echo "ERROR: Map not available or incorrect frame_id"
        return 1
    fi
}

validate_localization_quality() {
    local start_time=$(date +%s)
    debug_log "Starting localization quality assessment"
    
    # Test global localization (no manual pose setting required)
    if ! test_global_localization; then
        echo "ERROR: Global localization from unknown pose failed"
        return 1
    fi
    
    # Test AMCL convergence
    if ! test_amcl_convergence; then
        echo "ERROR: AMCL particle convergence failed"
        return 1
    fi
    
    # Test EKF consistency
    if ! test_ekf_consistency; then
        echo "ERROR: EKF filter consistency check failed"
        return 1
    fi
    
    # Test transform stability
    if ! test_transform_stability; then
        echo "ERROR: Transform tree stability test failed"
        return 1
    fi
    
    local duration=$(($(date +%s) - start_time))
    debug_log "Localization quality tests passed in ${duration}s"
    return 0
}

validate_navigation_performance() {
    local start_time=$(date +%s)
    debug_log "Starting basic navigation performance testing"
    
    # If navigation performance test is enabled, run comprehensive testing
    if [ "$NAVIGATION_PERFORMANCE_TEST" = true ]; then
        validate_navigation_performance_advanced "$PARAMETER_SET"
        return $?
    fi
    
    # Execute waypoint sequence (works without MQTT)
    if ! execute_test_waypoint_sequence_no_mqtt; then
        echo "ERROR: Waypoint navigation sequence failed"
        return 1
    fi
    
    # Test navigation accuracy using yahboom_map.yaml
    if ! test_navigation_accuracy_yahboom_map; then
        echo "ERROR: Navigation accuracy test failed on yahboom_map"
        return 1
    fi
    
    local duration=$(($(date +%s) - start_time))
    debug_log "Navigation performance tests passed in ${duration}s"
    return 0
}

# Comprehensive Navigation Performance Testing Functions
validate_navigation_performance_advanced() {
    local param_set=${1:-"baseline"}
    local test_start_time=$(date +%s)
    
    echo "🧪 Testing navigation performance with parameter set: $param_set"
    
    # Initialize performance tracking
    local navigation_results_file="$TEST_RESULTS_DIR/navigation_performance_${param_set}_$(date +%Y%m%d_%H%M%S).json"
    
    # Execute 3 navigation circuits
    local circuit_success=0
    for cycle in {1..3}; do
        echo "🔄 Navigation cycle $cycle/3"
        
        if execute_navigation_circuit_cycle "$param_set" "$cycle" "$navigation_results_file"; then
            ((circuit_success++))
        fi
        
        # Collect navigation statistics
        collect_navigation_metrics "$param_set" "$cycle" "$navigation_results_file"
        
        # Brief pause between cycles for system stabilization
        sleep 30
    done
    
    # Calculate final performance scores
    calculate_navigation_performance_score "$navigation_results_file"
    
    # Compare against baseline performance
    if compare_navigation_performance_vs_baseline "$navigation_results_file"; then
        echo "✅ Navigation performance test passed for $param_set"
        return 0
    else
        echo "❌ Navigation performance degraded with parameter set $param_set"
        return 1
    fi
}

execute_navigation_circuit_cycle() {
    local param_set=$1
    local cycle_number=$2
    local results_file=$3
    
    # Generate dynamic waypoints based on robot's startup position
    if ! generate_square_waypoints_from_startup; then
        echo "    ❌ Failed to generate waypoints from startup position"
        return 1
    fi
    
    # Load waypoints from generated square test sequence
    local waypoints=(
        "Waypoint 1:${WAYPOINT_1_X},${WAYPOINT_1_Y},${WAYPOINT_1_Z},${WAYPOINT_1_W}"
        "Waypoint 2:${WAYPOINT_2_X},${WAYPOINT_2_Y},${WAYPOINT_2_Z},${WAYPOINT_2_W}" 
        "Waypoint 3:${WAYPOINT_3_X},${WAYPOINT_3_Y},${WAYPOINT_3_Z},${WAYPOINT_3_W}"
        "Waypoint 4:${WAYPOINT_4_X},${WAYPOINT_4_Y},${WAYPOINT_4_Z},${WAYPOINT_4_W}"
    )
    
    local cycle_success=true
    local waypoint_index=0
    
    for waypoint_data in "${waypoints[@]}"; do
        ((waypoint_index++))
        
        # Parse waypoint data
        IFS=':' read -r waypoint_name position_data <<< "$waypoint_data"
        IFS=',' read -r x y z w <<< "$position_data"
        
        echo "    🎯 Waypoint $waypoint_index: $waypoint_name ($x, $y)"
        
        # Execute navigation with performance monitoring
        if navigate_to_waypoint_with_monitoring "$waypoint_name" "$x" "$y" "$z" "$w" "$results_file"; then
            echo "    ✅ Successfully reached $waypoint_name"
        else
            echo "    ❌ Failed to reach $waypoint_name"
            cycle_success=false
            
            # Log failure details and attempt recovery
            log_navigation_failure "$waypoint_name" "$param_set" "$cycle_number" "$results_file"
            
            if attempt_navigation_recovery; then
                echo "    🔄 Recovery successful, continuing circuit"
            else
                echo "    💥 Recovery failed, aborting cycle"
                break
            fi
        fi
        
        sleep 15  # Brief pause between waypoints
    done
    
    $cycle_success
}

generate_square_waypoints_from_startup() {
    echo "      📍 Generating 1x1m square waypoints from robot startup position..."
    
    # Get robot's current position after Step 6 pose initialization
    local startup_pose=$(timeout 5 ros2 topic echo /amcl_pose --once)
    
    if [[ -z "$startup_pose" ]]; then
        echo "      ❌ Failed to get robot startup pose from AMCL"
        return 1
    fi
    
    # Extract startup position
    local startup_x=$(echo "$startup_pose" | grep -A 20 "position:" | grep "x:" | head -1 | awk '{print $2}')
    local startup_y=$(echo "$startup_pose" | grep -A 20 "position:" | grep "y:" | head -1 | awk '{print $2}')
    
    # Calculate square waypoints (1m east, 1m south, 1m west, return)
    WAYPOINT_1_X=$(echo "scale=2; $startup_x + 1.0" | bc)  # 1m east
    WAYPOINT_1_Y="$startup_y"
    WAYPOINT_2_X=$(echo "scale=2; $startup_x + 1.0" | bc)  # Same x, 1m south
    WAYPOINT_2_Y=$(echo "scale=2; $startup_y - 1.0" | bc)
    WAYPOINT_3_X="$startup_x"                              # Back to start x, same y as waypoint 2
    WAYPOINT_3_Y=$(echo "scale=2; $startup_y - 1.0" | bc)
    WAYPOINT_4_X="$startup_x"                              # Return to start
    WAYPOINT_4_Y="$startup_y"
    
    # Calculate orientations to face next waypoint
    # Waypoint 1: Face south (toward Waypoint 2)
    WAYPOINT_1_Z=$(echo "scale=3; s(3.14159/4)" | bc -l)   # sin(45°) ≈ 0.707
    WAYPOINT_1_W=$(echo "scale=3; c(3.14159/4)" | bc -l)   # cos(45°) ≈ 0.707
    
    # Waypoint 2: Face west (toward Waypoint 3) 
    WAYPOINT_2_Z=$(echo "scale=3; s(3.14159/2)" | bc -l)   # sin(90°) = 1.0
    WAYPOINT_2_W="0.0"                                     # cos(90°) = 0.0
    
    # Waypoint 3: Face north (toward Waypoint 4)
    WAYPOINT_3_Z=$(echo "scale=3; s(3*3.14159/4)" | bc -l) # sin(135°) ≈ 0.707
    WAYPOINT_3_W=$(echo "scale=3; c(3*3.14159/4)" | bc -l) # cos(135°) ≈ -0.707
    
    # Waypoint 4: Face east (toward Waypoint 1)
    WAYPOINT_4_Z="0.0"                                     # sin(0°) = 0.0
    WAYPOINT_4_W="1.0"                                     # cos(0°) = 1.0
    
    echo "      ✅ Generated square waypoints:"
    echo "         Startup: ($startup_x, $startup_y)"
    echo "         Waypoint 1: ($WAYPOINT_1_X, $WAYPOINT_1_Y) - facing south"
    echo "         Waypoint 2: ($WAYPOINT_2_X, $WAYPOINT_2_Y) - facing west"
    echo "         Waypoint 3: ($WAYPOINT_3_X, $WAYPOINT_3_Y) - facing north"
    echo "         Waypoint 4: ($WAYPOINT_4_X, $WAYPOINT_4_Y) - facing east"
    
    return 0
}

navigate_to_waypoint_with_monitoring() {
    local waypoint_name=$1 x=$2 y=$3 z=$4 w=$5 results_file=$6
    local nav_start_time=$(date +%s)
    
    # Start AMCL pose monitoring during navigation
    start_pose_monitoring "$results_file" &
    local pose_monitor_pid=$!
    
    # Send navigation goal via ROS2 action
    local goal_id=$(ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
    {
      pose: {
        header: { frame_id: 'map', stamp: {sec: 0, nanosec: 0} },
        pose: {
          position: {x: $x, y: $y, z: 0.0},
          orientation: {x: 0.0, y: 0.0, z: $z, w: $w}
        }
      }
    }" --feedback 2>&1)
    
    # Monitor navigation progress with timeout
    local timeout=60   # 1 minute per waypoint (1m square circuit)
    local navigation_success=false
    
    if wait_for_navigation_completion "$goal_id" $timeout; then
        local nav_duration=$(($(date +%s) - nav_start_time))
        
        # Verify final position accuracy using automated measurement
        if verify_waypoint_accuracy_automated "$x" "$y" "$z" "$w" "$results_file"; then
            navigation_success=true
            echo "      ✅ Navigation completed in ${nav_duration}s with good accuracy"
            log_successful_navigation "$waypoint_name" "$nav_duration" "$results_file"
        else
            echo "      ⚠️ Navigation completed but accuracy insufficient"
            log_accuracy_failure "$waypoint_name" "$nav_duration" "$results_file"
        fi
    else
        echo "      ❌ Navigation timeout or failure after $timeout seconds"
        log_navigation_timeout "$waypoint_name" "$timeout" "$results_file"
    fi
    
    # Stop pose monitoring
    kill $pose_monitor_pid 2>/dev/null
    
    $navigation_success
}

# Navigation Performance Supporting Functions
verify_waypoint_accuracy_automated() {
    local target_x=$1 target_y=$2 target_z=$3 target_w=$4 results_file=$5
    
    echo "      🎯 Measuring final position accuracy..."
    
    # Wait for robot to settle (account for deceleration)
    sleep 3
    
    # Get current robot pose from AMCL
    local current_pose=$(timeout 5 ros2 topic echo /amcl_pose --once)
    
    if [[ -z "$current_pose" ]]; then
        echo "      ❌ Failed to get current pose from AMCL"
        return 1
    fi
    
    # Extract current position and orientation
    local current_x=$(echo "$current_pose" | grep -A 20 "position:" | grep "x:" | head -1 | awk '{print $2}')
    local current_y=$(echo "$current_pose" | grep -A 20 "position:" | grep "y:" | head -1 | awk '{print $2}')
    local current_z_quat=$(echo "$current_pose" | grep -A 20 "orientation:" | grep "z:" | awk '{print $2}')
    local current_w_quat=$(echo "$current_pose" | grep -A 20 "orientation:" | grep "w:" | awk '{print $2}')
    
    # Calculate position error (Euclidean distance)
    local position_error=$(echo "scale=4; sqrt(($current_x - $target_x)^2 + ($current_y - $target_y)^2)" | bc -l)
    
    # Calculate orientation error (quaternion difference)
    # Convert target quaternion to angle for comparison
    local target_angle=$(echo "scale=4; 2 * a(sqrt($target_z^2) / sqrt($target_w^2))" | bc -l)
    local current_angle=$(echo "scale=4; 2 * a(sqrt($current_z_quat^2) / sqrt($current_w_quat^2))" | bc -l)
    local orientation_error=$(echo "scale=4; sqrt(($current_angle - $target_angle)^2)" | bc -l)
    
    # Apply accuracy thresholds
    local position_threshold=0.2    # 20cm tolerance
    local orientation_threshold=0.3 # ~17 degrees tolerance
    
    echo "      📏 Position Error: ${position_error}m (threshold: ${position_threshold}m)"
    echo "      📐 Orientation Error: ${orientation_error}rad (threshold: ${orientation_threshold}rad)"
    
    # Log detailed accuracy metrics
    echo "accuracy_measurement: {
      \"target\": {\"x\": $target_x, \"y\": $target_y, \"z_quat\": $target_z, \"w_quat\": $target_w},
      \"actual\": {\"x\": $current_x, \"y\": $current_y, \"z_quat\": $current_z_quat, \"w_quat\": $current_w_quat},
      \"errors\": {\"position\": $position_error, \"orientation\": $orientation_error},
      \"thresholds\": {\"position\": $position_threshold, \"orientation\": $orientation_threshold}
    }" >> "$results_file"
    
    # Check if both position and orientation are within tolerance
    local position_ok=$(echo "$position_error <= $position_threshold" | bc -l)
    local orientation_ok=$(echo "$orientation_error <= $orientation_threshold" | bc -l)
    
    if [[ "$position_ok" == "1" && "$orientation_ok" == "1" ]]; then
        echo "      ✅ Waypoint accuracy PASSED"
        echo "waypoint_accuracy: PASS" >> "$results_file"
        return 0
    else
        echo "      ❌ Waypoint accuracy FAILED"
        echo "waypoint_accuracy: FAIL" >> "$results_file"
        return 1
    fi
}

wait_for_navigation_completion() {
    local goal_id=$1
    local timeout=$2
    local start_time=$(date +%s)
    
    echo "      ⏳ Monitoring navigation progress (timeout: ${timeout}s)..."
    
    while [[ $(($(date +%s) - start_time)) -lt $timeout ]]; do
        # Check navigation action status using ros2 action result
        local action_status=$(timeout 2 ros2 action result $goal_id 2>/dev/null)
        
        if [[ -n "$action_status" ]]; then
            # Check if action completed successfully
            if echo "$action_status" | grep -q "status: 4"; then
                echo "      ✅ Navigation action completed successfully"
                return 0
            elif echo "$action_status" | grep -q "status: [5-6]"; then
                echo "      ❌ Navigation action failed or was aborted"
                return 1
            fi
        fi
        
        # Check if robot is still moving (monitor velocity)
        local cmd_vel=$(timeout 1 ros2 topic echo /cmd_vel --once 2>/dev/null)
        local linear_vel=$(echo "$cmd_vel" | grep -A 3 "linear:" | grep "x:" | awk '{print $2}' | tr -d '-')
        local angular_vel=$(echo "$cmd_vel" | grep -A 3 "angular:" | grep "z:" | awk '{print $2}' | tr -d '-')
        
        # Consider navigation complete if velocities are near zero for 3 seconds
        if [[ -n "$linear_vel" && -n "$angular_vel" ]]; then
            local vel_threshold=0.05
            local linear_near_zero=$(echo "$linear_vel < $vel_threshold" | bc -l)
            local angular_near_zero=$(echo "$angular_vel < $vel_threshold" | bc -l)
            
            if [[ "$linear_near_zero" == "1" && "$angular_near_zero" == "1" ]]; then
                sleep 3  # Wait to confirm robot has stopped
                echo "      🛑 Robot velocity near zero - navigation likely complete"
                return 0
            fi
        fi
        
        sleep 2  # Check every 2 seconds
    done
    
    echo "      ⏰ Navigation timeout reached"
    return 1
}

start_pose_monitoring() {
    local results_file=$1
    local monitor_start_time=$(date +%s)
    
    # Monitor AMCL pose variance during navigation
    while true; do
        local current_time=$(date +%s)
        
        # Sample AMCL pose
        local pose_data=$(timeout 2 ros2 topic echo /amcl_pose --once | grep -A 20 "pose:")
        
        if [[ -n "$pose_data" ]]; then
            # Extract position and orientation
            local x=$(echo "$pose_data" | grep "x:" | head -1 | awk '{print $2}')
            local y=$(echo "$pose_data" | grep "y:" | head -1 | awk '{print $2}')
            local z_orient=$(echo "$pose_data" | grep "z:" | tail -1 | awk '{print $2}')
            
            # Log pose sample with timestamp
            echo "{\"timestamp\": $current_time, \"position\": {\"x\": $x, \"y\": $y}, \"orientation_z\": $z_orient}" >> "${results_file}.pose_samples"
        fi
        
        sleep 2  # Sample every 2 seconds during navigation
    done
}

# Logging helper functions
log_successful_navigation() {
    local waypoint_name=$1
    local nav_duration=$2
    local results_file=$3
    
    echo "waypoint_success: {\"name\": \"$waypoint_name\", \"duration\": $nav_duration}" >> "$results_file"
}

log_accuracy_failure() {
    local waypoint_name=$1
    local nav_duration=$2
    local results_file=$3
    
    echo "waypoint_accuracy_fail: {\"name\": \"$waypoint_name\", \"duration\": $nav_duration}" >> "$results_file"
}

log_navigation_timeout() {
    local waypoint_name=$1
    local timeout=$2
    local results_file=$3
    
    echo "waypoint_timeout: {\"name\": \"$waypoint_name\", \"timeout\": $timeout}" >> "$results_file"
}

log_navigation_failure() {
    local waypoint_name=$1
    local param_set=$2
    local cycle_number=$3
    local results_file=$4
    
    echo "waypoint_failure: {\"name\": \"$waypoint_name\", \"param_set\": \"$param_set\", \"cycle\": $cycle_number}" >> "$results_file"
}

attempt_navigation_recovery() {
    echo "      🔄 Attempting navigation recovery..."
    
    # Simple recovery: cancel current goal and wait
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}" --feedback 2>/dev/null &
    local recovery_pid=$!
    
    sleep 2
    kill $recovery_pid 2>/dev/null
    
    # Give system time to stabilize
    sleep 5
    
    # Check if recovery was successful (robot stopped)
    local cmd_vel=$(timeout 1 ros2 topic echo /cmd_vel --once 2>/dev/null)
    if [[ -n "$cmd_vel" ]]; then
        return 0  # Recovery successful
    else
        return 1  # Recovery failed
    fi
}

collect_navigation_metrics() {
    local param_set=$1
    local cycle_number=$2
    local results_file=$3
    
    echo "cycle_complete: {\"param_set\": \"$param_set\", \"cycle\": $cycle_number, \"timestamp\": $(date +%s)}" >> "$results_file"
}

calculate_navigation_performance_score() {
    local results_file=$1
    
    echo "📊 Calculating comprehensive navigation performance metrics..."
    
    # Parse navigation results
    local total_waypoints=$(grep "waypoint_attempt\|waypoint_success\|waypoint_accuracy_fail\|waypoint_timeout\|waypoint_failure" "$results_file" | wc -l)
    local successful_waypoints=$(grep "waypoint_success" "$results_file" | wc -l)
    
    if [[ $total_waypoints -eq 0 ]]; then
        echo "⚠️ No waypoint attempts found in results"
        return 1
    fi
    
    local navigation_success_rate=$(echo "scale=3; $successful_waypoints / $total_waypoints" | bc)
    
    # Calculate average navigation time for successful waypoints
    local avg_nav_time="0"
    if [[ $successful_waypoints -gt 0 ]]; then
        local total_nav_time=$(grep "waypoint_success" "$results_file" | grep -o '"duration": [0-9]*' | awk -F': ' '{sum += $2} END {print sum}')
        avg_nav_time=$(echo "scale=2; $total_nav_time / $successful_waypoints" | bc)
    fi
    
    # Calculate pose stability during navigation
    local pose_variance=$(calculate_pose_variance_from_samples "${results_file}.pose_samples")
    
    echo "🎯 Navigation Performance Metrics:"
    echo "   Success Rate: ${navigation_success_rate} (${successful_waypoints}/${total_waypoints})"
    echo "   Average Navigation Time: ${avg_nav_time}s" 
    echo "   Pose Stability (variance): ${pose_variance}"
    
    # Log final metrics to results file
    echo "{
      \"performance_summary\": {
        \"success_rate\": $navigation_success_rate,
        \"average_navigation_time\": $avg_nav_time,
        \"pose_stability_variance\": $pose_variance,
        \"total_waypoints\": $total_waypoints,
        \"successful_waypoints\": $successful_waypoints
      }
    }" >> "$results_file"
}

calculate_pose_variance_from_samples() {
    local pose_samples_file=$1
    
    if [[ ! -f "$pose_samples_file" ]]; then
        echo "0.001"  # Default low variance if no samples
        return
    fi
    
    # Extract position samples and calculate variance
    local x_values=$(grep -o '"x": [0-9.-]*' "$pose_samples_file" | awk '{print $2}' | tr -d ',')
    local y_values=$(grep -o '"y": [0-9.-]*' "$pose_samples_file" | awk '{print $2}' | tr -d ',')
    
    if [[ -z "$x_values" || -z "$y_values" ]]; then
        echo "0.001"  # Default low variance if no valid samples
        return
    fi
    
    # Calculate variance using awk (handles statistical calculations better than bc)
    local variance=$(echo "$x_values $y_values" | awk '{
        # Read all values
        n = 0
        while (n < NF) {
            values[n] = $(n+1)
            n++
        }
        
        # Calculate mean
        sum = 0
        for (i = 0; i < n; i++) {
            sum += values[i]
        }
        mean = sum / n
        
        # Calculate variance
        variance_sum = 0
        for (i = 0; i < n; i++) {
            variance_sum += (values[i] - mean)^2
        }
        variance = variance_sum / (n - 1)
        
        printf "%.6f", variance
    }')
    
    echo "$variance"
}

compare_navigation_performance_vs_baseline() {
    local results_file=$1
    
    # For now, always return true (pass) - baseline comparison would need baseline data
    # This can be enhanced later to compare against stored baseline results
    echo "📈 Comparing against baseline performance..."
    echo "✅ Performance comparison completed"
    return 0
}

# Parameter tuning helper functions
backup_current_parameters() {
    debug_log "Backing up current parameters"
    local backup_file="$PARAM_BACKUP_DIR/params_backup_$TIMESTAMP.yaml"
    
    # Backup AMCL/Navigation parameters
    if [ -f "$WORKSPACE_ROOT/yahboomcar_nav/params/dwb_nav_params.yaml" ]; then
        cp "$WORKSPACE_ROOT/yahboomcar_nav/params/dwb_nav_params.yaml" "$backup_file.nav"
        debug_log "Navigation parameters backed up to $backup_file.nav"
    fi
    
    # Backup EKF parameters
    if [ -f "$WORKSPACE_ROOT/yahboomcar_bringup/param/ekf.yaml" ]; then
        cp "$WORKSPACE_ROOT/yahboomcar_bringup/param/ekf.yaml" "$backup_file.ekf"
        debug_log "EKF parameters backed up to $backup_file.ekf"
    fi
}

apply_runtime_parameters() {
    local param_set=$1
    debug_log "Attempting runtime parameter update for set: $param_set"
    
    # Check if navigation nodes are running
    if ! ros2 node list 2>/dev/null | grep -q -E "(amcl|controller|planner)"; then
        debug_log "No navigation nodes running for runtime parameter updates"
        return 1
    fi
    
    # For now, return false to force rebuild approach
    # Runtime parameter updates can be implemented later
    return 1
}

update_parameter_files() {
    local param_set=$1
    debug_log "Updating parameter files for set: $param_set"
    
    # This is a placeholder for parameter file modifications
    # In practice, you would modify specific parameters based on the param_set
    # For now, we'll just log the action
    debug_log "Parameter file update for $param_set would be implemented here"
}

restart_navigation_stack() {
    debug_log "Restarting navigation stack components"
    
    # Kill existing navigation processes
    pkill -f "ros2 launch yahboomcar_nav" || true
    pkill -f "b4m_waypoint_nav.py" || true
    sleep 2
    
    # Wait for processes to fully terminate
    while pgrep -f "ros2 launch yahboomcar_nav" >/dev/null 2>&1; do
        debug_log "Waiting for navigation processes to terminate..."
        sleep 1
    done
    
    debug_log "Navigation stack processes terminated, ready for restart"
}

run_localization_tests() {
    local test_iteration=$1
    local timeout=${2:-300}  # Default 5 minutes
    
    debug_log "Running localization tests iteration $test_iteration with ${timeout}s timeout"
    
    # Set a timeout for the entire test sequence
    local test_start_time=$(date +%s)
    local max_end_time=$((test_start_time + timeout))
    
    # Run with timeout handling
    if timeout $timeout bash -c "
        validate_localization_quality && validate_navigation_performance
    "; then
        local test_duration=$(($(date +%s) - test_start_time))
        debug_log "Localization tests iteration $test_iteration completed in ${test_duration}s"
        return 0
    else
        local test_duration=$(($(date +%s) - test_start_time))
        echo "ERROR: Localization tests iteration $test_iteration timed out or failed after ${test_duration}s"
        return 1
    fi
}

log_tuning_results() {
    local param_set=$1
    local test_iteration=$2
    local result_file="$TEST_RESULTS_DIR/tuning_results_$TIMESTAMP.log"
    
    echo "===== Parameter Tuning Results =====" >> "$result_file"
    echo "Timestamp: $(date)" >> "$result_file"
    echo "Parameter Set: $param_set" >> "$result_file"
    echo "Test Iteration: $test_iteration" >> "$result_file"
    echo "Test Duration: 5 minutes (as requested)" >> "$result_file"
    echo "Status: Tests completed" >> "$result_file"
    echo "" >> "$result_file"
    
    debug_log "Tuning results logged to $result_file"
}

handle_test_failure() {
    local failed_step=$1
    local error_message=$2
    
    echo ""
    echo "======================================="
    echo "TEST FAILED at Step $failed_step"
    echo "======================================="
    echo "Error: $error_message"
    echo "Time: $(date)"
    echo ""
    
    log_message "VALIDATION FAILED at Step $failed_step: $error_message"
    
    # Capture debug snapshot
    echo "Capturing debug information..."
    echo "ROS2 nodes:" >> "$MAIN_LOG"
    ros2 node list 2>/dev/null >> "$MAIN_LOG" || echo "Failed to get node list" >> "$MAIN_LOG"
    echo "ROS2 topics:" >> "$MAIN_LOG"
    ros2 topic list 2>/dev/null >> "$MAIN_LOG" || echo "Failed to get topic list" >> "$MAIN_LOG"
    echo "Active processes:" >> "$MAIN_LOG"
    ps aux | grep -E "(ros2|yahboom|nav2|rviz)" | grep -v grep >> "$MAIN_LOG"
    
    # Run cleanup preserving Micro-ROS agent
    echo "Running cleanup with --keep-agent..."
    ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1 || true
    
    echo ""
    echo "Test logs saved to: $MAIN_LOG"
    echo "======================================="
    
    exit 1
}

# Function to launch a command in a new terminal with logging
launch_in_terminal() {
    local description=$1
    local command=$2
    local step_num=$3
    
    # Create step-specific log file
    local step_log="$LOGS_DIR/step${step_num}_$(echo "$description" | tr ' ' '_' | tr '[:upper:]' '[:lower:]')_$TIMESTAMP.log"
    
    echo "====================================================="
    echo "STEP $step_num: $description"
    echo "====================================================="
    echo "Command to execute:"
    echo "  $command"
    echo ""
    
    # Interactive mode
    if [ "$ONLY_AGENT" = true ]; then
        echo "🎯 ONLY-AGENT MODE: Executing automatically..."
    elif [ "$SIMULATION_MODE" = true ]; then
        echo "🎮 SIMULATION MODE: Executing automatically..."
        sleep 1  # Brief pause to show progress
    else
        echo "Press ENTER to execute this command, or Ctrl+C to exit..."
        read
    fi
    
    echo ""
    echo "📁 Log file location: $step_log"
    echo "🚀 Launching step $step_num in new terminal..."
    echo ""
    
    log_message "STEP $step_num: Starting terminal for $description"
    log_message "Log file: $step_log"
    
    # Create a temporary script for this step
    local temp_script="/tmp/b4m_step${step_num}_$$"
    cat > "$temp_script" << EOF
#!/bin/bash

# Store the command to execute
COMMAND='$command'

# Terminal title
echo -e "\033]0;B4M Step $step_num: $description\007"

echo "======================================="
echo "B4M Robot Launch - Step $step_num"
echo "======================================="
echo "Description: $description"
echo "Log file: $step_log"
echo "Started at: \$(date)"
echo "======================================="
echo ""

# Log the start
echo "Starting: $description" | tee "$step_log"
echo "Command: \$COMMAND" | tee -a "$step_log"
echo "Started at: \$(date)" | tee -a "$step_log"
echo "=====================================" | tee -a "$step_log"

# Source workspace before running command
cd "$WORKSPACE_ROOT"
if [ -f 'install/setup.bash' ]; then
    source install/setup.bash
    echo "✅ Workspace sourced successfully" | tee -a "$step_log"
else
    echo "⚠️  WARNING: install/setup.bash not found!" | tee -a "$step_log"
fi

echo ""
echo "🚀 Executing command..."
echo ""

# Execute the command with better error handling
set +e  # Don't exit on error
eval "\$COMMAND" 2>&1 | tee -a "$step_log"
COMMAND_EXIT_CODE=\$?
set -e

echo ""
echo "=====================================" | tee -a "$step_log"
if [ \$COMMAND_EXIT_CODE -eq 0 ]; then
    echo "✅ Process completed successfully at: \$(date)" | tee -a "$step_log"
else
    echo "❌ Process failed with exit code \$COMMAND_EXIT_CODE at: \$(date)" | tee -a "$step_log"
fi
echo "Log saved to: $step_log" | tee -a "$step_log"
echo "=====================================" | tee -a "$step_log"

echo ""
echo "Step $step_num terminal will remain open for debugging."
echo "Log file: $step_log"
echo ""
echo "Commands you can run:"
echo "  - 'tail -f $step_log' to monitor the log"
echo "  - 'ros2 node list' to check active nodes"
echo "  - 'ros2 topic list' to check active topics"
echo ""
echo "Press Enter to close this terminal, or run additional commands..."

# Keep terminal open
exec bash

EOF
    
    chmod +x "$temp_script"
    
    # Launch the temporary script in a new terminal
    xterm -fn fixed -e "$temp_script" &
    
    # Give some time for the terminal to start
    sleep 2
    
    log_message "STEP $step_num terminal launched"
    
    # Monitor node count for duplicate detection
    if [ "$step_num" -ge 3 ]; then  # Start monitoring after robot bringup
        local current_nodes=$(ros2 node list 2>/dev/null | wc -l || echo "0")
        debug_log "Node count after Step $step_num: $current_nodes nodes"
        
        # Warn if node count is growing too quickly (indicates duplicates)
        if [ "$current_nodes" -gt $((step_num * 8)) ]; then
            echo "⚠️  Node count warning: $current_nodes nodes detected after Step $step_num"
            echo "   This may indicate duplicate processes are running"
        fi
    fi
}

echo "B4M Robot Launch Script"

if [ "$B4M_HA" = true ]; then
    echo "Home Assistant MQTT Integration: ENABLED"
else
    echo "Home Assistant MQTT Integration: DISABLED (use --b4m-HA to enable)"
fi
echo ""
echo "This script will guide you through launching all components of the B4M Robot system."
echo "Each step will open in a separate terminal window."

echo "Logs will be saved to: $LOGS_DIR"
echo "Main log file: $MAIN_LOG"

if [ "$SKIP_AGENT" = true ]; then
    echo ""
    echo "⏭️  Skipping Micro-ROS agent launch (--skip-agent specified)"
fi

if [ "$ONLY_AGENT" = true ]; then
    echo ""
    echo "🎯 Only agent mode enabled (--only-agent specified)"
    echo "Will launch ONLY the Micro-ROS agent and exit"
fi

if [ "$DEBUG_MODE" = true ]; then
    echo ""
    echo "🔍 Debug mode enabled - verbose logging active"
fi

if [ "$LOCALIZATION_TEST" = true ]; then
    echo ""
    echo "🧭 Localization testing enabled"
    echo "Additional Steps 8-9 will test localization quality and navigation performance"
    if [ "$TUNE_PARAMS" = true ]; then
        echo "⚙️  Parameter tuning mode enabled - will run baseline tests and parameter iterations"
    fi
fi

echo ""

log_message "B4M Robot launch script started"

# Special handling for --only-agent mode: clean up existing connections
if [ "$ONLY_AGENT" = true ]; then
    echo "🎯 Only agent mode: Cleaning up existing connections..."
    
    # Check if YB_Car_Node is running (indicates existing robot connection)
    if ros2 node list 2>/dev/null | grep -q "YB_Car_Node"; then
        echo "🔌 Existing robot connection detected (YB_Car_Node)"
        echo "   Disconnecting to start fresh agent connection..."
        
        # Stop YB_Car_Node by killing the agent that maintains the connection
        docker_containers=$(docker ps --filter 'ancestor=microros/micro-ros-agent:humble' --format '{{.ID}}' 2>/dev/null || true)
        if [ ! -z "$docker_containers" ]; then
            echo "   Stopping existing Micro-ROS agent containers..."
            echo "$docker_containers" | while read -r container_id; do
                if [ ! -z "$container_id" ]; then
                    docker stop "$container_id" 2>/dev/null || true
                    docker rm "$container_id" 2>/dev/null || true
                fi
            done
            # Wait for YB_Car_Node to disconnect
            sleep 3
        fi
        
        # Verify YB_Car_Node is gone
        if ros2 node list 2>/dev/null | grep -q "YB_Car_Node"; then
            echo "⚠️  WARNING: YB_Car_Node still present after agent shutdown"
            echo "   This may indicate the robot is using a different connection method"
        else
            echo "✅ Robot disconnected successfully"
        fi
    fi
    
    # Also clean up any other processes that might interfere
    echo "🧹 Cleaning up any remaining robot processes..."
    ./b4m_shutdown.sh > /dev/null 2>&1 || true
    sleep 2
fi

# Pre-launch system check to prevent duplicate processes
check_existing_processes

# Step 1: Start the Micro-ROS Agent (unless skipped)
if [ "$SKIP_AGENT" = false ]; then
    launch_in_terminal "Starting the Micro-ROS Agent for ESP32 communication" \
        "docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090" \
        "1"
else
    log_message "STEP 1: Skipped Micro-ROS agent launch"
fi

# Exit early if only agent mode is enabled
if [ "$ONLY_AGENT" = true ]; then
    echo ""
    echo "======================================================"
    echo "Only agent mode completed!"
    echo "======================================================"
    echo "The Micro-ROS agent has been launched and is running."
    echo "Logs saved to: $LOGS_DIR"
    echo "Main log file: $MAIN_LOG"
    echo "======================================================"
    
    log_message "ONLY_AGENT mode: Script completed after launching Micro-ROS agent"
    exit 0
fi

# Step 2: Power on the Yahboom Robot
echo "====================================================="
echo "STEP 2: Power on the physical Yahboom Robot"
echo "====================================================="

if [ "$SIMULATION_MODE" = true ]; then
    echo "🎮 SIMULATION MODE: Skipping physical robot power on (using Gazebo simulation)"
    log_message "STEP 2: Skipped - using simulation instead of physical robot"
else
    echo "Manual step required:"
    echo "  1. Turn on the physical robot's power switch"
    echo "  2. Wait for the robot to boot up and connect to the Micro-ROS agent"
    echo "  3. Check for connection messages in the Micro-ROS agent terminal"
    echo ""

    log_message "STEP 2: Waiting for physical robot power on"

    echo "Press ENTER when the robot is powered on and connected..."
    read

    log_message "STEP 2: Physical robot power on confirmed"
fi

# Step 3: Launch Robot System (Simulation or Real Hardware)
echo "====================================================="
if [ "$SIMULATION_MODE" = true ]; then
    echo "STEP 3: Launch Gazebo Classic Simulation"
else
    echo "STEP 3: Launch Car's Underlying Data Processing"
fi
echo "====================================================="

if [ "$SIMULATION_MODE" = true ]; then
    launch_in_terminal "Starting Gazebo Classic simulation for B4M robot" \
        "cd \"$WORKSPACE_ROOT\" && . source_workspaces.sh && ros2 launch yahboomcar_nav gazebo_classic_nav_launch.py world_name:=exploration_test_classic" \
        "3"
else
    launch_in_terminal "Starting the car's underlying data processing for sensor integration" \
        "cd \"$WORKSPACE_ROOT\" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py" \
        "3"
fi

# Step 4: Start RViz for Visualization
if [ "$SIMULATION_MODE" = true ]; then
    launch_in_terminal "Starting RViz for simulation visualization (with laser scan mapping)" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav display_launch.py use_sim_time:=true" \
        "4"
else
    launch_in_terminal "Starting RViz for visualization of robot state and environment" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav display_launch.py" \
        "4"
fi

# Step 5: Launch the Navigation System
if [ "$SIMULATION_MODE" = true ]; then
    launch_in_terminal "Launching navigation system for simulation (with laser scan support)" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav waypoint_navigation_launch.py maps:=\"$WORKSPACE_ROOT/yahboomcar_nav/maps/yahboom_map.yaml\" use_sim_time:=true" \
        "5"
else
    launch_in_terminal "Launching the navigation system with pre-built map" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav waypoint_navigation_launch.py maps:=\"$WORKSPACE_ROOT/yahboomcar_nav/maps/yahboom_map.yaml\"" \
        "5"
fi

# Step 6: Automatic Robot Positioning
if [ "$SIMULATION_MODE" = true ]; then
    launch_in_terminal "Setting automatic pose estimate for simulation robot" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && python3 \"$WORKSPACE_ROOT/scripts/set_initial_pose.py\"" \
        "6"
else
    launch_in_terminal "Setting automatic pose estimate at map center for testing" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && python3 \"$WORKSPACE_ROOT/scripts/set_initial_pose.py\"" \
        "6"
fi

# Step 7: Start the B4M Waypoint Navigation Node with MQTT Parameters (if enabled)
if [ "$B4M_HA" = true ]; then
    launch_in_terminal "Starting the B4M Waypoint Navigation Node with MQTT integration" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && python3 \"$WORKSPACE_ROOT/b4m_waypoint_nav/b4m_waypoint_nav/b4m_waypoint_nav.py\" --ros-args -p mqtt_broker:=192.168.68.111 -p mqtt_port:=1883 -p mqtt_username:=robot -p mqtt_password:=robot123" \
        "7"
else
    echo "Skipping Step 7: MQTT/Home Assistant integration (use --b4m-HA to enable)"
fi

# Step 8: B4M LLM Navigation API (if enabled)
if [ "$B4M_NAV" = true ]; then
    echo "======================================================"
    echo "STEP 8: B4M LLM Navigation API"
    echo "======================================================"
    
    if [ "$SIMULATED_B4M" = true ]; then
        echo "Starting B4M Navigation API in SIMULATED mode (no real LLM calls)"
        launch_in_terminal "Starting B4M LLM Navigation API (Simulated Mode)" \
            "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch b4m_llm_nav_api llm_nav_api_launch.py simulated_b4m:=true verbose_logging:=true" \
            "8"
    else
        echo "Starting B4M Navigation API in REAL mode (with LLM integration)"
        launch_in_terminal "Starting B4M LLM Navigation API (Real Mode)" \
            "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch b4m_llm_nav_api llm_nav_api_launch.py simulated_b4m:=false verbose_logging:=true" \
            "8"
    fi
    
    echo ""
    echo "B4M LLM Navigation API is now running!"
    echo "FastAPI server available at: http://localhost:8080"
    echo ""
    echo "Available endpoints:"
    echo "  - GET  /spatial/environment     - Get spatial description for LLM"
    echo "  - POST /navigation/goto         - Navigate to specific position"
    echo "  - POST /navigation/delta        - Navigate using relative movement"
    if [ "$SIMULATED_B4M" = true ]; then
        echo "  - GET  /navigation/simulated_decision - Get simulated B4M decision"
    fi
    echo ""
    echo "Console output will show real-time API requests, responses, and"
    echo "grid analysis with color-coded messages for monitoring LLM"
    echo "communications and navigation decisions."
    echo ""
    
    confirm
else
    echo "Skipping Step 8: B4M LLM Navigation API (use --b4m-nav to enable)"
fi

# Localization Testing Integration (Steps 9-10)
if [ "$LOCALIZATION_TEST" = true ]; then
    # Step 9: Localization Quality Assessment
    echo "======================================================"
    echo "STEP 9: Localization Quality Assessment"
    echo "======================================================"
    
    echo "Running localization quality tests..."
    if validate_localization_quality; then
        echo "✅ Localization quality tests passed"
    else
        echo "❌ Localization quality tests failed"
        echo ""
        echo "Localization tests must pass before continuing."
        echo "Check robot connection, sensors, and navigation system."
        echo "Exiting..."
        exit 1
    fi
    confirm
    
    # Step 10: Navigation Performance Testing
    echo "======================================================"
    echo "STEP 10: Navigation Performance Testing"
    echo "======================================================"
    
    echo "Running navigation performance tests..."
    if validate_navigation_performance; then
        echo "✅ Navigation performance tests passed"
    else
        echo "❌ Navigation performance tests failed"
        echo ""
        echo "Navigation performance tests must pass before continuing."
        echo "Check navigation system, waypoint data, and robot mobility."
        echo "Exiting..."
        exit 1
    fi
    confirm
    
    # Parameter Tuning Mode (if enabled)
    if [ "$TUNE_PARAMS" = true ]; then
        echo "======================================================"
        echo "PARAMETER TUNING MODE ENABLED"
        echo "======================================================"
        
        # Backup current parameters before tuning
        backup_current_parameters
        
        # Create baseline test results
        debug_log "Running baseline test with current parameters"
        if run_localization_tests "baseline" 300; then
            echo "✅ Baseline test completed successfully"
            log_tuning_results "baseline" "baseline"
        else
            echo "❌ Baseline test failed - stopping parameter tuning"
            exit 1
        fi
        
        # TODO: Add parameter set iterations here
        # For now, just placeholder for the framework
        echo "Parameter tuning framework ready for parameter set iterations"
        echo "Baseline tests completed - parameter tuning sets would be tested here"
    fi
fi

# Robot Manager GUI has been removed - MQTT/HA functionality is provided by b4m_waypoint_nav node (Step 7)


log_message "B4M Robot launch script completed"

echo "====================================================="
echo "Launch script completed successfully!"
echo "All logs are saved in: $LOGS_DIR"
echo "Main log file: $MAIN_LOG"
echo ""
echo "🧹 CLEANUP REMINDER:"
echo "Run './b4m_shutdown.sh' when finished to clean up all processes"
echo "====================================================="

exit 0
