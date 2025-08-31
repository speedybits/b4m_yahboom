#!/bin/bash

# B4M Robot Launch Script
# This script automates the launch process for the B4M Robot 
# Each step will be launched in a separate terminal with user confirmation
#
# Usage: ./b4m_launch.sh [--skip-agent] [--only-agent] [--debug] [--simulation] [--regression] [--explore] [--nav] [--b4m-api] [--ollama] [--ollama-advanced] [--ollama-nav] [--ollama-nav-basic] [--b4m-HA] [--b4m-ping] [--navigation-performance-test] [--setup-wifi] [--skip-rebuild]
#   --skip-agent:                   Skip the Micro-ROS agent launch (Step 1)
#   --only-agent:                   Launch ONLY the Micro-ROS agent (Step 1) and exit
#   --debug:                        Enable verbose debug logging
#   --simulation:                   Launch in Gazebo simulation mode instead of real robot
#   --regression:                   Run comprehensive regression test suite (navigation + laser stability)
#   --explore:                      Enable autonomous exploration mode with obstacle avoidance
#   --nav:                          Enable Navigation 2 with SLAM for goal-based navigation in RViz
#   --b4m-api:                      Enable B4M API mode (duplicate of --explore for API integration)
#   --ollama:                       Enable Ollama mode (basic cardinal directions)
#   --ollama-advanced:              Enable Ollama Advanced mode (360° spatial context)
#   --ollama-nav:                   Enable Ollama Navigation mode (LLM-guided Nav2 goal selection)
#   --ollama-nav-basic:             Enable basic Ollama Navigation mode (copy of --nav for testing)
#   --b4m-HA:                       (Experimental) Enable Home Assistant MQTT integration features
#   --b4m-ping:                     Test bike4mind API with random obstacle detection messages
#   --navigation-performance-test:  (Untested) Execute 1x1m square navigation circuit testing
#   --setup-wifi:                   Interactive WiFi setup wizard for robot configuration

# Parse command line arguments
SKIP_AGENT=false
ONLY_AGENT=false
DEBUG_MODE=false
SIMULATION_MODE=false
REGRESSION_MODE=false
EXPLORE_MODE=false
NAV_MODE=false
B4M_API=false
OLLAMA_MODE=false
OLLAMA_ADVANCED_MODE=false
OLLAMA_NAV_MODE=false
B4M_PING=false
B4M_HA=false
NAVIGATION_PERFORMANCE_TEST=false
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
        --nav)
            NAV_MODE=true
            shift
            ;;
        --b4m-api)
            B4M_API=true
            shift
            ;;
        --ollama)
            OLLAMA_MODE=true
            shift
            ;;
        --ollama-advanced)
            OLLAMA_ADVANCED_MODE=true
            shift
            ;;
        --ollama-nav)
            OLLAMA_NAV_MODE=true
            shift
            ;;
        --ollama-nav-basic)
            OLLAMA_NAV_BASIC_MODE=true
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
        --navigation-performance-test)
            NAVIGATION_PERFORMANCE_TEST=true
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
            echo "Usage: $0 [--skip-agent] [--only-agent] [--debug] [--simulation] [--regression] [--explore] [--nav] [--b4m-api] [--ollama] [--ollama-advanced] [--ollama-nav] [--ollama-nav-basic] [--b4m-HA] [--b4m-ping] [--navigation-performance-test] [--setup-wifi] [--skip-rebuild]"
            echo "  --skip-agent:                   Skip the Micro-ROS agent launch (Step 1)"
            echo "  --only-agent:                   Launch ONLY the Micro-ROS agent (Step 1) and exit"
            echo "  --debug:                        Enable verbose debug logging"
            echo "  --simulation:                   Launch in Gazebo simulation mode instead of real robot"
            echo "  --regression:                   Run comprehensive regression test suite (navigation + laser stability)"
            echo "  --explore:                      Enable autonomous exploration mode with obstacle avoidance"
            echo "  --nav:                          Enable Navigation 2 with SLAM for goal-based navigation in RViz"
            echo "  --b4m-api:                      Enable B4M API mode (duplicate of --explore for API integration)"
            echo "  --ollama:                       Enable Ollama mode (basic cardinal directions)"
            echo "  --ollama-advanced:              Enable Ollama Advanced mode (360° spatial context)"
            echo "  --ollama-nav:                   Enable Ollama Navigation mode (LLM-guided Nav2 goal selection)"
            echo "  --ollama-nav-basic:             Enable basic Ollama Navigation mode (copy of --nav for testing)"
	    echo "  --b4m-HA:                       (Experimental) Enable Home Assistant MQTT integration features"
            echo "  --b4m-ping:                     Test bike4mind API with random obstacle detection messages"
	    echo "  --navigation-performance-test:  (Untested) Execute 1x1m square navigation circuit testing"
	    echo "  --setup-wifi:                   Interactive WiFi setup wizard for robot configuration"
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Get the workspace root directory (where this script is located)
WORKSPACE_ROOT=$(cd "$(dirname "$0")" && pwd)

# Create logs directory if it doesn't exist
LOGS_DIR="$WORKSPACE_ROOT/logs"
mkdir -p "$LOGS_DIR"

# Generate timestamp for log files
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAIN_LOG="$LOGS_DIR/b4m_launch_$TIMESTAMP.log"

# Validate argument combinations
if [ "$EXPLORE_MODE" = true ] && [ "$NAV_MODE" = true ]; then
    echo "ERROR: --explore and --nav modes cannot be used together"
    echo "Please choose either exploration or navigation mode"
    exit 1
fi

if [ "$EXPLORE_MODE" = true ] && [ "$REGRESSION_MODE" = true ]; then
    echo "ERROR: --explore mode is incompatible with --regression mode"
    echo "Exploration requires manual control while regression runs automated tests"
    exit 1
fi

if [ "$NAV_MODE" = true ] && [ "$REGRESSION_MODE" = true ]; then
    echo "ERROR: --nav mode is incompatible with --regression mode"
    echo "Navigation requires manual goal setting while regression runs automated tests"
    exit 1
fi


if [ "$EXPLORE_MODE" = true ] && [ "$NAVIGATION_PERFORMANCE_TEST" = true ]; then
    echo "ERROR: --explore mode is incompatible with --navigation-performance-test mode"
    echo "Both modes require different robot control patterns"
    exit 1
fi

if [ "$NAV_MODE" = true ] && [ "$NAVIGATION_PERFORMANCE_TEST" = true ]; then
    echo "ERROR: --nav mode is incompatible with --navigation-performance-test mode"
    echo "Both modes require different robot control patterns"
    exit 1
fi

if [ "$NAV_MODE" = true ] && [ "$B4M_API" = true ]; then
    echo "ERROR: --nav mode is incompatible with --b4m-api mode"
    echo "Both modes provide different navigation approaches"
    exit 1
fi

if [ "$NAV_MODE" = true ] && [ "$OLLAMA_MODE" = true ]; then
    echo "ERROR: --nav mode is incompatible with --ollama mode"
    echo "Both modes provide different navigation approaches"
    exit 1
fi

if [ "$B4M_API" = true ] && [ "$REGRESSION_MODE" = true ]; then
    echo "ERROR: --b4m-api mode is incompatible with --regression mode"
    echo "B4M API requires manual control while regression runs automated tests"
    exit 1
fi

if [ "$OLLAMA_MODE" = true ] && [ "$REGRESSION_MODE" = true ]; then
    echo "ERROR: --ollama mode is incompatible with --regression mode"
    echo "Ollama mode requires manual control while regression runs automated tests"
    exit 1
fi

if [ "$B4M_API" = true ] && [ "$NAVIGATION_PERFORMANCE_TEST" = true ]; then
    echo "ERROR: --b4m-api mode is incompatible with --navigation-performance-test mode"
    echo "Both modes require different robot control patterns"
    exit 1
fi

if [ "$OLLAMA_MODE" = true ] && [ "$NAVIGATION_PERFORMANCE_TEST" = true ]; then
    echo "ERROR: --ollama mode is incompatible with --navigation-performance-test mode"
    echo "Both modes require different robot control patterns"
    exit 1
fi

if [ "$B4M_API" = true ] && [ "$OLLAMA_MODE" = true ]; then
    echo "ERROR: --b4m-api mode is incompatible with --ollama mode"
    echo "Both modes provide different LLM integration approaches"
    exit 1
fi

if [ "$OLLAMA_MODE" = true ] && [ "$OLLAMA_ADVANCED_MODE" = true ]; then
    echo "ERROR: --ollama and --ollama-advanced modes cannot be used together"
    echo "Choose either basic or advanced Ollama integration"
    exit 1
fi
if [ "$OLLAMA_MODE" = true ] && [ "$OLLAMA_NAV_MODE" = true ]; then
    echo "ERROR: --ollama and --ollama-nav modes cannot be used together"
    echo "Choose either basic Ollama or Nav2-integrated Ollama mode"
    exit 1
fi
if [ "$OLLAMA_ADVANCED_MODE" = true ] && [ "$OLLAMA_NAV_MODE" = true ]; then
    echo "ERROR: --ollama-advanced and --ollama-nav modes cannot be used together"
    echo "Choose either advanced spatial context or Nav2-integrated mode"
    exit 1
fi

if [ "$B4M_API" = true ] && [ "$OLLAMA_ADVANCED_MODE" = true ]; then
    echo "ERROR: --b4m-api mode is incompatible with --ollama-advanced mode"
    echo "Both modes provide different navigation approaches"
    exit 1
fi

if [ "$NAV_MODE" = true ] && [ "$OLLAMA_ADVANCED_MODE" = true ]; then
    echo "ERROR: --nav mode is incompatible with --ollama-advanced mode"
    echo "Both modes provide different navigation approaches"
    exit 1
fi

if [ "$OLLAMA_ADVANCED_MODE" = true ] && [ "$REGRESSION_MODE" = true ]; then
    echo "ERROR: --ollama-advanced mode is incompatible with --regression mode"
    echo "Advanced Ollama mode requires manual control while regression runs automated tests"
    exit 1
fi

if [ "$OLLAMA_ADVANCED_MODE" = true ] && [ "$NAVIGATION_PERFORMANCE_TEST" = true ]; then
    echo "ERROR: --ollama-advanced mode is incompatible with --navigation-performance-test mode"
    echo "Both modes require different robot control patterns"
    exit 1
fi
if [ "$OLLAMA_NAV_MODE" = true ] && [ "$EXPLORE_MODE" = true ]; then
    echo "ERROR: --ollama-nav mode is incompatible with --explore mode"
    echo "Both modes provide different navigation approaches"
    exit 1
fi
if [ "$OLLAMA_NAV_MODE" = true ] && [ "$NAV_MODE" = true ]; then
    echo "ERROR: --ollama-nav mode is incompatible with --nav mode"
    echo "Ollama-nav already includes Navigation 2 functionality"
    exit 1
fi
if [ "$OLLAMA_NAV_MODE" = true ] && [ "$B4M_API" = true ]; then
    echo "ERROR: --ollama-nav mode is incompatible with --b4m-api mode"
    echo "Both modes provide different navigation approaches"
    exit 1
fi
if [ "$OLLAMA_NAV_MODE" = true ] && [ "$REGRESSION_MODE" = true ]; then
    echo "ERROR: --ollama-nav mode is incompatible with --regression mode"
    echo "Ollama navigation requires LLM service while regression runs automated tests"
    exit 1
fi
if [ "$OLLAMA_NAV_MODE" = true ] && [ "$NAVIGATION_PERFORMANCE_TEST" = true ]; then
    echo "ERROR: --ollama-nav mode is incompatible with --navigation-performance-test mode"
    echo "Both modes require different robot control patterns"
    exit 1
fi

# Ollama Nav Basic mode incompatibility checks
if [ "$OLLAMA_NAV_BASIC_MODE" = true ] && [ "$EXPLORE_MODE" = true ]; then
    echo "ERROR: --ollama-nav-basic mode is incompatible with --explore mode"
    echo "Both modes provide different navigation approaches"
    exit 1
fi
if [ "$OLLAMA_NAV_BASIC_MODE" = true ] && [ "$NAV_MODE" = true ]; then
    echo "ERROR: --ollama-nav-basic mode is incompatible with --nav mode"
    echo "Ollama-nav-basic is a copy of --nav functionality"
    exit 1
fi
if [ "$OLLAMA_NAV_BASIC_MODE" = true ] && [ "$B4M_API" = true ]; then
    echo "ERROR: --ollama-nav-basic mode is incompatible with --b4m-api mode"
    echo "Both modes provide different navigation approaches"
    exit 1
fi
if [ "$OLLAMA_NAV_BASIC_MODE" = true ] && [ "$OLLAMA_MODE" = true ]; then
    echo "ERROR: --ollama-nav-basic mode is incompatible with --ollama mode"
    echo "Both modes provide different navigation approaches"
    exit 1
fi
if [ "$OLLAMA_NAV_BASIC_MODE" = true ] && [ "$OLLAMA_ADVANCED_MODE" = true ]; then
    echo "ERROR: --ollama-nav-basic mode is incompatible with --ollama-advanced mode"
    echo "Both modes provide different navigation approaches"
    exit 1
fi
if [ "$OLLAMA_NAV_BASIC_MODE" = true ] && [ "$OLLAMA_NAV_MODE" = true ]; then
    echo "ERROR: --ollama-nav-basic mode is incompatible with --ollama-nav mode"
    echo "Both modes provide different Ollama navigation approaches"
    exit 1
fi
if [ "$OLLAMA_NAV_BASIC_MODE" = true ] && [ "$REGRESSION_MODE" = true ]; then
    echo "ERROR: --ollama-nav-basic mode is incompatible with --regression mode"  
    echo "Ollama navigation basic requires manual control while regression runs automated tests"
    exit 1
fi
if [ "$OLLAMA_NAV_BASIC_MODE" = true ] && [ "$NAVIGATION_PERFORMANCE_TEST" = true ]; then
    echo "ERROR: --ollama-nav-basic mode is incompatible with --navigation-performance-test mode"
    echo "Both modes require different robot control patterns"
    exit 1
fi

# B4M Ping mode incompatibility checks
if [ "$B4M_PING" = true ]; then
    if [ "$EXPLORE_MODE" = true ] || [ "$NAV_MODE" = true ] || [ "$B4M_API" = true ] || [ "$OLLAMA_MODE" = true ] || [ "$OLLAMA_ADVANCED_MODE" = true ] || [ "$OLLAMA_NAV_MODE" = true ] || [ "$OLLAMA_NAV_BASIC_MODE" = true ] || [ "$REGRESSION_MODE" = true ] || [ "$NAVIGATION_PERFORMANCE_TEST" = true ]; then
        echo "ERROR: --b4m-ping is incompatible with other modes"
        echo "B4M Ping is a standalone API testing tool"
        exit 1
    fi
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
        echo "Next steps:"
        echo "1. Disconnect the USB cable from the robot"
        echo "2. Power cycle the robot (turn off and on)"
        echo "3. Wait about 10-15 seconds for WiFi connection"
        echo "4. Run ./b4m_launch.sh to start the robot system"
        echo ""
        echo "WiFi setup complete. Exiting."
        exit 0
        
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

# Handle WiFi setup mode
if [ "$SETUP_WIFI" = true ]; then
    echo "🔧 WiFi Setup Mode"
    echo "================="
    echo "Starting interactive WiFi configuration wizard..."
    echo ""
    setup_wifi_interactive
    # WiFi setup is complete, exit the script
    exit 0
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

# Handle Navigation 2 mode with SLAM
if [ "$NAV_MODE" = true ]; then
    echo "🧭 NAVIGATION 2 WITH SLAM MODE"
    echo "======================================"
    echo "Launching Navigation 2 with Cartographer SLAM for goal-based navigation"
    
    if [ "$SIMULATION_MODE" = true ]; then
        echo "Mode: Gazebo Classic Simulation with Navigation World"
        WORLD_NAME="exploration_test_classic"  # Use exploration world for navigation
    else  
        echo "Mode: Real Robot Navigation with SLAM"
    fi
    
    echo ""
    echo "This mode will:"
    echo "- Launch Cartographer for real-time SLAM mapping"
    echo "- Start Navigation 2 stack for path planning and obstacle avoidance"
    echo "- Allow goal setting via RViz 2D Nav Goal tool"
    echo "- Build and use map simultaneously for navigation"
    echo ""
    
    # Ensure clean state before launching
    echo "🧹 ENSURING CLEAN STATE FOR NAVIGATION"
    echo "======================================"
    echo "Cleaning up any existing ROS2 processes to prevent TF conflicts..."
    ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
    sleep 3
    echo "✅ System cleanup completed"
    echo ""
    
    # Launch navigation sequence
    echo "🚀 NAVIGATION 2 LAUNCH SEQUENCE"
    echo "======================================"
    
    if [ "$SIMULATION_MODE" = true ]; then
        # Step 1: Launch Gazebo Classic simulation
        echo "🎮 Step 1: Starting Gazebo Classic simulation with navigation world"
        ros2 launch yahboomcar_nav gazebo_classic_nav_launch.py world_name:=$WORLD_NAME > "$LOGS_DIR/nav_gazebo_$TIMESTAMP.log" 2>&1 &
        GAZEBO_PID=$!
        echo "   Waiting for simulation initialization..."
        sleep 10
        
        # Verify laser scan is being published
        echo "   Verifying laser scan topic..."
        if timeout 5 ros2 topic echo /scan --once > /dev/null 2>&1; then
            echo "   ✅ Laser scan topic active"
        else
            echo "   ⚠️  Warning: Laser scan topic not detected"
        fi
        
        # Step 2: Launch robot bringup for EKF and sensor processing
        echo "🤖 Step 2: Starting robot sensor and control systems (EKF, IMU, etc.)"
        echo "   This is required even in simulation to get filtered /odom from EKF"
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py use_sim_time:=true > "$LOGS_DIR/nav_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # Verify odometry topic
        echo "   Verifying odometry is available..."
        timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ Odometry topic verified"
        else
            echo "   ⚠️  Warning: /odom topic not ready, continuing anyway..."
        fi
        
        # Step 3: Launch RViz for visualization
        echo "📊 Step 3: Starting RViz for map visualization and navigation"
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=true > "$LOGS_DIR/nav_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 5
        
    else
        echo "   ⚠️  Make sure physical robot is powered on and ready"
        echo "   ⚠️  Ensure navigation area is safe"
        read -p "   Press Enter when robot is ready for navigation..."
        
        # Step 1: Launch robot bringup 
        echo "🤖 Step 1: Starting robot sensor and control systems"
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py > "$LOGS_DIR/nav_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # Verify odometry topic
        echo "   Verifying odometry is available..."
        timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ Odometry topic verified"
        else
            echo "   ⚠️  Warning: /odom topic not ready, continuing anyway..."
        fi
        
        # Step 2: Launch RViz for visualization
        echo "📊 Step 2: Starting RViz for map visualization and navigation"  
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=false > "$LOGS_DIR/nav_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 3
    fi
    
    # Step 4: Launch TF bridge to connect odom_frame to odom
    echo "🔗 Step 4: Creating TF bridge between odom_frame and odom"
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom_frame odom > "$LOGS_DIR/nav_tf_bridge_$TIMESTAMP.log" 2>&1 &
    TF_BRIDGE_PID=$!
    echo "   Bridge created: odom_frame → odom (connects Cartographer to robot)"
    
    # Step 5: Launch SLAM Navigation (Cartographer + Nav2)
    echo "🗺️  Step 5: Starting SLAM Navigation with Cartographer and Nav2"
    if [ "$SIMULATION_MODE" = true ]; then
        # Launch combined Cartographer SLAM and Navigation 2 stack
        ros2 launch yahboomcar_nav cartographer_nav2_launch.py use_sim_time:=true > "$LOGS_DIR/nav_slam_navigation_$TIMESTAMP.log" 2>&1 &
    else
        ros2 launch yahboomcar_nav cartographer_nav2_launch.py use_sim_time:=false > "$LOGS_DIR/nav_slam_navigation_$TIMESTAMP.log" 2>&1 &
    fi
    SLAM_NAV_PID=$!
    echo "   Waiting for SLAM and Navigation stack initialization..."
    sleep 15  # Give more time for the full nav stack to initialize
    
    # Verify map topic is being published
    echo "   Verifying map topic..."
    if timeout 10 ros2 topic echo /map --once > /dev/null 2>&1; then
        echo "   ✅ Map topic active"
    else
        echo "   ⚠️  Warning: Map topic not detected yet (may appear shortly)"
    fi
    
    # Verify TF tree is complete
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
    
    # Verify Nav2 services are available
    echo "   Verifying Navigation 2 services..."
    timeout 5 ros2 service list | grep -q "navigate_to_pose"
    if [ $? -eq 0 ]; then
        echo "   ✅ Nav2 navigate_to_pose service available"
    else
        echo "   ⚠️  Warning: Nav2 services not detected yet"
    fi
    
    echo ""
    echo "✅ NAVIGATION 2 WITH SLAM ACTIVE"
    echo "======================================"
    echo "🧭 Robot is ready for goal-based navigation with SLAM"
    echo "📊 Use RViz to:"
    echo "   1. Monitor real-time map building: /map topic"
    echo "   2. Set navigation goals: Use '2D Nav Goal' tool in RViz toolbar"
    echo "   3. Watch path planning: Global and local costmaps"
    echo "   4. Track robot position: /tf displays robot location"
    echo ""
    echo "🎯 To navigate:"
    echo "   - Click '2D Nav Goal' button in RViz toolbar"
    echo "   - Click and drag on map to set goal position and orientation"
    echo "   - Robot will plan path and navigate while avoiding obstacles"
    echo ""
    echo "🛑 To stop navigation:"
    echo "   - Press Ctrl+C in this terminal, OR"
    echo "   - Run: ./b4m_shutdown.sh --keep-agent"
    echo ""
    echo "💡 Tips:"
    echo "   - The map builds as the robot moves and explores"
    echo "   - Set goals in already-mapped areas for best results"
    echo "   - Robot will re-plan if obstacles block the path"
    
    # Wait for user to stop
    trap 'echo "🛑 Stopping navigation..."; [ ! -z "$SLAM_NAV_PID" ] && kill $SLAM_NAV_PID 2>/dev/null; [ ! -z "$TF_BRIDGE_PID" ] && kill $TF_BRIDGE_PID 2>/dev/null; [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null; [ ! -z "$BRINGUP_PID" ] && kill $BRINGUP_PID 2>/dev/null; if [ "$SIMULATION_MODE" = true ]; then [ ! -z "$GAZEBO_PID" ] && kill $GAZEBO_PID 2>/dev/null; fi; ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1; echo "✅ Navigation stopped"; exit 0' INT
    
    # Keep the script running and show periodic status
    while true; do
        sleep 30
        echo "🧭 Navigation system active... (Ctrl+C to stop)"
        echo "   Set navigation goals using RViz '2D Nav Goal' tool"
    done
fi

# Handle Ollama Navigation Basic mode (exact copy of Navigation 2 mode with SLAM for testing)
if [ "$OLLAMA_NAV_BASIC_MODE" = true ]; then
    echo "🧭 OLLAMA NAVIGATION BASIC MODE (Nav2 Copy)"
    echo "======================================"
    echo "Launching Navigation 2 with Cartographer SLAM (copy of --nav for testing)"
    
    if [ "$SIMULATION_MODE" = true ]; then
        echo "Mode: Gazebo Classic Simulation with Navigation World"
        WORLD_NAME="exploration_test_classic"  # Use exploration world for navigation
    else  
        echo "Mode: Real Robot Navigation with SLAM"
    fi
    
    echo ""
    echo "This mode will:"
    echo "- Launch Cartographer for real-time SLAM mapping"
    echo "- Start Navigation 2 stack for path planning and obstacle avoidance"
    echo "- Allow goal setting via RViz 2D Nav Goal tool"
    echo "- Build and use map simultaneously for navigation"
    echo "- (Future: Add automatic 2D pose setting and Ollama features)"
    echo ""
    
    # Ensure clean state before launching
    echo "🧹 ENSURING CLEAN STATE FOR NAVIGATION"
    echo "======================================"
    echo "Cleaning up any existing ROS2 processes to prevent TF conflicts..."
    ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
    sleep 3
    echo "✅ System cleanup completed"
    echo ""
    
    # Launch navigation sequence
    echo "🚀 NAVIGATION 2 LAUNCH SEQUENCE"
    echo "======================================"
    
    if [ "$SIMULATION_MODE" = true ]; then
        # Step 1: Launch Gazebo Classic simulation
        echo "🎮 Step 1: Starting Gazebo Classic simulation with navigation world"
        ros2 launch yahboomcar_nav gazebo_classic_nav_launch.py world_name:=$WORLD_NAME > "$LOGS_DIR/nav_gazebo_$TIMESTAMP.log" 2>&1 &
        GAZEBO_PID=$!
        echo "   Waiting for simulation initialization..."
        sleep 10
        
        # Verify laser scan is being published
        echo "   Verifying laser scan topic..."
        if timeout 5 ros2 topic echo /scan --once > /dev/null 2>&1; then
            echo "   ✅ Laser scan topic active"
        else
            echo "   ⚠️  Warning: Laser scan topic not detected"
        fi
        
        # Step 2: Launch robot bringup for EKF and sensor processing
        echo "🤖 Step 2: Starting robot sensor and control systems (EKF, IMU, etc.)"
        echo "   This is required even in simulation to get filtered /odom from EKF"
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py use_sim_time:=true > "$LOGS_DIR/nav_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # Verify odometry topic
        echo "   Verifying odometry is available..."
        timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ Odometry topic verified"
        else
            echo "   ⚠️  Warning: /odom topic not ready, continuing anyway..."
        fi
        
        # Step 3: Launch RViz for visualization
        echo "📊 Step 3: Starting RViz for map visualization and navigation"
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=true > "$LOGS_DIR/nav_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 5
        
    else
        echo "   ⚠️  Make sure physical robot is powered on and ready"
        echo "   ⚠️  Ensure navigation area is safe"
        read -p "   Press Enter when robot is ready for navigation..."
        
        # Step 1: Launch robot bringup 
        echo "🤖 Step 1: Starting robot sensor and control systems"
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py > "$LOGS_DIR/nav_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # Verify odometry topic
        echo "   Verifying odometry is available..."
        timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ Odometry topic verified"
        else
            echo "   ⚠️  Warning: /odom topic not ready, continuing anyway..."
        fi
        
        # Step 2: Launch RViz for visualization
        echo "📊 Step 2: Starting RViz for map visualization and navigation"  
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=false > "$LOGS_DIR/nav_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 3
    fi
    
    # Step 4: Launch TF bridge to connect odom_frame to odom
    echo "🔗 Step 4: Creating TF bridge between odom_frame and odom"
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom_frame odom > "$LOGS_DIR/nav_tf_bridge_$TIMESTAMP.log" 2>&1 &
    TF_BRIDGE_PID=$!
    echo "   Bridge created: odom_frame → odom (connects Cartographer to robot)"
    
    # Step 5: Launch SLAM Navigation (Cartographer + Nav2)
    echo "🗺️  Step 5: Starting SLAM Navigation with Cartographer and Nav2"
    if [ "$SIMULATION_MODE" = true ]; then
        # Launch combined Cartographer SLAM and Navigation 2 stack
        ros2 launch yahboomcar_nav cartographer_nav2_launch.py use_sim_time:=true > "$LOGS_DIR/nav_slam_navigation_$TIMESTAMP.log" 2>&1 &
    else
        ros2 launch yahboomcar_nav cartographer_nav2_launch.py use_sim_time:=false > "$LOGS_DIR/nav_slam_navigation_$TIMESTAMP.log" 2>&1 &
    fi
    SLAM_NAV_PID=$!
    echo "   Waiting for SLAM and Navigation stack initialization..."
    sleep 15  # Give more time for the full nav stack to initialize
    
    # Verify map topic is being published
    echo "   Verifying map topic..."
    if timeout 10 ros2 topic echo /map --once > /dev/null 2>&1; then
        echo "   ✅ Map topic active"
    else
        echo "   ⚠️  Warning: Map topic not detected yet (may appear shortly)"
    fi
    
    # Verify TF tree is complete
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
    
    # Verify Nav2 services are available
    echo "   Verifying Navigation 2 services..."
    timeout 5 ros2 service list | grep -q "navigate_to_pose"
    if [ $? -eq 0 ]; then
        echo "   ✅ Nav2 navigate_to_pose service available"
    else
        echo "   ⚠️  Warning: Nav2 services not detected yet"
    fi
    
    # Step 6: Set automatic initial pose (replaces manual 2D pose estimation)
    echo "🎯 Step 6: Setting automatic initial pose at map center"
    echo "   This replaces manual 2D pose estimation in RViz"
    python3 scripts/set_initial_pose.py > "$LOGS_DIR/nav_initial_pose_$TIMESTAMP.log" 2>&1
    if [ $? -eq 0 ]; then
        echo "   ✅ Automatic initial pose set at (0.0, 0.0)"
    else
        echo "   ⚠️  Warning: Initial pose setting failed, check logs"
    fi
    
    echo ""
    echo "✅ OLLAMA NAVIGATION BASIC ACTIVE (Nav2 Copy)"
    echo "======================================"
    echo "🧭 Robot is ready for goal-based navigation with SLAM"
    echo "📊 Use RViz to:"
    echo "   1. Monitor real-time map building: /map topic"
    echo "   2. Set navigation goals: Use '2D Nav Goal' tool in RViz toolbar"  
    echo "   3. Watch path planning: Global and local costmaps"
    echo "   4. Track robot position: /tf displays robot location"
    echo "   5. Note: Initial pose is set automatically - no manual '2D Pose Estimate' needed"
    echo ""
    echo "🎯 To navigate:"
    echo "   - Click '2D Nav Goal' button in RViz toolbar"
    echo "   - Click and drag on map to set goal position and orientation"
    echo "   - Robot will plan path and navigate while avoiding obstacles"
    echo ""
    echo "🛑 To stop navigation:"
    echo "   - Press Ctrl+C in this terminal, OR"
    echo "   - Run: ./b4m_shutdown.sh --keep-agent"
    echo ""
    echo "💡 Tips:"
    echo "   - The map builds as the robot moves and explores"
    echo "   - Set goals in already-mapped areas for best results"
    echo "   - Robot will re-plan if obstacles block the path"
    echo ""
    echo "🔬 Testing Status:"
    echo "   - ✅ Basic Nav2 functionality (copied from --nav)"
    echo "   - ✅ Automatic 2D pose setting (implemented and active)"
    echo "   - ✅ Ollama spatial analysis (basic implementation ready)"
    echo "   - 🚧 LLM goal selection (basic implementation ready)"
    echo ""
    
    # Ask user if they want to enable Ollama features
    echo "🤖 OLLAMA INTEGRATION OPTIONS"
    echo "======================================"
    echo "Basic Nav2 navigation is ready. You can:"
    echo "1. Test manual navigation using RViz '2D Nav Goal' tool"
    echo "2. Enable Ollama spatial analysis for autonomous goal suggestions"
    echo ""
    read -p "Enable Ollama spatial analysis? (y/N): " enable_ollama
    
    if [[ "$enable_ollama" =~ ^[Yy]$ ]]; then
        echo "🧠 Step 7: Starting Ollama Basic Spatial Analysis"
        echo "   This will analyze surroundings every 30 seconds and suggest navigation goals"
        python3 scripts/ollama_basic_spatial.py > "$LOGS_DIR/ollama_spatial_$TIMESTAMP.log" 2>&1 &
        OLLAMA_PID=$!
        echo "   ✅ Ollama spatial analysis started (PID: $OLLAMA_PID)"
        echo ""
        echo "🤖 AUTONOMOUS NAVIGATION ACTIVE"
        echo "======================================"
        echo "   - Robot will analyze surroundings every 30 seconds"
        echo "   - Ollama LLM will suggest navigation goals based on spatial context"
        echo "   - Goals will appear as RViz markers and robot will navigate to them"
        echo "   - Check logs for Ollama reasoning: $LOGS_DIR/ollama_spatial_$TIMESTAMP.log"
        echo ""
    else
        echo "📋 Manual navigation mode - use RViz '2D Nav Goal' tool to set goals"
        echo ""
    fi
    
    # Wait for user to stop
    trap 'echo "🛑 Stopping navigation..."; [ ! -z "$OLLAMA_PID" ] && kill $OLLAMA_PID 2>/dev/null; [ ! -z "$SLAM_NAV_PID" ] && kill $SLAM_NAV_PID 2>/dev/null; [ ! -z "$TF_BRIDGE_PID" ] && kill $TF_BRIDGE_PID 2>/dev/null; [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null; [ ! -z "$BRINGUP_PID" ] && kill $BRINGUP_PID 2>/dev/null; if [ "$SIMULATION_MODE" = true ]; then [ ! -z "$GAZEBO_PID" ] && kill $GAZEBO_PID 2>/dev/null; fi; ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1; echo "✅ Navigation stopped"; exit 0' INT
    
    # Keep the script running and show periodic status
    while true; do
        sleep 30
        if [ ! -z "$OLLAMA_PID" ]; then
            echo "🧭 Ollama Navigation Basic active with AI... (Ctrl+C to stop)"
            echo "   Robot analyzing surroundings every 30 seconds and suggesting goals via Ollama LLM"
            echo "   Check Ollama logs: $LOGS_DIR/ollama_spatial_$TIMESTAMP.log"
        else
            echo "🧭 Ollama Navigation Basic active... (Ctrl+C to stop)"
            echo "   Set navigation goals using RViz '2D Nav Goal' tool"
            echo "   Manual navigation mode - Ollama analysis not enabled"
        fi
    done
fi

# Handle B4M API mode (same as explore mode but with spatial interpreter)
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
    
    # Ensure clean state before launching B4M API
    echo "🧹 ENSURING CLEAN STATE FOR B4M API"
    echo "======================================"
    echo "Cleaning up any existing ROS2 processes to prevent TF conflicts..."
    ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
    sleep 3
    echo "✅ System cleanup completed"
    echo ""
    
    # Launch B4M API sequence (identical to exploration sequence)
    echo "🚀 B4M API LAUNCH SEQUENCE"
    echo "======================================"
    
    if [ "$SIMULATION_MODE" = true ]; then
        # Step 1: Launch Gazebo Classic simulation with exploration world
        echo "🎮 Step 1: Starting Gazebo Classic simulation with exploration world"
        ros2 launch yahboomcar_nav gazebo_classic_nav_launch.py world_name:=$WORLD_NAME > "$LOGS_DIR/b4m_api_gazebo_$TIMESTAMP.log" 2>&1 &
        GAZEBO_PID=$!
        echo "   Waiting for simulation initialization..."
        sleep 10
        
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
        
        # Verify odometry topic
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
        sleep 5
        
    else
        echo "   ⚠️  Make sure physical robot is powered on and ready"
        echo "   ⚠️  Ensure exploration area is safe and obstacle-free"
        read -p "   Press Enter when robot is ready for B4M API exploration..."
        
        # Step 1: Launch robot bringup 
        echo "🤖 Step 1: Starting robot sensor and control systems"
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py > "$LOGS_DIR/b4m_api_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # Verify odometry topic
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
    sleep 10
    
    # Verify map topic is being published
    echo "   Verifying map topic..."
    if timeout 10 ros2 topic echo /map --once > /dev/null 2>&1; then
        echo "   ✅ Map topic active"
    else
        echo "   ⚠️  Warning: Map topic not detected yet (may appear shortly)"
    fi
    
    # Verify TF tree is complete before starting B4M API
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
    
    # Step 6: Start B4M Spatial Interpreter in foreground
    echo "🧠 Step 6: Starting B4M Spatial Interpreter"
    echo "   All output and interaction will happen in this terminal"
    echo ""
    echo "======================================================="
    echo "🧠 B4M Spatial Interpreter - Interactive Console" 
    echo "======================================================="
    echo "This console provides spatial descriptions and navigation control"
    echo "when the robot encounters obstacles."
    echo ""
    echo "🚀 Starting B4M Spatial Interpreter..."
    echo "======================================================="
    
    # Run the spatial interpreter in foreground with logging
    python3 "$WORKSPACE_ROOT/scripts/b4m_spatial_interpreter.py" 2>&1 | tee "$LOGS_DIR/b4m_api_spatial_$TIMESTAMP.log"
    
    # The spatial interpreter will run here in foreground
    # When it exits (Ctrl+C or completion), we'll clean up
    echo ""
    echo "✅ B4M API mode finished"
    echo "🧹 Cleaning up..."
    
    # Clean up background processes
    [ ! -z "$CARTOGRAPHER_PID" ] && kill $CARTOGRAPHER_PID 2>/dev/null
    [ ! -z "$TF_BRIDGE_PID" ] && kill $TF_BRIDGE_PID 2>/dev/null  
    [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null
    [ ! -z "$BRINGUP_PID" ] && kill $BRINGUP_PID 2>/dev/null
    if [ "$SIMULATION_MODE" = true ]; then
        [ ! -z "$GAZEBO_PID" ] && kill $GAZEBO_PID 2>/dev/null
    fi
    
    ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
    echo "✅ B4M API mode cleanup completed"
    exit 0
fi

# Handle Ollama mode (same as B4M API mode but with Ollama integration)
if [ "$OLLAMA_MODE" = true ]; then
    echo "🦙 OLLAMA MODE"
    echo "======================================"
    echo "Launching Ollama mode with Cartographer mapping (identical to --b4m-api)"
    
    if [ "$SIMULATION_MODE" = true ]; then
        echo "Mode: Gazebo Classic Simulation with Exploration World"
        WORLD_NAME="exploration_test_classic"  # Use exploration-specific world
    else  
        echo "Mode: Real Robot Ollama Exploration"
    fi
    
    echo ""
    echo "This mode will:"
    echo "- Launch Cartographer for real-time SLAM mapping"
    echo "- Start autonomous exploration with obstacle avoidance"
    echo "- Build a map while navigating safely"
    echo "- Stop exploration when area is sufficiently mapped"
    echo "- Provide Ollama LLM integration for spatial understanding"
    echo ""
    
    # Ensure clean state before launching Ollama mode
    echo "🧹 ENSURING CLEAN STATE FOR OLLAMA MODE"
    echo "======================================"
    echo "Cleaning up any existing ROS2 processes to prevent TF conflicts..."
    ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
    sleep 3
    echo "✅ System cleanup completed"
    echo ""
    
    # Launch Ollama sequence (identical to B4M API sequence)
    echo "🚀 OLLAMA LAUNCH SEQUENCE"
    echo "======================================"
    
    if [ "$SIMULATION_MODE" = true ]; then
        # Step 1: Launch Gazebo Classic simulation with exploration world
        echo "🎮 Step 1: Starting Gazebo Classic simulation with exploration world"
        ros2 launch yahboomcar_nav gazebo_classic_nav_launch.py world_name:=$WORLD_NAME > "$LOGS_DIR/ollama_gazebo_$TIMESTAMP.log" 2>&1 &
        GAZEBO_PID=$!
        echo "   Waiting for simulation initialization..."
        sleep 10
        
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
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py use_sim_time:=true > "$LOGS_DIR/ollama_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # Verify odometry topic
        echo "   Verifying odometry is available..."
        timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ Odometry topic verified"
        else
            echo "   ⚠️  Warning: /odom topic not ready, continuing anyway..."
        fi
        
        # Step 3: Launch RViz for visualization
        echo "📊 Step 3: Starting RViz for map visualization"
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=true > "$LOGS_DIR/ollama_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 5
        
    else
        echo "   ⚠️  Make sure physical robot is powered on and ready"
        echo "   ⚠️  Ensure exploration area is safe and obstacle-free"
        read -p "   Press Enter when robot is ready for Ollama exploration..."
        
        # Step 1: Launch robot bringup 
        echo "🤖 Step 1: Starting robot sensor and control systems"
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py > "$LOGS_DIR/ollama_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # Verify odometry topic
        echo "   Verifying odometry is available..."
        timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ Odometry topic verified"
        else
            echo "   ⚠️  Warning: /odom topic not ready, continuing anyway..."
        fi
        
        # Step 2: Launch RViz for visualization
        echo "📊 Step 2: Starting RViz for map visualization"  
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=false > "$LOGS_DIR/ollama_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 3
    fi
    
    # Step 4: Launch TF bridge to connect odom_frame to odom
    echo "🔗 Step 4: Creating TF bridge between odom_frame and odom"
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom_frame odom > "$LOGS_DIR/ollama_tf_bridge_$TIMESTAMP.log" 2>&1 &
    TF_BRIDGE_PID=$!
    echo "   Bridge created: odom_frame → odom (connects Cartographer to robot)"
    
    # Step 5: Launch Cartographer SLAM for real-time mapping
    echo "🗺️  Step 5: Starting Cartographer SLAM for real-time mapping"
    if [ "$SIMULATION_MODE" = true ]; then
        ros2 launch yahboomcar_nav map_cartographer_launch.py use_sim_time:=true > "$LOGS_DIR/ollama_cartographer_$TIMESTAMP.log" 2>&1 &
    else
        ros2 launch yahboomcar_nav map_cartographer_launch.py use_sim_time:=false > "$LOGS_DIR/ollama_cartographer_$TIMESTAMP.log" 2>&1 &
    fi
    CARTOGRAPHER_PID=$!
    echo "   Waiting for SLAM system initialization..."
    sleep 10
    
    # Verify map topic is being published
    echo "   Verifying map topic..."
    if timeout 10 ros2 topic echo /map --once > /dev/null 2>&1; then
        echo "   ✅ Map topic active"
    else
        echo "   ⚠️  Warning: Map topic not detected yet (may appear shortly)"
    fi
    
    # Verify TF tree is complete before starting Ollama
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
    
    # Step 6: Start Ollama Spatial Interpreter in foreground
    echo "🦙 Step 6: Starting Ollama Spatial Interpreter"
    echo "   All output and interaction will happen in this terminal"
    echo ""
    echo "======================================================="
    echo "🦙 Ollama Spatial Interpreter - Interactive Console" 
    echo "======================================================="
    echo "This console provides spatial descriptions and navigation control"
    echo "with Ollama LLM integration for enhanced spatial understanding."
    echo ""
    echo "🚀 Starting Ollama Spatial Interpreter..."
    echo "======================================================="
    
    # Run the spatial interpreter in foreground with logging, passing --ollama-mode flag
    # Use -u flag for unbuffered output to ensure real-time console display
    python3 -u "$WORKSPACE_ROOT/scripts/b4m_spatial_interpreter.py" --ollama-mode 2>&1 | tee "$LOGS_DIR/ollama_spatial_$TIMESTAMP.log"
    
    # The spatial interpreter will run here in foreground
    # When it exits (Ctrl+C or completion), we'll clean up
    echo ""
    echo "✅ Ollama mode finished"
    echo "🧹 Cleaning up..."
    
    # Clean up background processes
    [ ! -z "$CARTOGRAPHER_PID" ] && kill $CARTOGRAPHER_PID 2>/dev/null
    [ ! -z "$TF_BRIDGE_PID" ] && kill $TF_BRIDGE_PID 2>/dev/null  
    [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null
    [ ! -z "$BRINGUP_PID" ] && kill $BRINGUP_PID 2>/dev/null
    if [ "$SIMULATION_MODE" = true ]; then
        [ ! -z "$GAZEBO_PID" ] && kill $GAZEBO_PID 2>/dev/null
    fi
    
    ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
    echo "✅ Ollama mode cleanup completed"
    exit 0
fi

# Handle Ollama Advanced mode (360° spatial context)
if [ "$OLLAMA_ADVANCED_MODE" = true ]; then
    echo "🦙 OLLAMA ADVANCED MODE"
    echo "======================================"
    echo "Launching Ollama Advanced mode with 360° spatial context"
    
    # Check if Ollama is running
    if ! curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
        echo "❌ ERROR: Ollama is not running on localhost:11434"
        echo "   Please start Ollama first: ollama serve"
        exit 1
    fi
    
    echo "✅ Ollama service detected"
    
    if [ "$SIMULATION_MODE" = true ]; then
        echo "Mode: Gazebo Classic Simulation with 360° Analysis"
        WORLD_NAME="exploration_test_classic"  # Use exploration-specific world
    else  
        echo "Mode: Real Robot Ollama Advanced Navigation"
    fi
    
    echo ""
    echo "This mode will:"
    echo "- Launch Cartographer for real-time SLAM mapping"
    echo "- Provide comprehensive 360-degree spatial awareness"
    echo "- Stop at regular intervals for environmental assessment"
    echo "- Use structured navigation commands (Turn N degrees, move M meters)"
    echo "- Build a map while navigating with advanced LLM guidance"
    echo ""
    
    # Ensure clean state before launching Ollama Advanced mode
    echo "🧹 ENSURING CLEAN STATE FOR OLLAMA ADVANCED MODE"
    echo "======================================"
    echo "Cleaning up any existing ROS2 processes to prevent TF conflicts..."
    ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
    sleep 3
    echo "✅ System cleanup completed"
    echo ""
    
    # Launch Ollama Advanced sequence (identical to basic Ollama sequence)
    echo "🚀 OLLAMA ADVANCED LAUNCH SEQUENCE"
    echo "======================================"
    
    if [ "$SIMULATION_MODE" = true ]; then
        # Step 1: Launch Gazebo Classic simulation with exploration world
        echo "🎮 Step 1: Starting Gazebo Classic simulation with exploration world"
        ros2 launch yahboomcar_nav gazebo_classic_nav_launch.py world_name:=$WORLD_NAME > "$LOGS_DIR/ollama_advanced_gazebo_$TIMESTAMP.log" 2>&1 &
        GAZEBO_PID=$!
        echo "   Waiting for simulation initialization..."
        sleep 10
        
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
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py use_sim_time:=true > "$LOGS_DIR/ollama_advanced_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # Verify odometry topic
        echo "   Verifying odometry is available..."
        timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ Odometry topic verified"
        else
            echo "   ⚠️  Warning: /odom topic not ready, continuing anyway..."
        fi
        
        # Step 3: Launch RViz for visualization
        echo "📊 Step 3: Starting RViz for map visualization"
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=true > "$LOGS_DIR/ollama_advanced_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 5
        
    else
        echo "   ⚠️  Make sure physical robot is powered on and ready"
        echo "   ⚠️  Ensure navigation area is safe for 360° analysis"
        read -p "   Press Enter when robot is ready for Ollama Advanced navigation..."
        
        # Step 1: Launch robot bringup 
        echo "🤖 Step 1: Starting robot sensor and control systems"
        cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py > "$LOGS_DIR/ollama_advanced_bringup_$TIMESTAMP.log" 2>&1 &
        BRINGUP_PID=$!
        echo "   Waiting for sensor initialization and EKF startup..."
        sleep 12
        
        # Verify odometry topic
        echo "   Verifying odometry is available..."
        timeout 10 ros2 topic echo /odom --once > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo "   ✅ Odometry topic verified"
        else
            echo "   ⚠️  Warning: /odom topic not ready, continuing anyway..."
        fi
        
        # Step 2: Launch RViz for visualization
        echo "📊 Step 2: Starting RViz for map visualization"  
        ros2 launch yahboomcar_nav display_launch.py use_sim_time:=false > "$LOGS_DIR/ollama_advanced_rviz_$TIMESTAMP.log" 2>&1 &
        RVIZ_PID=$!
        sleep 3
    fi
    
    # Step 4: Launch TF bridge to connect odom_frame to odom
    echo "🔗 Step 4: Creating TF bridge between odom_frame and odom"
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom_frame odom > "$LOGS_DIR/ollama_advanced_tf_bridge_$TIMESTAMP.log" 2>&1 &
    TF_BRIDGE_PID=$!
    echo "   Bridge created: odom_frame → odom (connects Cartographer to robot)"
    
    # Step 5: Launch Cartographer SLAM for real-time mapping
    echo "🗺️  Step 5: Starting Cartographer SLAM for real-time mapping"
    if [ "$SIMULATION_MODE" = true ]; then
        ros2 launch yahboomcar_nav map_cartographer_launch.py use_sim_time:=true > "$LOGS_DIR/ollama_advanced_cartographer_$TIMESTAMP.log" 2>&1 &
    else
        ros2 launch yahboomcar_nav map_cartographer_launch.py use_sim_time:=false > "$LOGS_DIR/ollama_advanced_cartographer_$TIMESTAMP.log" 2>&1 &
    fi
    CARTOGRAPHER_PID=$!
    echo "   Waiting for SLAM system initialization..."
    sleep 10
    
    # Verify map topic is being published
    echo "   Verifying map topic..."
    if timeout 10 ros2 topic echo /map --once > /dev/null 2>&1; then
        echo "   ✅ Map topic active"
    else
        echo "   ⚠️  Warning: Map topic not detected yet (may appear shortly)"
    fi
    
    # Verify TF tree is complete before starting Ollama Advanced
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
    
    # Step 6: Start Ollama Advanced Spatial Interpreter in foreground
    echo "🦙 Step 6: Starting Ollama Advanced Spatial Interpreter"
    echo "   All output and interaction will happen in this terminal"
    echo ""
    echo "======================================================="
    echo "🦙 Ollama Advanced - 360° Spatial Context Console" 
    echo "======================================================="
    echo "Robot will use 360-degree spatial context for navigation"
    echo "Decision interval: 5.0m"
    echo "Model: llama3.2:latest"
    echo "======================================================="
    echo ""
    echo "🚀 Starting Ollama Advanced Spatial Interpreter..."
    echo "======================================================="
    
    # Run the spatial interpreter in foreground with logging, passing --ollama-advanced-mode flag
    # Use -u flag for unbuffered output to ensure real-time console display
    python3 -u "$WORKSPACE_ROOT/scripts/b4m_spatial_interpreter.py" --ollama-advanced-mode 2>&1 | tee "$LOGS_DIR/ollama_advanced_spatial_$TIMESTAMP.log"
    
    # The spatial interpreter will run here in foreground
    # When it exits (Ctrl+C or completion), we'll clean up
    echo ""
    echo "✅ Ollama Advanced mode finished"
    echo "🧹 Cleaning up..."
    
    # Clean up background processes
    [ ! -z "$CARTOGRAPHER_PID" ] && kill $CARTOGRAPHER_PID 2>/dev/null
    [ ! -z "$TF_BRIDGE_PID" ] && kill $TF_BRIDGE_PID 2>/dev/null  
    [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null
    [ ! -z "$BRINGUP_PID" ] && kill $BRINGUP_PID 2>/dev/null
    if [ "$SIMULATION_MODE" = true ]; then
        [ ! -z "$GAZEBO_PID" ] && kill $GAZEBO_PID 2>/dev/null
    fi
    
    ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
    echo "✅ Ollama Advanced mode cleanup completed"
    exit 0
fi

# Handle Ollama Navigation mode (LLM-guided Nav2 goal selection)
if [ "$OLLAMA_NAV_MODE" = true ]; then
    echo "🧭🦙 OLLAMA NAVIGATION MODE"
    echo "======================================"
    echo "Intelligent LLM-guided navigation with Navigation 2 stack"
    
    # Check if Ollama is running
    if ! curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
        echo "❌ ERROR: Ollama is not running on localhost:11434"
        echo "   Please start Ollama first: ollama serve"
        exit 1
    fi
    
    echo "✅ Ollama service detected"
    
    if [ "$SIMULATION_MODE" = true ]; then
        echo "Mode: Gazebo Classic Simulation with LLM Navigation"
        WORLD_NAME="exploration_test_classic"  # Use exploration-specific world
    else  
        echo "Mode: Real Robot Ollama Navigation with SLAM"
    fi
    
    echo ""
    echo "This mode will:"
    echo "- Launch Navigation 2 stack with Cartographer SLAM"
    echo "- Use Ollama LLM for intelligent 2D pose selection"
    echo "- Combine 360° spatial awareness with goal-based pathfinding"
    echo "- Balance exploration and navigation objectives autonomously"
    echo "- Display selected goals and reasoning in RViz"
    echo ""
    
    # Ensure clean state before launching Ollama Navigation mode
    echo "🧹 ENSURING CLEAN STATE FOR OLLAMA NAVIGATION"
    echo "======================================"
    echo "Cleaning up any existing ROS2 processes..."
    ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
    sleep 3
    echo "✅ System cleanup completed"
    echo ""
    
    # Start the standard robot components first
    echo "🚀 OLLAMA NAVIGATION LAUNCH SEQUENCE"
    echo "======================================"
    
    # Step 1: Start Micro-ROS Agent (if not skipped)
    if [ "$SKIP_AGENT" = false ]; then
        echo "Step 1: Starting Micro-ROS Agent..."
        docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090 > "$LOGS_DIR/microros_agent_$TIMESTAMP.log" 2>&1 &
        MICROROS_PID=$!
        sleep 3
        echo "✅ Micro-ROS Agent started (PID: $MICROROS_PID)"
    else
        echo "⏭️  Step 1: Micro-ROS Agent skipped (--skip-agent)"
    fi
    
    # Step 2: Robot power confirmation (simulation mode skips manual step)
    if [ "$SIMULATION_MODE" = false ]; then
        echo ""
        echo "Step 2: Please power on the physical robot and wait for connection..."
        echo "Press Enter when robot is connected and ready..."
        read
        echo "✅ Robot connection confirmed"
    else
        echo "Step 2: Simulation mode - skipping manual robot power step"
    fi
    
    # Step 3: Start robot bringup
    echo "Step 3: Starting robot sensor integration..."
    cd "$WORKSPACE_ROOT" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py > "$LOGS_DIR/bringup_$TIMESTAMP.log" 2>&1 &
    BRINGUP_PID=$!
    sleep 5
    echo "✅ Robot bringup started (PID: $BRINGUP_PID)"
    
    # Step 4: Start RViz
    echo "Step 4: Starting RViz for visualization..."
    cd "$WORKSPACE_ROOT" && . install/setup.bash && ros2 launch yahboomcar_nav display_launch.py > "$LOGS_DIR/rviz_$TIMESTAMP.log" 2>&1 &
    RVIZ_PID=$!
    sleep 3
    echo "✅ RViz started (PID: $RVIZ_PID)"
    
    # Step 5: Launch Navigation 2 with SLAM
    echo "Step 5: Launching Navigation 2 with Cartographer SLAM..."
    if [ "$SIMULATION_MODE" = true ]; then
        # For simulation, use Gazebo with Navigation 2
        cd "$WORKSPACE_ROOT" && . install/setup.bash && ros2 launch yahboomcar_nav gazebo_classic_nav_launch.py world:="$WORKSPACE_ROOT/yahboomcar_nav/worlds/navigation_test_classic.world" > "$LOGS_DIR/nav2_gazebo_$TIMESTAMP.log" 2>&1 &
        NAV2_PID=$!
    else
        # For real robot, use Navigation 2 with SLAM from scratch
        cd "$WORKSPACE_ROOT" && . install/setup.bash && ros2 launch yahboomcar_nav nav2_slam_launch.py > "$LOGS_DIR/nav2_slam_$TIMESTAMP.log" 2>&1 &
        NAV2_PID=$!
    fi
    sleep 8
    echo "✅ Navigation 2 with SLAM started (PID: $NAV2_PID)"
    
    # Step 6: Initial pose setup (for real robot)
    if [ "$SIMULATION_MODE" = false ]; then
        echo "Step 6: Setting initial robot pose..."
        cd "$WORKSPACE_ROOT" && . install/setup.bash && python3 "$WORKSPACE_ROOT/scripts/set_initial_pose.py" > "$LOGS_DIR/initial_pose_$TIMESTAMP.log" 2>&1
        sleep 2
        echo "✅ Initial pose set"
    else
        echo "Step 6: Simulation mode - robot pose automatically initialized"
    fi
    
    # Final step: Start Ollama Navigation Controller in foreground
    echo ""
    echo "======================================"
    echo "🦙 Starting Ollama Navigation Controller"
    echo "======================================"
    echo "The controller will now take over navigation using Ollama LLM"
    echo "for intelligent goal selection and path planning."
    echo ""
    echo "Press Ctrl+C to stop the system"
    echo "======================================"
    echo ""
    
    # Run the ollama navigation controller in foreground with logging
    python3 -u "$WORKSPACE_ROOT/scripts/ollama_nav_controller.py" 2>&1 | tee "$LOGS_DIR/ollama_nav_$TIMESTAMP.log"
    
    # When controller exits (Ctrl+C or completion), clean up
    echo ""
    echo "✅ Ollama Navigation mode finished"
    echo "🧹 Cleaning up..."
    
    # Clean up background processes
    [ ! -z "$NAV2_PID" ] && kill $NAV2_PID 2>/dev/null
    [ ! -z "$RVIZ_PID" ] && kill $RVIZ_PID 2>/dev/null
    [ ! -z "$BRINGUP_PID" ] && kill $BRINGUP_PID 2>/dev/null
    [ ! -z "$MICROROS_PID" ] && kill $MICROROS_PID 2>/dev/null
    
    # Use shutdown script for thorough cleanup (keep agent for real robot)
    if [ "$SIMULATION_MODE" = true ]; then
        ./b4m_shutdown.sh > /dev/null 2>&1
    else
        ./b4m_shutdown.sh --keep-agent > /dev/null 2>&1
    fi
    
    echo "✅ Ollama Navigation mode cleanup completed"
    exit 0
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


# Function to ask for user confirmation
confirm() {
    echo ""
    read -p "Press Enter to continue to the next step or Ctrl+C to exit..."
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
        *)
            echo "ERROR: Unknown step number for validation: $step_num"
            return 1
            ;;
    esac
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
    echo "🎮 SIMULATION MODE: Skipping physical robot power on (using simulation)"
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

# Step 3: Launch the Car's Underlying Data Processing
launch_in_terminal "Starting the car's underlying data processing for sensor integration" \
    "cd \"$WORKSPACE_ROOT\" && . source_workspaces.sh && ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py" \
    "3"

# Step 4: Start RViz for Visualization
launch_in_terminal "Starting RViz for visualization of robot state and environment" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav display_launch.py" \
    "4"

# Step 5: Launch the Navigation System
launch_in_terminal "Launching the navigation system with pre-built map" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && ros2 launch yahboomcar_nav waypoint_navigation_launch.py maps:=\"$WORKSPACE_ROOT/yahboomcar_nav/maps/yahboom_map.yaml\"" \
    "5"

# Step 6: Automatic Robot Positioning
launch_in_terminal "Setting automatic pose estimate at map center for testing" \
    "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && python3 \"$WORKSPACE_ROOT/scripts/set_initial_pose.py\"" \
    "6"

# Step 7: Start the B4M Waypoint Navigation Node with MQTT Parameters (if enabled)
if [ "$B4M_HA" = true ]; then
    launch_in_terminal "Starting the B4M Waypoint Navigation Node with MQTT integration" \
        "cd \"$WORKSPACE_ROOT\" && . install/setup.bash && python3 \"$WORKSPACE_ROOT/b4m_waypoint_nav/b4m_waypoint_nav/b4m_waypoint_nav.py\" --ros-args -p mqtt_broker:=192.168.68.111 -p mqtt_port:=1883 -p mqtt_username:=robot -p mqtt_password:=robot123" \
        "7"
else
    echo "Skipping Step 7: MQTT/Home Assistant integration (use --b4m-HA to enable)"
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
