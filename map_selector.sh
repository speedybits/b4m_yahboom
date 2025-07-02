#!/bin/bash

# Map Selector for Yahboom Navigation
# This script provides a text-based menu to select which map to use for navigation
# It copies the selected map files to yahboom_map.pgm and yahboom_map.yaml
# Then runs colcon build to update the install directory

# Define colors for better readability
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Define the maps directory
MAPS_DIR="/home/yahboom/b4m_yahboom/yahboomcar_nav/maps"

# Check if maps directory exists
if [ ! -d "$MAPS_DIR" ]; then
    echo -e "${YELLOW}Warning: Maps directory not found at $MAPS_DIR${NC}"
    echo -e "${YELLOW}Creating directory...${NC}"
    mkdir -p "$MAPS_DIR"
    if [ $? -ne 0 ]; then
        echo -e "${RED}Error: Failed to create maps directory${NC}"
        exit 1
    fi
fi

# Find available maps
find_maps() {
    # Check for map files (both .pgm and .yaml must exist)
    for pgm_file in "$MAPS_DIR"/*.pgm; do
        # Skip if no .pgm files exist (glob doesn't match)
        if [[ ! -f "$pgm_file" ]]; then
            continue
        fi
        
        # Get base name without extension
        local base_name=$(basename "$pgm_file" .pgm)
        
        # Check if corresponding .yaml file exists
        if [[ -f "$MAPS_DIR/$base_name.yaml" ]]; then
            echo "$base_name"
        fi
    done
}

# Display the menu of available maps
display_menu() {
    local maps=("$@")
    local map_count=${#maps[@]}
    
    echo -e "\n===== Yahboom Robot Map Selector =====\n"
    
    if [[ $map_count -eq 0 ]]; then
        echo -e "${RED}No valid maps found in $MAPS_DIR${NC}"
        echo -e "${YELLOW}A valid map requires both .pgm and .yaml files.${NC}"
        echo -e "${YELLOW}Please create maps using gmapping or copy existing maps to $MAPS_DIR${NC}\n"
        echo -e "${GREEN}1${NC}) Exit"
        return
    fi
    
    echo -e "${YELLOW}Available maps:${NC}"
    for ((i=0; i<$map_count; i++)); do
        echo -e "${GREEN}$((i+1))${NC}) ${maps[i]}"
    done
    
    # Add exit option
    echo -e "${GREEN}$((map_count+1))${NC}) Exit without changes"
}

# Process user selection
process_selection() {
    local maps=("$@")
    local map_count=${#maps[@]}
    local selection
    
    # Read user selection
    read -p "\nEnter the number of the map you want to use: " selection
    
    # Validate input is a number
    if ! [[ "$selection" =~ ^[0-9]+$ ]]; then
        echo -e "\n${RED}Invalid input. Please enter a number.${NC}"
        return 1
    fi
    
    # Handle no maps case
    if [[ $map_count -eq 0 ]]; then
        if [[ "$selection" -eq 1 ]]; then
            echo -e "\n${YELLOW}Exiting.${NC}"
            exit 0
        else
            echo -e "\n${RED}Invalid selection. Please enter 1 to exit.${NC}"
            return 1
        fi
    fi
    
    # Handle exit option
    if [[ "$selection" -eq $((map_count+1)) ]]; then
        echo -e "\n${YELLOW}Exiting without changes.${NC}"
        exit 0
    fi
    
    # Validate selection is in range
    if [[ "$selection" -lt 1 || "$selection" -gt $((map_count+1)) ]]; then
        echo -e "\n${RED}Invalid selection. Please choose a number between 1 and $((map_count+1)).${NC}"
        return 1
    fi
    
    # Get selected map name
    local selected_map=${maps[$((selection-1))]}
    echo -e "\n${GREEN}Selected map: $selected_map${NC}"
    
    # Only copy files if not selecting yahboom_map
    if [[ "$selected_map" != "yahboom_map" ]]; then
        echo "Copying map files to source directory..."
        cp "$MAPS_DIR/$selected_map.pgm" "$MAPS_DIR/yahboom_map.pgm"
        cp "$MAPS_DIR/$selected_map.yaml" "$MAPS_DIR/yahboom_map.yaml"
        
        if [[ $? -ne 0 ]]; then
            echo -e "${RED}Error copying map files${NC}"
            return 1
        fi
    else
        echo -e "${YELLOW}Using existing yahboom_map files (no copying needed)${NC}"
    fi
    
    # Run colcon build
    echo -e "\nRunning colcon build to update installed maps..."
    cd /home/yahboom/b4m_yahboom
    . /opt/ros/humble/setup.bash
    colcon build --packages-select yahboomcar_nav
    
    if [[ $? -ne 0 ]]; then
        echo -e "${RED}Error running colcon build${NC}"
        return 1
    fi
    
    echo -e "\n${GREEN}Map selection complete! The robot will now use the '$selected_map' map for navigation.${NC}"
    echo -e "\nTo use this map, launch the navigation system with:"
    echo -e "${BLUE}ros2 launch yahboomcar_nav waypoint_navigation_launch.py${NC}"
    
    return 0
}

# Main function
main() {
    # Find available maps
    mapfile -t available_maps < <(find_maps)
    
    # Display menu
    display_menu "${available_maps[@]}"
    
    # Process selection
    while true; do
        process_selection "${available_maps[@]}"
        if [[ $? -eq 0 ]]; then
            break
        fi
        echo -e "\nPlease try again."
    done
}

# Run the main function
main