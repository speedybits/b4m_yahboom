#!/bin/bash

# Script to find, copy, and fix all references to yahboomcar_ws paths
# Updated: $(date)

# Base directories
SOURCE_BASE="/home/yahboom/unused_yahboomcar_ws"
SOURCE_BASE2="/home/yahboom/unused_ros2_ws"
TARGET_BASE="/home/yahboom/b4m_yahboom"

# Log file
LOG_FILE="$TARGET_BASE/path_fix_log.txt"
echo "Path fix operation started at $(date)" > "$LOG_FILE"

# Function to ensure directory exists
ensure_dir_exists() {
    local dir="$1"
    if [ ! -d "$dir" ]; then
        echo "Creating directory: $dir" | tee -a "$LOG_FILE"
        mkdir -p "$dir"
    fi
}

# Function to copy a file if it exists in source locations
copy_file_if_exists() {
    local src_path="$1"
    local target_path="$2"
    
    # Remove /home/yahboom/yahboomcar_ws/src/ prefix to get relative path
    local rel_path="${src_path#/home/yahboom/yahboomcar_ws/src/}"
    
    # Try different source locations
    local source1="$SOURCE_BASE/src/$rel_path"
    local source2="$SOURCE_BASE2/src/$rel_path"
    
    # Create target directory
    local target_dir=$(dirname "$target_path")
    ensure_dir_exists "$target_dir"
    
    # Check if file already exists in target
    if [ -f "$target_path" ]; then
        echo "File already exists in target: $target_path" | tee -a "$LOG_FILE"
        return 0
    fi
    
    # Try to copy from first source
    if [ -f "$source1" ]; then
        echo "Copying from $source1 to $target_path" | tee -a "$LOG_FILE"
        cp "$source1" "$target_path"
        return 0
    fi
    
    # Try to copy from second source
    if [ -f "$source2" ]; then
        echo "Copying from $source2 to $target_path" | tee -a "$LOG_FILE"
        cp "$source2" "$target_path"
        return 0
    fi
    
    # File not found in any source
    echo "WARNING: Could not find source file for: $src_path" | tee -a "$LOG_FILE"
    return 1
}

# Part 1: Fix documentation files (original functionality)
DOC_DIR="$TARGET_BASE/doc_txt"
modified_count=0

echo "Starting path replacement in documentation files..." | tee -a "$LOG_FILE"

# Process each .txt file in the doc_txt directory
for file in "$DOC_DIR"/*.txt; do
    filename=$(basename "$file")
    
    # Check if file contains yahboomcar_ws
    if grep -q "yahboomcar_ws" "$file"; then
        echo "Processing: $filename" | tee -a "$LOG_FILE"
        
        # Replace various forms of yahboomcar_ws paths
        sed -i 's|/home/yahboom/yahboomcar_ws|/home/yahboom/b4m_yahboom|g' "$file"
        sed -i 's|~/yahboomcar_ws|~/b4m_yahboom|g' "$file"
        sed -i 's|/root/yahboomcar_ros2_ws/yahboomcar_ws|/home/yahboom/b4m_yahboom|g' "$file"
        sed -i 's|~/yahboomcar_ros2_ws/yahboomcar_ws|~/b4m_yahboom|g' "$file"
        
        modified_count=$((modified_count + 1))
    fi
done

echo "Documentation path replacement complete. Modified $modified_count files." | tee -a "$LOG_FILE"

# Part 2: Find all files with references to yahboomcar_ws
echo "Finding files with references to yahboomcar_ws..." | tee -a "$LOG_FILE"
FILES_WITH_REFS=$(grep -r --include="*.py" --include="*.launch.py" --include="*.rviz" --include="*.yaml" --include="*.xml" --include="*.urdf" "/home/yahboom/yahboomcar_ws" "$TARGET_BASE" | cut -d: -f1 | sort | uniq)

# Process each file with references
for file in $FILES_WITH_REFS; do
    echo "Processing file: $file" | tee -a "$LOG_FILE"
    
    # Extract all yahboomcar_ws paths from the file
    PATHS=$(grep -o '/home/yahboom/yahboomcar_ws/src/[^"'\''` ]*' "$file" | sort | uniq)
    
    for path in $PATHS; do
        # Determine target path
        target_path="$TARGET_BASE/${path#/home/yahboom/yahboomcar_ws/src/}"
        
        # Copy the file if it exists
        copy_file_if_exists "$path" "$target_path"
    done
    
    # Replace paths in the file
    echo "Updating paths in: $file" | tee -a "$LOG_FILE"
    sed -i 's|/home/yahboom/yahboomcar_ws/src/|/home/yahboom/b4m_yahboom/|g' "$file"
    sed -i 's|/home/yahboom/yahboomcar_ws|/home/yahboom/b4m_yahboom|g' "$file"
done

# Part 3: Process any yahboomcar_ros2_ws references
echo "Finding files with references to yahboomcar_ros2_ws..." | tee -a "$LOG_FILE"
FILES_WITH_REFS2=$(grep -r --include="*.py" --include="*.launch.py" --include="*.rviz" --include="*.yaml" --include="*.xml" --include="*.urdf" "/home/yahboom/yahboomcar_ros2_ws" "$TARGET_BASE" | cut -d: -f1 | sort | uniq)

# Process each file with references
for file in $FILES_WITH_REFS2; do
    echo "Processing file: $file" | tee -a "$LOG_FILE"
    
    # Extract all yahboomcar_ros2_ws paths from the file
    PATHS=$(grep -o '/home/yahboom/yahboomcar_ros2_ws/[^"'\''` ]*' "$file" | sort | uniq)
    
    for path in $PATHS; do
        # Determine target path
        rel_path="${path#/home/yahboom/yahboomcar_ros2_ws/}"
        target_path="$TARGET_BASE/$rel_path"
        
        # Copy the file if it exists
        copy_file_if_exists "$path" "$target_path"
    done
    
    # Replace paths in the file
    echo "Updating paths in: $file" | tee -a "$LOG_FILE"
    sed -i 's|/home/yahboom/yahboomcar_ros2_ws/|/home/yahboom/b4m_yahboom/|g' "$file"
done

echo "Path fix operation completed at $(date)" | tee -a "$LOG_FILE"
echo "See $LOG_FILE for details"
