#!/bin/bash

# Script to replace yahboomcar_ws paths with b4m_yahboom paths in documentation files
# Created: $(date)

# Directory containing the documentation files
DOC_DIR="/home/yahboom/b4m_yahboom/doc_txt"

# Counter for modified files
modified_count=0

echo "Starting path replacement in documentation files..."

# Process each .txt file in the doc_txt directory
for file in "$DOC_DIR"/*.txt; do
    filename=$(basename "$file")
    
    # Check if file contains yahboomcar_ws
    if grep -q "yahboomcar_ws" "$file"; then
        echo "Processing: $filename"
        
        # Replace various forms of yahboomcar_ws paths
        sed -i 's|/home/yahboom/yahboomcar_ws|/home/yahboom/b4m_yahboom|g' "$file"
        sed -i 's|~/yahboomcar_ws|~/b4m_yahboom|g' "$file"
        sed -i 's|/root/yahboomcar_ros2_ws/yahboomcar_ws|/home/yahboom/b4m_yahboom|g' "$file"
        sed -i 's|~/yahboomcar_ros2_ws/yahboomcar_ws|~/b4m_yahboom|g' "$file"
        
        modified_count=$((modified_count + 1))
    fi
done

echo "Path replacement complete."
echo "Modified $modified_count files."
