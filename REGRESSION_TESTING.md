# Automated Regression Testing with Image Comparison

The B4M Yahboom project includes comprehensive automated regression testing that validates both functionality and visual output quality.

## Overview

The regression test system performs a controlled 360° rotation while capturing RViz screenshots, then compares them against reference images to ensure consistent laser scan visualization and SLAM mapping functionality.

## Running Regression Tests

### Basic Command
```bash
./b4m_HA_launch.sh --simulation --regression
```

This command will:
1. Launch Gazebo Classic simulation with exploration world
2. Start RViz visualization 
3. Initialize Cartographer SLAM system
4. Perform controlled 360° robot rotation
5. Capture screenshots at 3 key moments
6. Compare screenshots with reference images
7. Report pass/fail based on 90% similarity threshold

## Test Process

### Phase 1: System Launch
- **Gazebo**: Launches simulation environment
- **RViz**: Starts visualization with laser scan and map displays  
- **Cartographer**: Initializes real-time SLAM mapping
- **Identical to exploration mode** for consistent results

### Phase 2: Controlled Rotation Test
- **Initial Screenshot**: Captures RViz at test start with laser scans visible
- **360° Rotation**: Robot rotates in place at 0.3 rad/s (same as exploration)
- **Mid-Rotation Screenshot**: Captures state at 180° showing map progress
- **Final Screenshot**: Captures completed state with full SLAM map
- **Movement mimics autonomous exploration** for identical visualization

### Phase 3: Image Comparison
- **Multi-Method Analysis**: Uses 4 different similarity algorithms
- **90% Threshold**: Test passes if all screenshots ≥90% similar to references
- **Detailed Logging**: Reports individual similarity scores for debugging

## Image Comparison Methods

The system uses weighted combination of multiple similarity measures:

### 1. Histogram Similarity (30% weight)
- Compares color distribution in HSV space
- Detects changes in laser scan colors and map rendering

### 2. Structural Similarity - SSIM (40% weight) 
- Analyzes structural patterns and luminance
- Most important for detecting layout changes

### 3. Feature Matching (20% weight)
- Uses ORB feature detection and matching
- Robust against small timing differences

### 4. Template Matching (10% weight)
- Overall pattern correlation
- Provides baseline similarity measure

## Reference Screenshots

Reference images are stored in `regression/reference_screenshots/`:

- `reference_initial.png` - Perfect laser scan visualization at start
- `reference_mid.png` - Mid-rotation with active map building  
- `reference_final.png` - Complete 360° rotation with full map

These represent the expected "gold standard" visualization quality.

## Installation Requirements

### Ubuntu/Linux
```bash
sudo apt install python3-opencv python3-skimage
```

### macOS (in UTM VM)
```bash
sudo apt install python3-opencv python3-skimage
```

## Expected Results

A successful regression test should show similarity scores like:

```
INITIAL: ✅ PASS (99.0%)
MID-ROTATION: ✅ PASS (97.9%) 
FINAL: ✅ PASS (98.3%)

🎯 FINAL RESULT: ✅ ALL COMPARISONS PASSED
```

## Troubleshooting

### Test Fails with Low Similarity
- Check if laser scans are visible in RViz during test
- Verify Cartographer SLAM is working correctly
- Look for timing issues in screenshot capture
- Check logs in `logs/regression_rotation_*.log`

### Missing Dependencies
```bash
# Install required packages
sudo apt update
sudo apt install python3-opencv python3-skimage
```

### Reference Screenshots Outdated
If system improvements require new reference images:
1. Run successful test that produces good visualization
2. Copy new screenshots from `regression/screenshots/` to `regression/reference_screenshots/`
3. Rename to `reference_initial.png`, `reference_mid.png`, `reference_final.png`
4. Commit updated references

## Integration with Development Workflow

### Before Committing Code
Always run regression test before committing non-documentation changes:
```bash
./b4m_HA_launch.sh --simulation --regression
```

### Continuous Integration
The regression test provides automated validation that:
- Laser scan visualization remains consistent
- SLAM mapping functionality works correctly  
- RViz configuration produces expected output
- Screenshot capture system functions properly

### Code Changes That Trigger Failures
Changes to these areas may cause regression failures:
- RViz configuration (`yahboomcar_nav/rviz/view.rviz`)
- Cartographer launch files (`cartographer_launch.py`)
- Laser scan display settings
- SLAM mapping parameters
- Screenshot timing or capture methods

## File Locations

### Scripts
- `scripts/regression_rotation.py` - Main rotation test script
- `scripts/compare_screenshots.py` - Image comparison engine
- `scripts/capture_rviz.sh` - Screenshot capture utility

### Configuration  
- `regression/reference_screenshots/` - Reference images (committed)
- `regression/screenshots/` - Test outputs (gitignored)
- `b4m_HA_launch.sh` - Main launch script with regression mode

### Logs
- `logs/regression_rotation_*.log` - Detailed test execution logs
- `regression/screenshots/comparison_results.json` - Detailed similarity metrics

This automated testing ensures the robot's visualization system maintains high quality and consistency across development changes.