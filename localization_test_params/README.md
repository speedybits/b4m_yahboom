# Localization Test Parameter Sets

This directory contains AMCL parameter configurations for systematic localization testing as documented in `b4m_localization_tune.md`.

## Parameter Set Files

### baseline_amcl_params.yaml
- **Purpose**: Current production configuration used as reference
- **Characteristics**: Standard settings from robot1_amcl_params.yaml
- **Use case**: Baseline comparison for all testing

### indoor_optimized_amcl_params.yaml
- **Purpose**: Optimized for indoor environments with smooth floors
- **Key changes**: 
  - Reduced motion noise (alpha1-5: 0.1-0.15)
  - More aggressive beam processing (80 max_beams, beam skipping enabled)
  - Higher sensor model confidence (z_hit: 0.7, z_rand: 0.2)
  - More responsive updates (update_min_a: 0.15, update_min_d: 0.2)
- **Expected performance**: Better accuracy in clean indoor environments

### high_precision_amcl_params.yaml
- **Purpose**: Maximum localization accuracy at computational cost
- **Key changes**:
  - Very low motion noise (alpha1-5: 0.05-0.08)
  - Maximum laser utilization (120 max_beams, minimal beam skipping)
  - High particle count (3000 max, 1000 min particles)
  - Very tight error tolerance (pf_err: 0.02)
  - Maximum sensor confidence (z_hit: 0.8)
- **Expected performance**: Highest accuracy, may be slower to converge

### balanced_amcl_params.yaml
- **Purpose**: Good balance between accuracy, responsiveness, and efficiency
- **Key changes**:
  - Moderate motion noise (alpha1-5: 0.15-0.18)
  - Balanced beam processing (70 max_beams)
  - Standard particle count (2000 max, 400 min)
  - Moderate error tolerance (pf_err: 0.04)
  - Balanced sensor model (z_hit: 0.6, z_rand: 0.3)
- **Expected performance**: Good all-around performance for general use

### fast_convergence_amcl_params.yaml
- **Purpose**: Rapid particle convergence and quick recovery
- **Key changes**:
  - Higher motion noise tolerance (alpha1-5: 0.25-0.3)
  - Aggressive beam skipping (50 max_beams)
  - Reduced particle count (1200 max, 200 min)
  - Higher error tolerance (pf_err: 0.08)
  - Aggressive recovery parameters (recovery_alpha_fast: 0.2)
- **Expected performance**: Fastest convergence, may sacrifice some accuracy

## Usage with Testing Framework

These parameter files are used with the navigation testing framework in `b4m_launch.sh`:

```bash
# Test with specific parameter set
./b4m_launch.sh --navigation-performance-test --parameter-set indoor_optimized

# Available parameter sets:
# - baseline (default)
# - indoor_optimized
# - high_precision
# - balanced
# - fast_convergence
```

## Testing Methodology

Each parameter set is evaluated using:
1. **Navigation Success Rate**: Percentage of successful waypoint navigation
2. **Navigation Time**: Average time to reach each waypoint
3. **Position Accuracy**: Distance error at each waypoint
4. **Recovery Performance**: Time to recover from localization failures

See `b4m_localization_tune.md` for complete testing procedures and metrics.

## Parameter Tuning Guidelines

### Motion Model Parameters (alpha1-alpha5)
- **Lower values**: More confident in odometry, tighter particle distribution
- **Higher values**: Less confident in odometry, wider particle spread
- **Typical range**: 0.05-0.3 for indoor robots

### Particle Filter Parameters
- **max_particles**: More particles = higher accuracy, slower computation
- **min_particles**: Minimum for stability, affects convergence speed
- **pf_err**: Lower = tighter convergence, higher = more exploration

### Sensor Model Parameters
- **z_hit**: Weight for accurate laser measurements (0.5-0.8)
- **z_rand**: Weight for random/unexpected readings (0.15-0.35)
- **z_max/z_short**: Usually kept low (0.02-0.1)

### Update Parameters
- **update_min_d**: Minimum translation for filter update (0.1-0.3m)
- **update_min_a**: Minimum rotation for filter update (0.1-0.3 rad)
- Lower values = more frequent updates, higher computational load