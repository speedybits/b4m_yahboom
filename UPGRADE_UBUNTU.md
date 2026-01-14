# Ubuntu Upgrade Assessment: 22.04 to 24.04 for B4M Robot Project

## Executive Summary

This document analyzes the requirements for upgrading the B4M Yahboom robot project from **Ubuntu 22.04 (Jammy)** with **ROS2 Humble** to **Ubuntu 24.04 (Noble)** with **ROS2 Jazzy**. The upgrade involves significant changes across multiple components and requires careful planning.

**Recommendation**: This is a **major undertaking** requiring substantial code changes, particularly for Gazebo migration. Consider whether the benefits outweigh the effort for your use case.

---

## Current System Configuration

| Component | Current Version |
|-----------|-----------------|
| Ubuntu | 22.04 LTS (Jammy Jellyfish) |
| ROS2 | Humble Hawksbill |
| Python | 3.10 |
| Gazebo | Classic 11.10.2 |
| Navigation | Nav2 (Humble) |
| SLAM | Cartographer + Gmapping |
| Micro-ROS | Humble Docker image |

---

## Target System Configuration

| Component | Target Version |
|-----------|----------------|
| Ubuntu | 24.04 LTS (Noble Numbat) |
| ROS2 | Jazzy Jalisco (LTS until May 2029) |
| Python | 3.12 |
| Gazebo | Harmonic (LTS until September 2028) |
| Navigation | Nav2 (Jazzy) |
| SLAM | slam_toolbox (recommended) or Cartographer |
| Micro-ROS | Jazzy branch (in development) |

---

## b4m_launch.sh Modes Analysis

The launch script supports many modes. Here's how each would be affected by the upgrade:

### Core Infrastructure Modes

| Mode | Description | Upgrade Impact |
|------|-------------|----------------|
| `--skip-agent` | Skip Micro-ROS agent | **Medium** - Needs Jazzy Docker image |
| `--only-agent` | Launch only Micro-ROS agent | **Medium** - Docker image update required |
| `--debug` | Enable verbose logging | **Low** - No changes expected |
| `--setup-wifi` | WiFi configuration wizard | **Low** - Python serial code should work |

### Simulation Modes

| Mode | Description | Upgrade Impact |
|------|-------------|----------------|
| `--simulation` | Gazebo Classic simulation | **HIGH** - Complete migration to Gazebo Harmonic required |

### Navigation & SLAM Modes

| Mode | Description | Upgrade Impact |
|------|-------------|----------------|
| `--explore` | Autonomous exploration with Cartographer | **High** - SLAM and launch file changes |
| `--nav` | Navigation 2 with SLAM | **Medium** - Nav2 parameter updates |
| `--regression` | Regression test suite | **Medium** - Test updates for new APIs |

### LLM Integration Modes

| Mode | Description | Upgrade Impact |
|------|-------------|----------------|
| `--ollama` | Basic Ollama control | **Low** - Python 3.12 compatibility check |
| `--ollama-advanced` | 360° spatial context | **Low** - Python 3.12 compatibility check |
| `--ollama-nav` | LLM-guided Nav2 goals | **Medium** - Nav2 API changes |
| `--ollama-nav-explore` | Autonomous LLM exploration | **Medium** - Nav2 + SLAM changes |

### Home Assistant & API Modes

| Mode | Description | Upgrade Impact |
|------|-------------|----------------|
| `--b4m-api` | B4M API integration | **Low** - Python 3.12 compatibility |
| `--b4m-HA` | Home Assistant MQTT | **Low** - MQTT libraries stable |
| `--b4m-ping` | API testing tool | **Low** - Minimal changes |

### Testing Modes

| Mode | Description | Upgrade Impact |
|------|-------------|----------------|
| `--navigation-performance-test` | Navigation circuit testing | **Medium** - Nav2 changes |

---

## Major Component Changes Required

### 1. Gazebo Classic to Gazebo Harmonic Migration (HIGH EFFORT)

**This is the most significant change.** Gazebo Classic reached end-of-life in January 2025 and is not supported in ROS2 Jazzy.

#### Required Changes:

- **Package dependencies**: Replace `gazebo_ros_pkgs` with `ros_gz` packages
- **Launch files**: Update all Gazebo launch files
  - Change `gazebo_ros` to `ros_gz_sim`
  - Replace `spawn_entity.py` with `create` node
  - Replace `-entity` argument with `-name`
- **World files**: Convert from SDF (Gazebo Classic) format to updated SDF format
- **URDF/Model files**: Update sensor and plugin references
- **Topic bridging**: Configure `ros_gz_bridge` for ROS2 topic integration

#### Files Requiring Updates:
```
yahboomcar_nav/launch/gazebo_classic_nav_launch.py
yahboomcar_nav/worlds/navigation_test_classic.world
yahboomcar_description/urdf/yahboomcar_robot_classic_nav.urdf
```

#### Resources:
- [Official Migration Guide](https://gazebosim.org/docs/latest/migrating_gazebo_classic_ros2_packages/)
- [ROS2 Jazzy Gazebo Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [Gazebo Harmonic with ROS Installation](https://gazebosim.org/docs/latest/ros_installation/)

### 2. SLAM Package Changes (MEDIUM EFFORT)

#### Gmapping
- **Status**: No official ROS2 Jazzy support
- **Recommendation**: Migrate to **slam_toolbox** (officially supported and recommended)
- **Alternative**: Community-maintained gmapping port exists but is unofficial

#### Cartographer
- **Status**: Available for Jazzy (version 2.0.9004)
- **Changes**: Package name and API updates may be required

#### slam_toolbox (Recommended)
- **Status**: Fully supported, actively maintained
- **Benefits**:
  - Better performance in large environments
  - Lifelong mapping support
  - Synchronous/asynchronous modes
  - Direct replacement for gmapping

#### Resources:
- [slam_toolbox Jazzy Documentation](https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/)
- [Nav2 SLAM Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)

### 3. Navigation 2 Changes (MEDIUM EFFORT)

#### Key Changes in Nav2 for Jazzy:
- MPPI is now the default local planner (replacing DWB)
- Route Server added for predefined navigation graphs
- Gazebo Modern (Harmonic) integration
- Parameter file format updates may be required

#### Files Requiring Review:
```
yahboomcar_nav/params/*.yaml
yahboomcar_nav/launch/*nav*.py
```

#### Resources:
- [Nav2 Jazzy Migration Guide](https://docs.nav2.org/migration/Jazzy.html)
- [Nav2 Getting Started](https://docs.nav2.org/getting_started/index.html)

### 4. Micro-ROS Updates (MEDIUM EFFORT)

#### Current Setup:
```bash
docker run microros/micro-ros-agent:humble udp4 --port 8090
```

#### Required Changes:
- Update Docker image from `humble` to `jazzy`
- Verify ESP32 firmware compatibility with Jazzy micro-ROS libraries
- Test WiFi/UDP transport functionality

#### Status:
- Jazzy branch exists in `micro_ros_setup` repository
- ESP32 support is in development
- May require building from source if Docker image not available

#### Resources:
- [micro_ros_setup Jazzy Branch](https://github.com/micro-ROS/micro_ros_setup/blob/jazzy/README.md)
- [micro-ROS ESP32 Port](https://micro.ros.org/blog/2020/08/27/esp32/)

---

### Can You Keep Micro-ROS on Humble While Upgrading Everything Else?

**Short Answer: No, this is not recommended and will likely cause serious problems.**

#### The Problem: Cross-Distribution Communication

ROS2 distributions are **not guaranteed to communicate with each other**. According to official ROS2 documentation:

> "Nodes are not guaranteed to be able to communicate across distributions. For example, a node built & running against Humble is not guaranteed to be able to communicate correctly with a node built & running against Iron [or Jazzy]. It may or may not work, but it is not supported and should not be relied upon."

#### Known Issues with Humble ↔ Jazzy Communication

1. **Memory Exhaustion**: Running both Humble and Jazzy nodes on the same network has been reported to cause machines running Humble nodes to **run out of memory** due to DDS discovery traffic. Simply running `ros2 topic list` from Jazzy while Humble subscribers exist can trigger this issue.

2. **Message Serialization Changes**: Different ROS2 distributions may use different message serialization formats or versions, causing silent data corruption or communication failures.

3. **DDS Version Mismatches**: The underlying DDS implementations (Fast-DDS, Cyclone DDS) evolve between distributions and may have protocol-level incompatibilities.

#### Why This Matters for Micro-ROS Specifically

Your current setup uses:
```bash
docker run microros/micro-ros-agent:humble udp4 --port 8090
```

The Micro-ROS agent acts as a **bridge** between the ESP32 (running micro-ROS client) and the ROS2 network. If you:
- Keep the agent on Humble
- Upgrade everything else to Jazzy

You would have:
```
ESP32 ←→ Micro-ROS Agent (Humble) ←→ ROS2 Nodes (Jazzy)
                                    ↑
                            Cross-distribution
                            communication here
```

This creates exactly the unsupported cross-distribution scenario that causes problems.

#### DDS Compatibility Constraints

Micro-ROS has additional constraints:
- **Fast-DDS Required**: Micro-ROS only guarantees compatibility with Fast-DDS (not Cyclone DDS)
- **Services/Actions**: Cross-vendor DDS communication only works for topics, not services or actions
- **Your robot uses**: `/odom`, `/cmd_vel`, `/scan` topics AND likely services for navigation

#### What Would Break

| Component | Likely Failure Mode |
|-----------|---------------------|
| Odometry (`/odom`) | May work intermittently, data corruption possible |
| Velocity commands (`/cmd_vel`) | Robot may not respond or respond erratically |
| Laser scan (`/scan`) | Data may be corrupted or missing |
| TF transforms | Transform tree may become inconsistent |
| Services | **Will not work** across distributions |
| Actions (Nav2 goals) | **Will not work** across distributions |

#### The Only Safe Options

1. **Upgrade Everything Together**
   - Micro-ROS agent to Jazzy
   - ESP32 firmware to Jazzy-compatible micro-ROS
   - All ROS2 nodes to Jazzy

2. **Keep Everything on Humble**
   - Maintain current Ubuntu 22.04 + ROS2 Humble
   - Supported until May 2027

3. **Isolated Testing Only**
   - Use separate `ROS_DOMAIN_ID` values
   - No cross-distribution communication
   - Run Humble and Jazzy systems completely independently

#### Docker Isolation Won't Help

Even running the Micro-ROS agent in a Docker container doesn't solve this:
- The agent still communicates over the DDS network
- DDS discovery happens at the network level
- The Docker container shares the host network (`--net=host`)

#### References

- [ROS2 Cross-Distribution Incompatibility Discussion](https://discourse.openrobotics.org/t/incompatability-between-distributions/43747)
- [Humble/Jazzy Memory Issue](https://github.com/ros2/rmw_fastrtps/issues/797)
- [micro-ROS DDS Compatibility](https://answers.ros.org/question/405948/communication-between-two-different-distributions-using-micro-ros/)

---

### 5. Python 3.12 Compatibility (MEDIUM EFFORT)

Ubuntu 24.04 ships with Python 3.12, which introduces breaking changes:

#### Key Issues:
- ROS2 Jazzy is built against Python 3.12
- Some libraries (e.g., pyrealsense2) don't support Python 3.12
- `pip install` behavior changed - virtual environments now required
- colcon uses system Python by default

#### ROSIE Dependencies Review:
| Package | Python 3.12 Status |
|---------|-------------------|
| faster-whisper | Check compatibility |
| sounddevice | Compatible |
| numpy | Compatible |
| webrtcvad | May need update |
| Flask | Compatible |
| pynput | Compatible |
| pyaudio | Compatible |

#### Workarounds:
- Use virtual environments with `python -m colcon build`
- Some packages may need manual building
- Consider Docker for problematic dependencies

#### Resources:
- [ROS2 and Python Virtual Environments](https://robotics.stackexchange.com/questions/115407/best-practice-when-using-ros2-and-python-virtual-env)

### 6. robot_localization / EKF (LOW EFFORT)

The robot_localization package is available for Jazzy with minimal changes expected.

```bash
sudo apt install ros-jazzy-robot-localization
```

#### Resources:
- [robot_localization Package](https://index.ros.org/p/robot_localization/)
- [Sensor Fusion Tutorial](https://automaticaddison.com/sensor-fusion-and-robot-localization-using-ros-2-jazzy/)

### 7. IMU Tools (LOW-MEDIUM EFFORT)

The imu_ws workspace will need verification:
- Check if imu_tools packages are available for Jazzy
- Verify EKF configuration compatibility
- Test IMU data fusion pipeline

---

## Step-by-Step Upgrade Plan

### Phase 1: Preparation (Estimated: 1-2 days)

1. **Backup everything**
   ```bash
   cp -r ~/projects/b4m_yahboom ~/projects/b4m_yahboom_backup_$(date +%Y%m%d)
   ```

2. **Document current working state**
   - Test all modes on current system
   - Record working parameters
   - Save RViz configurations

3. **Create test environment**
   - Set up Ubuntu 24.04 VM or dual-boot
   - Install ROS2 Jazzy

### Phase 2: Core Package Migration (Estimated: 3-5 days)

1. **Install ROS2 Jazzy base**
   ```bash
   sudo apt install ros-jazzy-desktop
   ```

2. **Update package.xml files**
   - Change build dependencies
   - Update version constraints

3. **Migrate custom packages**
   - Update Python shebangs to `#!/usr/bin/env python3`
   - Fix any Python 3.12 incompatibilities
   - Update CMakeLists.txt if needed

### Phase 3: Gazebo Migration (Estimated: 5-10 days)

This is the most time-consuming phase:

1. **Install Gazebo Harmonic**
   ```bash
   sudo apt install ros-jazzy-ros-gz
   ```

2. **Update launch files**
   - Convert gazebo_ros references to ros_gz_sim
   - Update spawn commands
   - Configure topic bridges

3. **Update world files**
   - Convert SDF format if needed
   - Update plugin references

4. **Update robot URDF**
   - Change Gazebo plugin names
   - Update sensor configurations

5. **Test simulation**
   - Verify robot spawning
   - Check sensor data (laser, IMU)
   - Test odometry and TF tree

### Phase 4: SLAM Migration (Estimated: 2-3 days)

1. **Install slam_toolbox**
   ```bash
   sudo apt install ros-jazzy-slam-toolbox
   ```

2. **Update launch files**
   - Replace Cartographer/gmapping references
   - Configure slam_toolbox parameters

3. **Test mapping**
   - Verify map generation
   - Check localization accuracy

### Phase 5: Navigation Stack (Estimated: 2-3 days)

1. **Install Nav2**
   ```bash
   sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
   ```

2. **Update parameters**
   - Review and update nav2_params.yaml
   - Consider MPPI controller parameters

3. **Test navigation**
   - Verify path planning
   - Test obstacle avoidance
   - Check recovery behaviors

### Phase 6: Micro-ROS & Hardware (Estimated: 2-3 days)

1. **Update Micro-ROS agent**
   - Try `microros/micro-ros-agent:jazzy` Docker image
   - Build from source if needed

2. **Test hardware communication**
   - Verify ESP32 connection
   - Check sensor data flow
   - Test motor control

### Phase 7: Integration Testing (Estimated: 3-5 days)

1. Test each b4m_launch.sh mode:
   - `--simulation`
   - `--explore`
   - `--nav`
   - `--ollama-*` modes
   - `--b4m-HA`

2. Run regression tests

3. Verify ROSIE functionality

---

## Risk Assessment

### High Risk Areas

| Component | Risk Level | Mitigation |
|-----------|------------|------------|
| Gazebo Migration | **High** | Budget extra time, extensive testing |
| Micro-ROS Jazzy | **High** | Have fallback plan, consider Docker isolation |
| Python 3.12 Breaking Changes | **Medium-High** | Test all Python scripts early |
| SLAM Accuracy Changes | **Medium** | Comparative testing with saved maps |

### Rollback Strategy

1. Keep Ubuntu 22.04 partition/VM available
2. Maintain git branches for both versions
3. Document all changes for potential reversal

---

## Alternative: Stay on Ubuntu 22.04 + ROS2 Humble

### Reasons to Consider Staying:

1. **Stability**: Current system is working
2. **Humble LTS**: Supported until May 2027
3. **Gazebo Classic**: Still functional (though EOL)
4. **Lower Risk**: No migration effort required
5. **Micro-ROS Maturity**: Better Humble support currently

### When to Upgrade:

1. **Gazebo Classic issues**: If simulation breaks due to EOL
2. **New Features Needed**: Jazzy has improvements
3. **Hardware Upgrade**: New computer with Ubuntu 24.04
4. **2027 Approaching**: Before Humble EOL

---

## Timeline Summary

| Phase | Duration | Dependencies |
|-------|----------|--------------|
| Preparation | 1-2 days | None |
| Core Migration | 3-5 days | Preparation |
| Gazebo Migration | 5-10 days | Core Migration |
| SLAM Migration | 2-3 days | Gazebo (for simulation testing) |
| Navigation Stack | 2-3 days | SLAM |
| Micro-ROS & Hardware | 2-3 days | Core Migration |
| Integration Testing | 3-5 days | All phases |

**Total Estimated Time: 3-5 weeks** (depending on issues encountered)

---

## Resources & References

### Official Documentation
- [ROS2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Gazebo Harmonic ROS Installation](https://gazebosim.org/docs/latest/ros_installation/)
- [Gazebo Classic Migration Guide](https://gazebosim.org/docs/latest/migrating_gazebo_classic_ros2_packages/)
- [Nav2 Jazzy Migration](https://docs.nav2.org/migration/Jazzy.html)
- [slam_toolbox Documentation](https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/)

### Community Resources
- [ROS Discourse](https://discourse.openrobotics.org/)
- [Gazebo Community](https://community.gazebosim.org/)
- [micro-ROS GitHub](https://github.com/micro-ROS/micro_ros_setup)

### Release Information
- [ROS2 Jazzy Release (May 2024)](https://www.openrobotics.org/blog/2024/5/ros-jazzy-jalisco-released)
- [ROS2 Kilted Release (May 2025)](https://docs.ros.org/en/jazzy/Releases/Release-Kilted-Kaiju.html) - Non-LTS
- [Jazzy EOL: May 2029](https://docs.ros.org/en/jazzy/Releases.html)

---

## Conclusion

Upgrading to Ubuntu 24.04 with ROS2 Jazzy is a significant undertaking, primarily due to the **Gazebo Classic to Gazebo Harmonic migration**. The decision should be based on:

1. **Urgency**: Is there a pressing need to upgrade?
2. **Resources**: Do you have 3-5 weeks to dedicate?
3. **Risk Tolerance**: Can you handle potential regressions?

If the current system meets your needs, **staying on Ubuntu 22.04 + ROS2 Humble until 2027** is a valid and pragmatic choice. If upgrading, plan for the Gazebo migration to consume the majority of your time.

---

*Document generated: November 2025*
*Current system: Ubuntu 22.04, ROS2 Humble, Gazebo Classic 11.10.2*
