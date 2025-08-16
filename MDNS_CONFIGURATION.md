# mDNS Configuration for B4M Robot

## Overview

The B4M robot configuration now supports mDNS (multicast DNS) hostname resolution, allowing you to use hostnames like `robot.local` instead of fixed IP addresses. This is particularly useful in environments where IP addresses may change due to DHCP.

## Benefits of Using mDNS

1. **No Fixed IP Required**: Use memorable hostnames instead of IP addresses
2. **Dynamic IP Support**: Works even when DHCP assigns different IPs
3. **Network Flexibility**: Easier to move robots between networks
4. **Zero Configuration**: Automatic discovery on the same subnet

## Prerequisites

### Linux (Ubuntu/Debian)
```bash
# Install Avahi for mDNS support
sudo apt-get update
sudo apt-get install avahi-daemon avahi-utils

# Verify Avahi is running
systemctl status avahi-daemon
```

### macOS
- mDNS (Bonjour) is built-in and enabled by default

### Windows
- Install Bonjour Print Services or iTunes (includes Bonjour)

## Configuration Methods

### Method 1: Environment Variable (Recommended)

```bash
# Set the hostname for the Micro-ROS agent
export ROBOT_AGENT_HOSTNAME="victus.local"

# Configure the robot
python3 config_robot.py
```

### Method 2: .env File

Create or edit `.env` file:
```env
# Robot network configuration
ROBOT_WIFI_SSID=your_wifi_network
ROBOT_WIFI_PASSWORD=your_password

# Use hostname instead of IP
ROBOT_AGENT_HOSTNAME=victus.local
# Note: This takes precedence over ROBOT_AGENT_IP if both are set
```

### Method 3: Fallback to IP

If hostname resolution fails, the system will fall back to IP:
```bash
export ROBOT_AGENT_HOSTNAME="robot.local"  # Primary (might fail)
export ROBOT_AGENT_IP="192.168.68.105"     # Fallback
```

## Setup Helper Script

Use the provided setup script to configure and test mDNS:

```bash
# Run the mDNS setup helper
./setup_mdns.sh

# This script will:
# - Check mDNS installation
# - Test hostname resolution
# - Optionally set a custom hostname
# - Provide configuration recommendations
```

## Testing mDNS Resolution

### Quick Test
```bash
# Test if a hostname resolves
avahi-resolve -n victus.local

# Or use the test script
python3 test_mdns_config.py
```

### Manual Testing
```python
from config_robot import resolve_hostname_to_ip

# Test hostname resolution
ip = resolve_hostname_to_ip("victus.local")
print(f"Resolved to: {ip}")
```

## How It Works

1. **Configuration**: Set `ROBOT_AGENT_HOSTNAME` to your desired hostname
2. **Resolution**: `config_robot.py` automatically resolves the hostname to IP
3. **Robot Programming**: The resolved IP is sent to the ESP32 firmware
4. **Connection**: ESP32 connects to Micro-ROS agent at the resolved IP

## Architecture

```
┌─────────────────┐     mDNS Query      ┌──────────────────┐
│  config_robot   │────────────────────>│  Avahi/Bonjour   │
│     (Python)    │<────────────────────│    (System)      │
└────────┬────────┘     IP Address      └──────────────────┘
         │
         │ Serial (USB)
         │ Sends resolved IP
         ▼
┌─────────────────┐     UDP Connection  ┌──────────────────┐
│   ESP32 Robot   │────────────────────>│  Micro-ROS Agent │
│   (Firmware)    │      Port 8090      │    (Docker)      │
└─────────────────┘                     └──────────────────┘
```

## Troubleshooting

### Hostname Not Resolving

1. **Check Avahi is running**:
   ```bash
   sudo systemctl status avahi-daemon
   sudo systemctl start avahi-daemon
   ```

2. **Verify network connectivity**:
   ```bash
   ping victus.local
   ```

3. **Test with avahi-browse**:
   ```bash
   avahi-browse -a  # List all mDNS services
   ```

### Common Issues

- **"Name or service not known"**: Target device not advertising via mDNS
- **Different subnets**: mDNS only works on the same network segment
- **Firewall blocking**: Port 5353 (UDP) must be open for mDNS

### Docker Considerations

The Micro-ROS agent runs with `--net=host`, so it inherits the host's network configuration and mDNS resolution capabilities. No additional Docker configuration is needed.

## Best Practices

1. **Use descriptive hostnames**: `robot-1.local`, `robot-kitchen.local`
2. **Document your hostnames**: Keep a list of robot hostnames
3. **Test before deployment**: Verify mDNS works in your network environment
4. **Have a fallback**: Always know the IP address as backup

## Examples

### Full Configuration Example

```bash
# Set all robot configuration via environment
export ROBOT_WIFI_SSID="RobotNetwork"
export ROBOT_WIFI_PASSWORD="SecurePassword123"
export ROBOT_AGENT_HOSTNAME="robot-controller.local"
export ROBOT_AGENT_PORT="8090"

# Configure the robot
python3 config_robot.py
```

### Integration with b4m_launch.sh

The system works seamlessly with the existing launch scripts:
```bash
# The Micro-ROS agent will be accessible via mDNS
./b4m_launch.sh

# Robot configured with hostname will connect automatically
```

## Security Considerations

- mDNS broadcasts hostnames on the local network
- Use only on trusted networks
- Consider firewall rules for production deployments
- mDNS traffic is not encrypted

## Further Reading

- [Avahi Documentation](https://www.avahi.org/)
- [mDNS RFC 6762](https://tools.ietf.org/html/rfc6762)
- [Micro-ROS Documentation](https://micro.ros.org/)