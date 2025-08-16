#!/usr/bin/env python3
"""
Test script to demonstrate mDNS hostname resolution for robot configuration.
This shows how the robot can be configured using hostnames instead of fixed IPs.
"""

import os
import sys
from config_robot import resolve_hostname_to_ip

def test_hostname_resolution():
    """Test various hostname resolution scenarios"""
    
    print("=" * 60)
    print("mDNS Hostname Resolution Test for B4M Robot")
    print("=" * 60)
    print()
    
    # Test cases
    test_cases = [
        ("192.168.68.105", "Fixed IP address"),
        ("victus.local", "Local mDNS hostname"),
        ("localhost", "Localhost"),
        ("robot.local", "Example robot hostname (may not exist)"),
    ]
    
    results = []
    
    for hostname, description in test_cases:
        print(f"Testing: {hostname} ({description})")
        try:
            resolved_ip = resolve_hostname_to_ip(hostname)
            print(f"  ✓ Success: Resolved to {resolved_ip}")
            results.append((hostname, resolved_ip, True))
        except Exception as e:
            print(f"  ✗ Failed: {str(e)}")
            results.append((hostname, None, False))
        print()
    
    # Summary
    print("=" * 60)
    print("Summary:")
    print("-" * 60)
    
    successful = sum(1 for _, _, success in results if success)
    total = len(results)
    
    print(f"Successful resolutions: {successful}/{total}")
    print()
    
    for hostname, ip, success in results:
        status = "✓" if success else "✗"
        ip_str = ip if ip else "Failed to resolve"
        print(f"  {status} {hostname:20} -> {ip_str}")
    
    print()
    print("=" * 60)
    
    # Show how to use with environment variables
    print("\nHow to use with environment variables:")
    print("-" * 40)
    print("1. Using hostname (recommended for dynamic environments):")
    print("   export ROBOT_AGENT_HOSTNAME=\"victus.local\"")
    print("   python3 config_robot.py")
    print()
    print("2. Using fixed IP (traditional method):")
    print("   export ROBOT_AGENT_IP=\"192.168.68.105\"")
    print("   python3 config_robot.py")
    print()
    print("3. In .env file:")
    print("   ROBOT_AGENT_HOSTNAME=victus.local")
    print("   # or")
    print("   ROBOT_AGENT_IP=192.168.68.105")
    print()
    
    # Test current environment
    print("Current environment configuration:")
    print("-" * 40)
    
    hostname_env = os.environ.get('ROBOT_AGENT_HOSTNAME')
    ip_env = os.environ.get('ROBOT_AGENT_IP')
    
    if hostname_env:
        print(f"ROBOT_AGENT_HOSTNAME is set: {hostname_env}")
        try:
            resolved = resolve_hostname_to_ip(hostname_env)
            print(f"  Would resolve to: {resolved}")
        except:
            print(f"  Warning: Cannot resolve this hostname")
    else:
        print("ROBOT_AGENT_HOSTNAME is not set")
    
    if ip_env:
        print(f"ROBOT_AGENT_IP is set: {ip_env}")
    else:
        print("ROBOT_AGENT_IP is not set (would use default: 192.168.1.100)")
    
    print()
    print("Note: ROBOT_AGENT_HOSTNAME takes precedence over ROBOT_AGENT_IP")
    print("=" * 60)

if __name__ == "__main__":
    test_hostname_resolution()