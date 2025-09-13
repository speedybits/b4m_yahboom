To use a .local address instead of a fixed IP in Micro-ROS, you would leverage mDNS (Multicast DNS), which allows devices to be discovered on a local network using their hostname, typically ending in .local (e.g., robot.local). Instead of hardcoding a device's IP address, you reference its hostname, and the mDNS protocol will resolve this to the correct IP dynamically on the same subnet.

Steps to use .local addresses with Micro-ROS
Enable mDNS on your Micro-ROS device: Your device needs to advertise its hostname via mDNS. Most modern embedded operating systems (e.g., Linux with Avahi, ESP-IDF with mDNS component) support this.

Configure Micro-ROS client to connect using the .local hostname: Instead of specifying a fixed IP (192.168.x.x), use the hostname (such as robot.local).

Ensure mDNS support on the client: The computer or microcontroller running Micro-ROS must support mDNS. On most PCs, mDNS works out-of-the-box; on microcontrollers, you may need an additional library.

Avoid using .local for DHCP-assigned static IPs: .local is reserved for mDNS and should not be used for manual static DNS entries unless mDNS is enabled.

Important notes
Not all networks or routers support mDNS out-of-the-box. Some require additional configuration or installation of mDNS responders (like Avahi for Linux or Bonjour for Windows).

Micro-ROS itself does not manage hostnames, so this setup must be handled at your OS/network layer.


Example
If your agent is running on robot.local, configure your Micro-ROS client to connect using:

text
ros_agent_ip: "robot.local"
Instead of:

text
ros_agent_ip: "192.168.1.10"
This way, if the agent's IP changes, discovery remains automatic as long as mDNS is active and network conditions allow.
