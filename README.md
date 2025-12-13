# NC Youth Robotics - ROS2 Project

This repository contains a complete ROS2-based robotics platform for educational purposes.

## Documentation

The complete project documentation is available in **`main.tex`**.

### Compiling the Documentation

To generate the PDF documentation:

```bash
# Install LaTeX (Ubuntu/Debian)
sudo apt-get install texlive-latex-base texlive-latex-extra texlive-fonts-recommended

# Compile the documentation
pdflatex main.tex

# If you have references or need to regenerate TOC, run twice:
pdflatex main.tex
pdflatex main.tex
```

The documentation covers:
- Installation guides for ROS2 Humble on Ubuntu and Raspberry Pi
- PlatformIO setup for Arduino development
- Complete code documentation with snippets
- How to build, run, and deploy the system
- Using RQT for debugging and visualization
- Troubleshooting common issues

## Quick Start

1. **Read the documentation**: Compile `main.tex` and read the PDF
2. **Install ROS2 Humble**: Follow the installation guide in the documentation
3. **Build the workspace**:
   ```bash
   cd ros2ws/
   rosdep install --from-paths src -y --ignore-src
   colcon build --symlink-install
   source install/setup.bash
   ```
4. **Program the Arduino**:
   ```bash
   cd Arduino/
   pio run --target upload
   ```
5. **Launch the system**:
   ```bash
   ros2 launch robot_drive_interface multi_node.launch.py
   ```

## Project Structure

- `ros2ws/` - ROS2 workspace containing all ROS2 packages
- `Arduino/` - PlatformIO project for motor control firmware
- `shared/` - Shared protocol definitions between ROS2 and Arduino
- `main.tex` - Complete documentation in LaTeX format
- `Dockerfile` - Container for deployment
- `deploy.sh` - Deployment script for Raspberry Pi

## Key Packages

- **robot_command_node** - Gamepad controller interface
- **robot_drive_interface** - Serial communication with Arduino and camera feed
- **robot_perception** - YOLO-based object detection
- **robot_msgs** - Custom ROS2 message definitions

See the full documentation for detailed information about each package.

## License

[Specify your license here]

## Contact

NC Youth Robotics
