# 2D Robotic Drawing Machine (2drdmnp)

A Python-based image processing system that converts digital images into coordinate instructions for a 2-axis robotic drawing machine using Arduino control.

## Overview

This project processes digital images and generates optimized drawing paths for a robotic arm system. The system converts bitmap images into a series of coordinate points, calculates optimal drawing sequences, and transmits servo motor angles via serial communication to an Arduino-controlled robotic drawing machine.

## Features

- **Image Processing**: Converts color images to binary (black/white) format
- **Path Optimization**: Implements nearest-neighbor algorithm for efficient drawing sequences
- **Inverse Kinematics**: Calculates servo angles for 2-DOF robotic arm positioning
- **Serial Communication**: Real-time data transmission to Arduino via PySerial
- **Pen Control**: Manages pen up/down states for continuous vs. discrete drawing
- **Coordinate Export**: Saves processed data to CSV format for analysis

## System Architecture

### Hardware Requirements
- 2-DOF robotic arm with servo motors
- Arduino microcontroller (compatible with `/dev/ttyACM1` or `COM1`)
- Serial communication interface
- Drawing surface and pen mechanism

### Software Components
1. **Image Preprocessing**: OpenCV-based image conversion and scaling
2. **Path Planning**: Coordinate extraction and optimization algorithms  
3. **Kinematics Engine**: Forward/inverse kinematics calculations
4. **Communication Layer**: Serial protocol for Arduino interface

## Installation

### Prerequisites
```bash
pip install opencv-python numpy matplotlib pyserial
```

### Dependencies
- `cv2` (OpenCV) - Image processing
- `numpy` - Numerical computations
- `matplotlib` - Visualization and plotting
- `serial` - Arduino communication
- `math` - Trigonometric calculations

## Usage

### Basic Operation
1. Place your input image as `test12.png` in the project directory
2. Connect Arduino to `/dev/ttyACM1` (Linux/Mac) or modify to `COM1` (Windows)
3. Run the main script:
```bash
python FinalCodetoBePresented.py
```

### Configuration Parameters
- **Arm Lengths**: `l1 = l2 = 353.624` (modify based on your hardware)
- **Serial Port**: `/dev/ttyACM1` (Linux/Mac) or `COM1` (Windows)
- **Baud Rate**: `9600`
- **Image Scale**: `40%` reduction for processing optimization

## Algorithm Details

### Image Processing Pipeline
1. **Color Conversion**: RGB → Grayscale → Binary threshold
2. **Scaling**: Reduces image size for processing efficiency
3. **Coordinate Extraction**: Identifies all black pixels as drawing points

### Path Optimization
- **Nearest Neighbor**: Minimizes pen travel distance between points
- **Pen State Logic**: Determines when to lift/lower pen based on pixel proximity
- **Distance Matrix**: Pre-calculates all point-to-point distances

### Inverse Kinematics
For a 2-DOF planar arm:
```
α = (π/2) + atan(y/x) - (1/2)×acos(1-(x²+y²)/(2×l1×l2))
β = acos(1-(x²+y²)/(2×l1×l2)) - π
```

## File Structure

```
2drdmnp/
├── FinalCodetoBePresented.py    # Main processing script
├── README.md                    # Project documentation
├── finalCSV.csv                 # Generated coordinate data
├── leoutput.csv                 # Alternative output format
├── robodraw.pdf                 # Technical documentation
└── test12.png                   # Input image (user-provided)
```

## Data Format

The system generates CSV files with three data rows:
- **Row 1**: Alpha angles (servo 1 positions in radians)
- **Row 2**: Beta angles (servo 2 positions in radians)  
- **Row 3**: Pen states (0 = up, 1 = down)

## Serial Protocol

Data transmission format to Arduino:
```
[255][Alpha_byte][Beta_byte][Pen_state]
```
- Start byte: `255` (synchronization marker)
- Angle conversion: `(angle × 180/π) % 255`
- Transmission delay: 1.5 seconds per command

## Troubleshooting

### Common Issues
- **Serial Connection**: Verify correct port and Arduino connection
- **Image Format**: Ensure input image is readable by OpenCV
- **Arm Limits**: Check if calculated positions are within workspace
- **Timing**: Adjust delay if Arduino processing is slower

### Performance Optimization
- Reduce image resolution for faster processing
- Implement more sophisticated path planning (TSP algorithms)
- Add collision detection for complex geometries

## Contributing

This project is open for improvements in:
- Advanced path optimization algorithms
- Real-time visualization
- Multi-color drawing support
- Calibration routines
- Error handling and recovery

## Technical Specifications

- **Workspace**: Determined by arm lengths (default: ~707mm reach)
- **Resolution**: Dependent on input image scaling
- **Processing Time**: Varies with image complexity
- **Communication**: 9600 baud serial interface

## License

This project is available for educational and research purposes. Please refer to the repository for specific licensing terms.

## Author

Created by sharma18b - A robotics project combining computer vision, path planning, and mechatronics for automated drawing applications.
