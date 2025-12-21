# Changelog

## [2.1.0] - December 2024

### Added
- Visual distance estimation from bounding box sizes
- Sensor health monitoring with stuck detection
- Vision-primary sensor fusion strategy
- Threat assessment and prioritization system
- Confidence-based weighted fusion
- Graceful degradation on sensor failures
- Enhanced logging with distance source tracking
- Conflict detection and resolution
- Auto-save on critical threats

### Changed
- Upgraded to YOLOv11n from YOLOv8n
- Reduced track expiry from 2s to 1s (prevents lag)
- Increased IoU threshold from 0.3 to 0.4 (better matching)
- Improved detection interval handling
- Enhanced debugging output

### Fixed
- "Finger problem" - objects visible to camera but outside sensor FOV
- Ghost boxes from slow-moving objects
- Sensor failures going undetected
- Distance estimation for small objects
- Lag effect from expired tracks

### Performance
- 8-10 FPS on Pi 4B (up from 3.4 FPS)
- <100ms end-to-end latency
- ±3cm accuracy with sensor validation
- ±15cm accuracy visual-only mode

## [1.0.0] - November 2024

### Initial Release
- YOLOv8n object detection
- Basic sensor fusion
- LIDAR, TOF, and ultrasonic sensors
- ESP32 sensor hub
- Audio feedback via buzzer
