from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Tuple, Optional, Iterable, Dict, Any, Set


class OperationalMode(Enum):
    """High-level mission operational states"""
    SCAN_AREA = auto()  # Autonomous area scanning using frontier detection
    NAVIGATE_BASE = auto()  # Return to initial position from any location
    REACH_GOAL = auto()  # Navigate from start to target using optimal path
    COMPLETED = auto()  # Mission accomplishment


@dataclass
class PositionData:
    """Global coordinate system position (meters/degrees), theta in degrees, CCW positive"""
    coord_x: float
    coord_y: float
    orientation: float


@dataclass
class DriveConfig:
    """Kinematic calibration parameters for odometry and encoder conversion"""
    wheel_diameter: float  # Wheel diameter in millimeters
    axle_half_length: float  # Half distance between wheels
    encoder_resolution: int  # Encoder pulses per revolution


@dataclass
class MotionConstraints:
    """Motion execution accuracy thresholds"""
    position_threshold: float = 0.08  # Position acceptance margin
    heading_threshold: float = 15  # Orientation acceptance margin


@dataclass
class NavigationConstraints:
    """Path following precision requirements"""
    waypoint_tolerance: float = 0.05  # Waypoint proximity threshold
    angular_tolerance: float = 30  # Heading alignment tolerance


@dataclass
class KinematicLimits:
    """Velocity and timing constraints for trajectory segmentation"""
    maximum_velocity: float = 0.3
    maximum_rotation: float = 120.0
    minimum_duration: float = 0.15
    maximum_duration: float = 1.2


@dataclass
class MotorCommand:
    """
    Executable motion command: specifies target cumulative encoder counts
    for both wheels with required execution duration.
    The control system should drive wheel cumulative values to reach these
    counts within the specified time frame.
    """
    left_encoder_total: int
    right_encoder_total: int
    execution_period: int
    intermediate_target: PositionData  # Local target pose for monitoring
    accuracy_requirements: MotionConstraints


# Environment classification thresholds
CLEARANCE_THRESHOLD = 250
COLLISION_THRESHOLD = 45


@dataclass
class PathSegment:
    """Continuous motion segment between waypoints"""
    start_pose: PositionData
    end_pose: PositionData
    curvature_profile: List[float]
    velocity_profile: List[float]


@dataclass
class NavigationContext:
    """Complete navigation state and parameters"""
    current_mode: OperationalMode
    vehicle_pose: PositionData
    trajectory_plan: List[PositionData]
    active_constraints: NavigationConstraints
    system_limits: KinematicLimits


@dataclass
class SensorReadings:
    """Real-time sensor data collection"""
    proximity_data: List[float]
    inertial_measurements: Tuple[float, float, float]
    wheel_encoders: Tuple[int, int]
    environmental_flags: Set[str]


class MotionPlanner:
    """Generates smooth motion trajectories"""

    def generate_trajectory(self,
                            start: PositionData,
                            destination: PositionData,
                            constraints: KinematicLimits) -> List[MotorCommand]:
        """Convert global path to executable motor commands"""
        # Implementation would calculate intermediate points
        # and convert to encoder targets
        pass

    def calculate_curvature(self, current: PositionData, target: PositionData) -> float:
        """Compute path curvature between positions"""
        # Implementation for curvature calculation
        pass


class StateSupervisor:
    """Monitors mission progress and state transitions"""

    def evaluate_completion(self,
                            actual: PositionData,
                            desired: PositionData,
                            thresholds: MotionConstraints) -> bool:
        """Check if target position is achieved within tolerances"""
        position_error = ((actual.coord_x - desired.coord_x) ** 2 +
                          (actual.coord_y - desired.coord_y) ** 2) ** 0.5
        heading_error = abs(actual.orientation - desired.orientation)

        return (position_error <= thresholds.position_threshold and
                heading_error <= thresholds.heading_threshold)

    def transition_mode(self,
                        current: OperationalMode,
                        conditions: Dict[str, Any]) -> OperationalMode:
        """Determine appropriate operational mode based on conditions"""
        # Implementation for state machine transitions
        pass


# Utility functions
def compute_pose_difference(a: PositionData, b: PositionData) -> Tuple[float, float]:
    """Calculate positional and angular differences between poses"""
    distance = ((a.coord_x - b.coord_x) ** 2 + (a.coord_y - b.coord_y) ** 2) ** 0.5
    angle_diff = (a.orientation - b.orientation) % 360
    return distance, min(angle_diff, 360 - angle_diff)


def validate_environment_mapping(scan_data: List[int]) -> bool:
    """Verify environment data quality for navigation"""
    valid_readings = [x for x in scan_data if x > COLLISION_THRESHOLD]
    return len(valid_readings) > len(scan_data) * 0.7