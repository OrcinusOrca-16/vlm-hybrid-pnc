import math
from dataclasses import dataclass

from src.common.types import PathPoint, VehicleState


@dataclass(frozen=True)
class PathReferenceMatch:
    """Interpolated geometric reference matched to the current vehicle pose."""

    x: float
    y: float
    yaw: float
    curvature: float
    s: float
    segment_index: int


@dataclass(frozen=True)
class SafeSpeedLimitDiagnostic:
    """Path segment that most strongly limits steering-rate-based speed."""

    segment_index: int
    start_s: float
    end_s: float
    start_curvature: float
    end_curvature: float
    start_steering: float
    end_steering: float
    dsteering_ds: float
    speed_limit_mps: float


def normalize_angle(angle: float) -> float:
    """Normalize one angle to [-pi, pi]."""

    return math.atan2(math.sin(angle), math.cos(angle))


def curvature_to_steering(curvature_per_m: float, wheelbase_m: float) -> float:
    """Convert path curvature to kinematic-bicycle steering angle."""

    if wheelbase_m <= 0.0:
        raise ValueError("wheelbase_m must be positive.")
    if not math.isfinite(curvature_per_m):
        raise ValueError("curvature_per_m must be finite.")
    return math.atan(wheelbase_m * curvature_per_m)


def find_safe_speed_limiting_segment(
    path: list[PathPoint],
    wheelbase_m: float,
    max_steering_angle: float,
    max_steering_rate: float,
    numerical_tolerance: float = 1e-9,
) -> SafeSpeedLimitDiagnostic | None:
    """Return the path segment with the largest absolute steering gradient."""

    if not path:
        raise ValueError("path must contain at least one point.")
    if wheelbase_m <= 0.0:
        raise ValueError("wheelbase_m must be positive.")
    if max_steering_angle <= 0.0:
        raise ValueError("max_steering_angle must be positive.")
    if max_steering_rate <= 0.0:
        raise ValueError("max_steering_rate must be positive.")
    if numerical_tolerance <= 0.0:
        raise ValueError("numerical_tolerance must be positive.")

    steering_angles = [curvature_to_steering(point.curvature, wheelbase_m) for point in path]
    for steering in steering_angles:
        if abs(steering) > max_steering_angle + numerical_tolerance:
            raise ValueError("Path requires steering outside the configured steering-angle limit.")

    if len(path) == 1:
        return None

    best: SafeSpeedLimitDiagnostic | None = None
    max_abs_gradient = 0.0

    for index in range(len(path) - 1):
        delta_s = path[index + 1].s - path[index].s
        if delta_s <= 0.0:
            raise ValueError("PathPoint.s must be strictly increasing.")

        dsteering_ds = (steering_angles[index + 1] - steering_angles[index]) / delta_s
        abs_gradient = abs(dsteering_ds)
        if abs_gradient <= max_abs_gradient:
            continue

        max_abs_gradient = abs_gradient
        speed_limit = math.inf if abs_gradient <= numerical_tolerance else max_steering_rate / abs_gradient
        best = SafeSpeedLimitDiagnostic(
            segment_index=index,
            start_s=path[index].s,
            end_s=path[index + 1].s,
            start_curvature=path[index].curvature,
            end_curvature=path[index + 1].curvature,
            start_steering=steering_angles[index],
            end_steering=steering_angles[index + 1],
            dsteering_ds=dsteering_ds,
            speed_limit_mps=speed_limit,
        )

    return best


def compute_constant_safe_speed(
    path: list[PathPoint],
    target_speed_mps: float,
    wheelbase_m: float,
    max_steering_angle: float,
    max_steering_rate: float,
    numerical_tolerance: float = 1e-9,
) -> float:
    """Return one steering-rate-limited constant speed for the whole path."""

    if target_speed_mps < 0.0:
        raise ValueError("target_speed_mps must not be negative.")

    limiting_segment = find_safe_speed_limiting_segment(
        path=path,
        wheelbase_m=wheelbase_m,
        max_steering_angle=max_steering_angle,
        max_steering_rate=max_steering_rate,
        numerical_tolerance=numerical_tolerance,
    )

    if limiting_segment is None or not math.isfinite(limiting_segment.speed_limit_mps):
        return target_speed_mps

    return min(target_speed_mps, limiting_segment.speed_limit_mps)


def match_path_reference(
    path: list[PathPoint],
    vehicle_state: VehicleState,
    start_segment_index: int = 0,
) -> PathReferenceMatch:
    """Match the vehicle to the closest forward path segment."""

    if len(path) < 2:
        raise ValueError("path must contain at least two points for segment matching.")
    if not isinstance(start_segment_index, int):
        raise TypeError("start_segment_index must be an integer.")
    if not 0 <= start_segment_index < len(path) - 1:
        raise ValueError("start_segment_index must identify an existing path segment.")

    best_distance_squared = math.inf
    best_match: PathReferenceMatch | None = None

    for segment_index in range(start_segment_index, len(path) - 1):
        start = path[segment_index]
        end = path[segment_index + 1]
        segment_x = end.x - start.x
        segment_y = end.y - start.y
        segment_norm_squared = segment_x**2 + segment_y**2

        if segment_norm_squared <= 1e-12:
            raise ValueError("Path contains a zero-length Cartesian segment.")

        vehicle_offset_x = vehicle_state.x - start.x
        vehicle_offset_y = vehicle_state.y - start.y
        interpolation = (vehicle_offset_x * segment_x + vehicle_offset_y * segment_y) / segment_norm_squared
        interpolation = max(0.0, min(1.0, interpolation))

        reference_x = start.x + interpolation * segment_x
        reference_y = start.y + interpolation * segment_y
        error_x = vehicle_state.x - reference_x
        error_y = vehicle_state.y - reference_y
        distance_squared = error_x**2 + error_y**2

        if distance_squared >= best_distance_squared:
            continue

        yaw_delta = normalize_angle(end.yaw - start.yaw)
        reference_yaw = normalize_angle(start.yaw + interpolation * yaw_delta)
        reference_curvature = start.curvature + interpolation * (end.curvature - start.curvature)
        reference_s = start.s + interpolation * (end.s - start.s)

        best_distance_squared = distance_squared
        best_match = PathReferenceMatch(
            x=reference_x,
            y=reference_y,
            yaw=reference_yaw,
            curvature=reference_curvature,
            s=reference_s,
            segment_index=segment_index,
        )

    if best_match is None:
        raise RuntimeError("Failed to match vehicle pose to the path.")
    return best_match


def compute_path_tracking_error(vehicle_state: VehicleState, reference: PathReferenceMatch) -> tuple[float, float]:
    """Return signed lateral and heading errors to one matched path reference."""

    error_x = vehicle_state.x - reference.x
    error_y = vehicle_state.y - reference.y
    normal_x = -math.sin(reference.yaw)
    normal_y = math.cos(reference.yaw)
    lateral_error = error_x * normal_x + error_y * normal_y
    heading_error = normalize_angle(vehicle_state.yaw - reference.yaw)
    return lateral_error, heading_error
