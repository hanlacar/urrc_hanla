#!/usr/bin/env python3
"""Convert SIMPLE MCU encoder/steering feedback into wheel odometry.

The current SIMPLE firmware counts ENC_A RISING edges only, so its cumulative
count increases in both directions.  By default the adapter restores the
distance sign from /mcu/applied_drive.  Set encoder_counts_are_signed:=true
only for a future encoder source whose cumulative delta already carries the
direction; in that mode the drive sign is deliberately not applied again.

``counts_per_meter`` is the measured vehicle calibration: 797.0 count/m.
Steering is treated as zero-curvature until a separate raw-ADC validator marks
the feedback valid; the adapter never derives a believable angle from an
invalid sensor value.
"""

from dataclasses import dataclass
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32
from tf2_ros import TransformBroadcaster


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class IntegrationResult:
    accepted: bool
    baseline_only: bool = False
    reason: str = ""
    ds: float = 0.0
    dtheta: float = 0.0
    linear_x: float = 0.0
    angular_z: float = 0.0


class BicycleOdometry:
    """ROS-independent rear-axle bicycle odometry state and integration."""

    def __init__(
        self,
        *,
        wheelbase_m: float,
        counts_per_meter: float,
        max_encoder_delta_counts: int,
        encoder_counts_are_signed: bool = False,
        steering_feedback_required: bool = False,
    ):
        if not math.isfinite(wheelbase_m) or wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be finite and positive")
        if not math.isfinite(counts_per_meter) or counts_per_meter <= 0.0:
            raise ValueError("counts_per_meter must be finite and positive")
        if max_encoder_delta_counts <= 0:
            raise ValueError("max_encoder_delta_counts must be positive")

        self.wheelbase_m = float(wheelbase_m)
        self.counts_per_meter = float(counts_per_meter)
        self.max_encoder_delta_counts = int(max_encoder_delta_counts)
        self.encoder_counts_are_signed = bool(encoder_counts_are_signed)
        self.steering_feedback_required = bool(steering_feedback_required)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.steering_deg = 0.0
        self.steering_valid = not self.steering_feedback_required
        self.drive_sign = 0
        self._last_encoder = None
        self._last_stamp_sec = None

    def set_steering_deg(self, steering_deg: float) -> None:
        if math.isfinite(float(steering_deg)):
            self.steering_deg = float(steering_deg)

    def set_steering_valid(self, valid: bool) -> None:
        self.steering_valid = bool(valid)

    def set_applied_drive(self, applied_drive: float) -> None:
        value = float(applied_drive)
        if not math.isfinite(value):
            self.drive_sign = 0
        elif value > 0.0:
            self.drive_sign = 1
        elif value < 0.0:
            self.drive_sign = -1
        else:
            self.drive_sign = 0

    def update_encoder(self, encoder: int, stamp_sec: float) -> IntegrationResult:
        """Integrate one cumulative encoder sample.

        Every rejected sample still becomes the new baseline.  This is what
        prevents an MCU reset/reconnect from being replayed on the next sample.
        """
        encoder = int(encoder)
        stamp_sec = float(stamp_sec)
        if self._last_encoder is None:
            self._last_encoder = encoder
            self._last_stamp_sec = stamp_sec
            return IntegrationResult(accepted=True, baseline_only=True)

        previous_encoder = self._last_encoder
        previous_stamp = self._last_stamp_sec
        raw_delta = encoder - previous_encoder
        self._last_encoder = encoder
        self._last_stamp_sec = stamp_sec

        # ENC_A RISING is monotonic.  A decrease means MCU reset, reconnect, or
        # integer rollover; all are safer to re-baseline than to integrate.
        if not self.encoder_counts_are_signed and raw_delta < 0:
            return IntegrationResult(
                accepted=False,
                baseline_only=True,
                reason=(f"monotonic encoder decreased from {previous_encoder} "
                        f"to {encoder}"),
            )
        if abs(raw_delta) > self.max_encoder_delta_counts:
            return IntegrationResult(
                accepted=False,
                baseline_only=True,
                reason=(f"encoder delta {raw_delta} exceeds guard "
                        f"{self.max_encoder_delta_counts}"),
            )

        if self.encoder_counts_are_signed:
            signed_counts = raw_delta
        elif raw_delta == 0:
            signed_counts = 0
        elif self.drive_sign == 0:
            return IntegrationResult(
                accepted=False,
                baseline_only=True,
                reason=("encoder advanced while /mcu/applied_drive has no "
                        "direction"),
            )
        else:
            signed_counts = self.drive_sign * abs(raw_delta)

        ds = signed_counts / self.counts_per_meter
        # Invalid feedback must not corrupt yaw.  Propulsion is independently
        # blocked by mcu_simple_compat while this validity bit is false.
        effective_steering_deg = (
            self.steering_deg if self.steering_valid else 0.0)
        steer_rad = math.radians(effective_steering_deg)
        dtheta = ds * math.tan(steer_rad) / self.wheelbase_m
        theta_mid = self.yaw + 0.5 * dtheta
        self.x += ds * math.cos(theta_mid)
        self.y += ds * math.sin(theta_mid)
        self.yaw = normalize_angle(self.yaw + dtheta)

        linear_x = 0.0
        angular_z = 0.0
        if previous_stamp is not None:
            dt = stamp_sec - previous_stamp
            if math.isfinite(dt) and dt > 0.0:
                linear_x = ds / dt
                angular_z = dtheta / dt

        return IntegrationResult(
            accepted=True,
            ds=ds,
            dtheta=dtheta,
            linear_x=linear_x,
            angular_z=angular_z,
        )


def _set_planar_rotation(rotation, yaw: float) -> None:
    rotation.x = 0.0
    rotation.y = 0.0
    rotation.z = math.sin(0.5 * yaw)
    rotation.w = math.cos(0.5 * yaw)


def make_odometry(stamp, state, result, odom_frame, base_frame):
    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = str(odom_frame)
    odom.child_frame_id = str(base_frame)
    odom.pose.pose.position.x = state.x
    odom.pose.pose.position.y = state.y
    odom.pose.pose.position.z = 0.0
    _set_planar_rotation(odom.pose.pose.orientation, state.yaw)
    odom.twist.twist.linear.x = result.linear_x
    odom.twist.twist.angular.z = result.angular_z
    odom.pose.covariance[0] = 0.05
    odom.pose.covariance[7] = 0.05
    odom.pose.covariance[35] = 0.10 if state.steering_valid else 1.0e3
    odom.twist.covariance[0] = 0.05
    odom.twist.covariance[35] = 0.10 if state.steering_valid else 1.0e3
    return odom


def make_transform(stamp, state, odom_frame, base_frame):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = str(odom_frame)
    transform.child_frame_id = str(base_frame)
    transform.transform.translation.x = state.x
    transform.transform.translation.y = state.y
    transform.transform.translation.z = 0.0
    _set_planar_rotation(transform.transform.rotation, state.yaw)
    return transform


class McuOdomAdapter(Node):

    def __init__(self):
        super().__init__("mcu_odom_adapter")
        defaults = {
            "odom_topic": "/odom",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "wheelbase_m": 0.73,
            "counts_per_meter": 797.0,
            "publish_tf": True,
            "encoder_topic": "/mcu/encoder",
            "steering_topic": "/mcu/steer_deg",
            "steering_valid_topic": "/mcu/steering_feedback_valid",
            "steering_feedback_required": True,
            "drive_topic": "/mcu/applied_drive",
            "max_encoder_delta_counts": 1000,
            # Current ENC_A RISING firmware is unsigned/monotonic.
            "encoder_counts_are_signed": False,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in defaults}

        self.integrator = BicycleOdometry(
            wheelbase_m=float(self.p["wheelbase_m"]),
            counts_per_meter=float(self.p["counts_per_meter"]),
            max_encoder_delta_counts=int(
                self.p["max_encoder_delta_counts"]),
            encoder_counts_are_signed=bool(
                self.p["encoder_counts_are_signed"]),
            steering_feedback_required=bool(
                self.p["steering_feedback_required"]),
        )
        self.odom_pub = self.create_publisher(
            Odometry, str(self.p["odom_topic"]), 10)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if bool(self.p["publish_tf"]) else None)
        self.create_subscription(
            Int32, str(self.p["encoder_topic"]), self._on_encoder, 10)
        self.create_subscription(
            Float32, str(self.p["steering_topic"]), self._on_steering, 10)
        self.create_subscription(
            Bool, str(self.p["steering_valid_topic"]),
            self._on_steering_valid, 10)
        self.create_subscription(
            Float32, str(self.p["drive_topic"]), self._on_drive, 10)

        direction_mode = (
            "signed encoder delta" if self.integrator.encoder_counts_are_signed
            else "monotonic ENC_A + applied-drive sign")
        self.get_logger().info(
            f"SIMPLE MCU odometry active: {self.p['odom_topic']} and "
            f"{self.p['odom_frame']}->{self.p['base_frame']}; "
            f"counts_per_meter={self.p['counts_per_meter']}, "
            f"steering_feedback_required="
            f"{bool(self.p['steering_feedback_required'])}, "
            f"direction={direction_mode}, "
            f"publish_tf={bool(self.p['publish_tf'])}")

    def _on_steering(self, msg: Float32) -> None:
        self.integrator.set_steering_deg(msg.data)

    def _on_steering_valid(self, msg: Bool) -> None:
        was_valid = self.integrator.steering_valid
        self.integrator.set_steering_valid(msg.data)
        if was_valid and not self.integrator.steering_valid:
            self.get_logger().error(
                "STEERING_FEEDBACK_INVALID: odom yaw integration is using "
                "zero curvature until feedback recovers")

    def _on_drive(self, msg: Float32) -> None:
        self.integrator.set_applied_drive(msg.data)

    def _on_encoder(self, msg: Int32) -> None:
        now = self.get_clock().now()
        result = self.integrator.update_encoder(
            msg.data, now.nanoseconds * 1e-9)
        if not result.accepted:
            self.get_logger().warning(
                f"encoder sample ignored and baseline reset: {result.reason}")

        stamp = now.to_msg()
        odom = make_odometry(
            stamp, self.integrator, result,
            self.p["odom_frame"], self.p["base_frame"])
        self.odom_pub.publish(odom)
        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(make_transform(
                stamp, self.integrator,
                self.p["odom_frame"], self.p["base_frame"]))


def main(args=None):
    rclpy.init(args=args)
    node = McuOdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
