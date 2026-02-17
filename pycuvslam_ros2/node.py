#!/usr/bin/env python3
#
# PyCuVSLAM RGBD ROS2 Wrapper
# Subscribes to Orbbec Gemini 2L camera topics and runs cuVSLAM RGBD odometry.
#

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import message_filters
import cv2
import numpy as np

import cuvslam


class PyCuvSlamNode(Node):
    def __init__(self):
        super().__init__('pycuvslam_node')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'camera_link')
        self.declare_parameter('publish_tf', True)

        # Depth
        self.declare_parameter('depth_scale_factor', 1000.0)  # Reference script uses 1000.0
        self.declare_parameter('enable_depth_denoising', True)
        self.declare_parameter('fast_depth', True)

        # Tracker tuning
        self.declare_parameter('async_sba', True)
        self.declare_parameter('use_motion_model', True)
        self.declare_parameter('use_gpu', True)

        # Warmup
        self.declare_parameter('warmup_frames', 30)

        # Visualization
        self.declare_parameter('use_rerun', False)

        # ── State ───────────────────────────────────────────────────
        self.tracker = None
        self.initialized = False
        self.frame_count = 0
        self.last_img_ts = -1
        self.trajectory = []  # For visualization
        self.visualizer = None

        # ── Rerun Init ──────────────────────────────────────────────
        if self.get_parameter('use_rerun').value:
            try:
                from .visualizer import RerunVisualizer
                self.visualizer = RerunVisualizer(num_viz_cameras=1)
                self.get_logger().info("Rerun visualizer enabled")
            except ImportError as e:
                self.get_logger().error(f"Failed to import RerunVisualizer: {e}")
                self.visualizer = None

        # ── TF ──────────────────────────────────────────────────────
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Publishers ──────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '~/odom', 10)

        # ── CameraInfo subscriber (one-shot) ────────────────────────
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._camera_info_cb,
            10,
        )

        # ── Synced RGB + Depth subscribers ──────────────────────────
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.rgb_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('rgb_topic').value, qos_profile=img_qos
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('depth_topic').value, qos_profile=img_qos
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05
        )
        self.sync.registerCallback(self._image_cb)

        self.get_logger().info('PyCuVSLAM node started — waiting for CameraInfo…')

    # ────────────────────────────────────────────────────────────────
    # Image helpers
    # ────────────────────────────────────────────────────────────────
    @staticmethod
    def _ros_image_to_numpy(msg: Image, desired_encoding: str = 'passthrough') -> np.ndarray:
        """Convert a ROS Image to a numpy array, handling stride/padding."""
        if '16UC1' in msg.encoding:
            dtype, n_ch = np.uint16, 1
        elif '8UC3' in msg.encoding or 'rgb8' in msg.encoding or 'bgr8' in msg.encoding:
            dtype, n_ch = np.uint8, 3
        elif '8UC1' in msg.encoding or 'mono8' in msg.encoding:
            dtype, n_ch = np.uint8, 1
        else:
            dtype, n_ch = np.uint8, 1  # fallback

        expected_row_bytes = msg.width * n_ch * np.dtype(dtype).itemsize

        if msg.step != 0 and msg.step != expected_row_bytes:
            # padded rows – crop each row to expected width
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            raw = raw.reshape((msg.height, msg.step))
            raw = raw[:, :expected_row_bytes]
            img = raw.copy().view(dtype)
        else:
            img = np.frombuffer(msg.data, dtype=dtype)

        if n_ch > 1:
            img = img.reshape((msg.height, msg.width, n_ch))
        else:
            img = img.reshape((msg.height, msg.width))

        # Colour conversions
        if desired_encoding == 'mono8' and n_ch == 3:
            if 'rgb8' in msg.encoding:
                img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            elif 'bgr8' in msg.encoding:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        elif desired_encoding == 'bgr8' and n_ch == 3 and 'rgb8' in msg.encoding:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        return img

    def _enhance_depth(self, depth: np.ndarray) -> np.ndarray:
        """Optional median-filter denoising on the depth image."""
        if self.get_parameter('fast_depth').value:
            return cv2.medianBlur(depth.astype(np.uint16), 3)
        else:
            d = cv2.medianBlur(depth.astype(np.uint16), 5)
            d = cv2.GaussianBlur(d.astype(np.float32), (5, 5), 1.0).astype(np.uint16)
            return d

    # ────────────────────────────────────────────────────────────────
    # Tracker initialisation
    # ────────────────────────────────────────────────────────────────
    def _camera_info_cb(self, msg: CameraInfo):
        if self.initialized:
            return  # already built the tracker

        self.get_logger().info(
            f'CameraInfo received: {msg.width}x{msg.height}, '
            f'fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}'
        )

        # ── Distortion ──────────────────────────────────────────────
        if msg.distortion_model == 'plumb_bob' and len(msg.d) >= 5:
            dist = cuvslam.Distortion(
                cuvslam.Distortion.Model.Brown, list(msg.d[:5])
            )
            self.get_logger().info(f'Using Brown distortion: {list(msg.d[:5])}')
        else:
            dist = cuvslam.Distortion(cuvslam.Distortion.Model.Pinhole)
            self.get_logger().info('Using Pinhole (no distortion)')

        # ── Camera ──────────────────────────────────────────────────
        cam = cuvslam.Camera(
            size=[msg.width, msg.height],
            principal=[msg.k[2], msg.k[5]],       # cx, cy
            focal=[msg.k[0], msg.k[4]],            # fx, fy
            rig_from_camera=cuvslam.Pose(
                rotation=[0.0, 0.0, 0.0, 1.0],     # identity quaternion xyzw
                translation=[0.0, 0.0, 0.0],
            ),
            distortion=dist,
        )

        # ── Rig ─────────────────────────────────────────────────────
        rig = cuvslam.Rig(cameras=[cam])

        # ── RGBD settings ───────────────────────────────────────────
        rgbd_settings = cuvslam.Tracker.OdometryRGBDSettings(
            depth_scale_factor=self.get_parameter('depth_scale_factor').value,
            depth_camera_id=0,
            enable_depth_stereo_tracking=False,
        )

        # ── Odometry config ─────────────────────────────────────────
        odom_cfg = cuvslam.Tracker.OdometryConfig(
            odometry_mode=cuvslam.Tracker.OdometryMode.RGBD,
            rgbd_settings=rgbd_settings,
            use_gpu=self.get_parameter('use_gpu').value,
            async_sba=self.get_parameter('async_sba').value,
            use_motion_model=self.get_parameter('use_motion_model').value,
            use_denoising=False,
            enable_observations_export=self.get_parameter('use_rerun').value,
            enable_landmarks_export=False,
            enable_final_landmarks_export=False,
        )

        # ── Tracker ─────────────────────────────────────────────────
        self.tracker = cuvslam.Tracker(rig, odom_config=odom_cfg)
        self.initialized = True

        # Unsubscribe — we only needed one CameraInfo
        self.destroy_subscription(self.camera_info_sub)
        self.camera_info_sub = None

        self.get_logger().info('✅ cuVSLAM Tracker initialised (RGBD mode)')

    # ────────────────────────────────────────────────────────────────
    # Main tracking callback
    # ────────────────────────────────────────────────────────────────
    def _image_cb(self, rgb_msg: Image, depth_msg: Image):
        if not self.initialized or self.tracker is None:
            return

        # ── Convert images ──────────────────────────────────────────
        try:
            gray = self._ros_image_to_numpy(rgb_msg, 'mono8')
        except Exception as e:
            self.get_logger().warn(f'RGB conversion failed: {e}', throttle_duration_sec=2.0)
            return

        try:
            depth = self._ros_image_to_numpy(depth_msg, 'passthrough')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}', throttle_duration_sec=2.0)
            return

        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)

        # Sanity check
        if gray.shape[:2] != depth.shape[:2]:
            self.get_logger().warn(
                f'Size mismatch: rgb={gray.shape[:2]} depth={depth.shape[:2]}',
                throttle_duration_sec=2.0,
            )
            return
        
        # Debug depth data
        if self.frame_count % 30 == 0:
            d_min, d_max, d_mean = np.min(depth), np.max(depth), np.mean(depth)
            if d_max == 0:
                 self.get_logger().warn(f'Depth image is EMPTY (all zeros)!', throttle_duration_sec=2.0)
            else:
                 self.get_logger().info(f'Depth stats: min={d_min} max={d_max} mean={d_mean:.1f}', throttle_duration_sec=5.0)

        # ── Optional depth denoising ────────────────────────────────
        if self.get_parameter('enable_depth_denoising').value:
            depth = self._enhance_depth(depth)

        # ── Timestamp ───────────────────────────────────────────────
        ts_ns = int(rgb_msg.header.stamp.sec * 1_000_000_000 + rgb_msg.header.stamp.nanosec)
        if ts_ns <= self.last_img_ts:
            return  # duplicate / out-of-order
        self.last_img_ts = ts_ns

        self.frame_count += 1

        # ── Warmup ──────────────────────────────────────────────────
        warmup = self.get_parameter('warmup_frames').value
        if self.frame_count <= warmup:
            if self.frame_count % 10 == 0:
                self.get_logger().info(f'Warming up… {self.frame_count}/{warmup}')

        # ── Track ───────────────────────────────────────────────────
        try:
            pose_estimate, slam_pose = self.tracker.track(
                timestamp=ts_ns,
                images=[gray],
                depths=[depth],
            )
        except Exception as e:
            self.get_logger().error(f'track() error: {e}', throttle_duration_sec=2.0)
            return

        if pose_estimate is None or pose_estimate.world_from_rig is None:
            self.get_logger().warn('Tracking lost', throttle_duration_sec=2.0)
            return

        # Skip publishing during warmup
        if self.frame_count <= warmup:
            return

        # ── Extract pose ────────────────────────────────────────────
        pose_cov = pose_estimate.world_from_rig
        p = pose_cov.pose  # cuvslam.Pose

        # Log periodically
        if self.frame_count % 60 == 0:
            self.get_logger().info(
                f'[{self.frame_count}] pos=({p.translation[0]:.3f}, '
                f'{p.translation[1]:.3f}, {p.translation[2]:.3f})'
            )

        # ── Publish Odometry ────────────────────────────────────────
        odom_frame = self.get_parameter('odom_frame_id').value
        base_frame = self.get_parameter('base_frame_id').value

        odom = Odometry()
        odom.header.stamp = rgb_msg.header.stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame

        odom.pose.pose.position.x = float(p.translation[0])
        odom.pose.pose.position.y = float(p.translation[1])
        odom.pose.pose.position.z = float(p.translation[2])
        # cuvslam quaternion is [x, y, z, w]
        odom.pose.pose.orientation.x = float(p.rotation[0])
        odom.pose.pose.orientation.y = float(p.rotation[1])
        odom.pose.pose.orientation.z = float(p.rotation[2])
        odom.pose.pose.orientation.w = float(p.rotation[3])

        # Fill covariance from cuVSLAM if available
        try:
            cov = pose_cov.covariance  # 6×6 numpy
            if cov is not None:
                odom.pose.covariance = [float(v) for v in cov.flatten()]
        except Exception:
            pass  # leave zeros

        self.odom_pub.publish(odom)

        # ── Publish TF ──────────────────────────────────────────────
        if self.get_parameter('publish_tf').value:
            t = TransformStamped()
            t.header.stamp = rgb_msg.header.stamp
            t.header.frame_id = odom_frame
            t.child_frame_id = base_frame
            t.transform.translation.x = odom.pose.pose.position.x
            t.transform.translation.y = odom.pose.pose.position.y
            t.transform.translation.z = odom.pose.pose.position.z
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

        # ── Rerun Visualizer ────────────────────────────────────────
        if self.visualizer:
            try:
                # Accumulate trajectory (position only)
                pos = np.array(p.translation)
                self.trajectory.append(pos)
                
                # Get observations
                observations = []
                if self.get_parameter('use_rerun').value:
                     observations = self.tracker.get_last_observations(0)
                
                # Visualize
                # We only have one camera for visualization here (camera 0)
                self.visualizer.visualize_frame(
                    frame_id=self.frame_count,
                    images=[gray],
                    pose=p,
                    observations_main_cam=[observations],
                    trajectory=self.trajectory,
                    timestamp=ts_ns
                )
            except Exception as e:
                self.get_logger().warn(f"Visualizer error: {e}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = PyCuvSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()