#!/usr/bin/env python3
"""Launches the AprilTagNode composable-node container for the
isaac_ros_apriltag bridge (see memory: isaac_ros_apriltag_gpu).

Self-contained (does NOT depend on isaac_ros_examples' IsaacROSLaunchFragment
base class used by the package's own isaac_ros_apriltag_core.launch.py --
that package isn't installed on this machine and pulling it in just for a
thin launch-fragment abstraction wasn't worth it) -- builds the
ComposableNodeContainer directly instead.

RectifyNode is deliberately NOT included, as of 2026-08-05 -- confirmed by
reading isaac_ros_image_proc's actual rectify_node.cpp source (not
guessed): RectifyNode's image callback is typed to require a genuine
nvidia::isaac_ros::nitros::NitrosImage, with NO fallback path for a plain
sensor_msgs/Image publisher (it inherits plain rclcpp::Node, not a
NitrosNode base with negotiation/fallback logic -- unlike NITROS's general
backward-compatibility design, THIS node's C++ implementation doesn't
actually contain that fallback machinery). A real NITROS-producing
publisher would need to be written in C++ against NVIDIA's NITROS SDK --
out of scope for now. AprilTagNode's OWN image callback, by contrast, DOES
accept a plain sensor_msgs/Image directly (confirmed working end-to-end at
~29Hz real detections, tag IDs 20/2/5/21/4 all present, zero stalls).

Practical effect: images reaching AprilTagNode are NOT distortion-corrected
for this camera's ~120deg fisheye lens. Some corner-position error is
expected near the frame edges as a result. The existing homography fit
(apriltag_localize.py's TableCalibration, reused unchanged by
isaac_ros_apriltag_bridge.py) is itself an empirical per-position fit, not
a physics-based undistortion, so it may partially absorb this -- not yet
measured/validated in practice. Revisit real rectification (write a C++
NITROS bridge, or check for Python NITROS support in a newer Isaac ROS
release) as its own future task if accuracy proves insufficient.

Run alongside (these are plain rclpy nodes, not composable, so they run as
separate processes). If using a fixed crop (recommended, see
isaac_ros_image_publisher.py's module docstring -- --auto-crop's live
mid-stream resize crashes AprilTagNode's CUDA decoder), measure the crop
rect FIRST, before this launch file, so AprilTagNode only ever sees the
final cropped resolution from the moment it starts receiving frames:

    # one-time, before any of the below (re-run only if the camera moves):
    python3 fleet/measure_crop_rect.py --port 8765 --out fleet/crop_rect.json

    # terminal 1 (start this BEFORE terminal 2's crop command goes out):
    ros2 launch tools/isaac/isaac_apriltag_launch.py

    # terminal 2:
    python3 tools/isaac/isaac_image_publisher.py --port 8765 \\
        --crop-rect-file fleet/crop_rect.json

    # terminal 3:
    python3 tools/isaac/isaac_apriltag_bridge.py --rosbridge 192.168.0.212:9090

tile_size is the pixel window for ADAPTIVE THRESHOLDING (confirmed in
NVIDIA's Isaac ROS docs, not guessed) -- local image binarization before
edge/corner detection, so a genuine detection-robustness parameter and not
the GPU-parallelism knob it was first assumed to be. The package default of
4 is small relative to how large these tags appear in frame.

SWEPT ON HARDWARE 2026-09-29, full frame + --undistort, ~30s windows each.
Detection rate per tag, and frame rate:

                     tile=4           tile=8          tile=16
    fps (mean)    45.5 (41-50)    44.8 (40-48)    43.7 (38-49)
    Alvik1-5           100%            100%            100%
    Alvik6        70.1 (64-79)    92.8 (89-96)    94.9 (92-97)
    tag 20        98.1 (97-99)   98.7 (96-100)   98.5 (98-100)
    tag 21        96.1 (94-99)   99.1 (98-100)   98.9 (97-100)
    tag 22       94.0 (82-100)   98.3 (94-100)   93.7 (81-100)
    tag 23           2.5 (0-7)       1.6 (0-4)       2.0 (0-5)

**Default is now 8**, which takes the entire Alvik6 gain (+22.8 points) and
is also the best setting for tags 21 and 22, for 0.7 fps -- inside the
run-to-run spread. 16 adds only +2 more on Alvik6 while tag 22 falls back
to 93.7% and the fps spread widens; past 8 you are paying for nothing.

tile_size does NOT address tag 23, which is flat at 1.6-2.5% across the
whole sweep. That is a separate failure and not a tuning problem -- see
README.md.

Usage with a different tile_size:
    ros2 launch tools/isaac/isaac_apriltag_launch.py tile_size:=16
"""
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> launch.LaunchDescription:
    tile_size_arg = DeclareLaunchArgument(
        'tile_size', default_value='8',
        description='Adaptive-thresholding window size in pixels for '
                     'cuAprilTags detection (NVIDIA docs: "Tile/window '
                     'size used for adaptive thresholding in pixels"). '
                     '8, not the package default of 4: measured 2026-09-29 '
                     'to take Alvik6 from 70.1%% to 92.8%% detection for '
                     '0.7 fps. See the module docstring for the full sweep.')

    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        namespace='',
        parameters=[{
            # 3.835in ref tag, matching apriltag_localize.py's
            # DEFAULT_REF_TAG_SIZE_IN -- converted to meters here since this
            # node's 'size' param is in meters.
            'size': 3.835 * 0.0254,
            'max_tags': 64,
            'tile_size': LaunchConfiguration('tile_size'),
            'tag_family': 'tag36h11',
            'backends': 'CUDA',
        }],
        remappings=[
            # No RectifyNode in this pipeline (see module docstring) --
            # AprilTagNode reads our plain publisher's raw image/camera_info
            # directly instead of the usual image_rect/camera_info_rect.
            ('image', 'image'),
            ('camera_info', 'camera_info'),
        ],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[apriltag_node],
        output='screen',
    )

    return launch.LaunchDescription([tile_size_arg, container])
