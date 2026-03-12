#!/usr/bin/env python3
################################################################################
# Copyright 2022 FZI Research Center for Information Technology
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
################################################################################

# -----------------------------------------------------------------------------
# \file    converter.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2022/11/09
#
# -----------------------------------------------------------------------------

import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Joy


class converter(Node):
    """Convert Twist messages to WrenchStamped"""

    TRANSLATION_MODE = "translation"
    ROTATION_MODE = "rotation"
    COMBINED_MODE = "combined"
    def __init__(self):
        super().__init__("converter")

        self.twist_topic = self.declare_parameter("twist_topic", "my_twist").value
        self.joy_topic = self.declare_parameter("joy_topic", "/spacenav/joy").value
        self.wrench_topic = self.declare_parameter("wrench_topic", "my_wrench").value
        self.frame_id = self.declare_parameter("frame_id", "world").value
        self.translation_button_idx = int(
            self.declare_parameter("translation_button_idx", 0).value
        )
        self.rotation_button_idx = int(
            self.declare_parameter("rotation_button_idx", 1).value
        )
        self.command_timeout = float(self.declare_parameter("command_timeout", 0.25).value)
        self.publish_zero_on_disable = bool(
            self.declare_parameter("publish_zero_on_disable", True).value
        )
        period = 1.0 / self.declare_parameter("publishing_rate", 100).value
        self.timer = self.create_timer(period, self.publish)
        self.teleop_mode = None
        self.teleop_enabled = False
        self.zero_sent_while_disabled = False
        self.last_twist_time = None

        self.buffer = WrenchStamped()

        self.pub = self.create_publisher(WrenchStamped, self.wrench_topic, 3)
        self.sub = self.create_subscription(Twist, self.twist_topic, self.twist_cb, 1)
        self.joy_sub = self.create_subscription(Joy, self.joy_topic, self.joy_cb, 1)
        self.get_logger().info(
            "Hold button 0 for translation, button 1 for rotation, both for combined."
        )

    def _is_pressed(self, buttons, idx):
        return 0 <= idx < len(buttons) and buttons[idx] == 1

    def _clear_buffer(self):
        self.buffer.wrench.force.x = 0.0
        self.buffer.wrench.force.y = 0.0
        self.buffer.wrench.force.z = 0.0
        self.buffer.wrench.torque.x = 0.0
        self.buffer.wrench.torque.y = 0.0
        self.buffer.wrench.torque.z = 0.0

    def _stamp_buffer(self, now):
        self.buffer.header.stamp = now.to_msg()
        self.buffer.header.frame_id = self.frame_id

    def joy_cb(self, data):
        translation_pressed = self._is_pressed(data.buttons, self.translation_button_idx)
        rotation_pressed = self._is_pressed(data.buttons, self.rotation_button_idx)

        if translation_pressed and rotation_pressed:
            self.teleop_mode = self.COMBINED_MODE
            self.teleop_enabled = True
            self.zero_sent_while_disabled = False
            return
        if translation_pressed:
            self.teleop_mode = self.TRANSLATION_MODE
            self.teleop_enabled = True
            self.zero_sent_while_disabled = False
            return
        if rotation_pressed:
            self.teleop_mode = self.ROTATION_MODE
            self.teleop_enabled = True
            self.zero_sent_while_disabled = False
            return

        self.teleop_mode = None
        self.teleop_enabled = False
        self._clear_buffer()
        self.zero_sent_while_disabled = False

    def twist_cb(self, data):
        if not self.teleop_enabled or self.teleop_mode is None:
            return

        self.last_twist_time = self.get_clock().now()
        self._stamp_buffer(self.last_twist_time)

        if self.teleop_mode in (self.TRANSLATION_MODE, self.COMBINED_MODE):
            self.buffer.wrench.force.x = data.linear.x
            self.buffer.wrench.force.y = data.linear.y
            self.buffer.wrench.force.z = data.linear.z
        else:
            self.buffer.wrench.force.x = 0.0
            self.buffer.wrench.force.y = 0.0
            self.buffer.wrench.force.z = 0.0

        if self.teleop_mode in (self.ROTATION_MODE, self.COMBINED_MODE):
            self.buffer.wrench.torque.x = data.angular.x
            self.buffer.wrench.torque.y = data.angular.y
            self.buffer.wrench.torque.z = data.angular.z
        else:
            self.buffer.wrench.torque.x = 0.0
            self.buffer.wrench.torque.y = 0.0
            self.buffer.wrench.torque.z = 0.0

    def publish(self):
        now = self.get_clock().now()

        if not self.teleop_enabled:
            if self.publish_zero_on_disable and not self.zero_sent_while_disabled:
                self._clear_buffer()
                self._stamp_buffer(now)
                try:
                    self.pub.publish(self.buffer)
                except Exception:
                    pass
                self.zero_sent_while_disabled = True
            return

        timeout_ns = int(self.command_timeout * 1e9)
        if (
            self.last_twist_time is None
            or (now - self.last_twist_time).nanoseconds > timeout_ns
        ):
            # Drop stale commands when input updates stop unexpectedly.
            self._clear_buffer()
            self._stamp_buffer(now)
        try:
            self.pub.publish(self.buffer)
        except Exception:
            # Swallow 'publish() to closed topic' error.
            # This rarely happens on killing this node.
            pass


def main(args=None):
    rclpy.init(args=args)
    node = converter()
    rclpy.spin(node)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rclpy.shutdown()
        sys.exit(0)
    except Exception as e:
        print(e)
        sys.exit(1)
