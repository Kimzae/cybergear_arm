#!/usr/bin/env python3
"""
사인파 기준 궤적 발행기 (테스트용)

- /arm/joint_states 를 한 번 읽어서 "현재 자세"를 중심(c)으로 잡고
- q_d(t) = c + A * r(t) * sin(w t)  를 /arm/reference 로 발행
- r(t) 는 처음 ramp 초 동안 0 -> 1 로 부드럽게 증가 (갑자기 튀지 않게)
- 속도·가속도도 해석적으로 계산해서 같이 보냄 (SMC 가 사용)

실행:
  ros2 run cybergear_arm sine_reference.py --ros-args -p amplitude:="[0.3, 0.3]" -p freq:=0.3
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class SineReference(Node):
    def __init__(self):
        super().__init__('sine_reference')
        self.A = list(self.declare_parameter('amplitude', [0.3, 0.3]).value)  # [rad]
        self.f = float(self.declare_parameter('freq', 0.3).value)             # [Hz]
        self.ramp = float(self.declare_parameter('ramp', 2.0).value)          # [s]
        rate = float(self.declare_parameter('rate_hz', 200.0).value)

        self.center = None
        self.t0 = None
        self.create_subscription(JointState, 'arm/joint_states', self.on_js, 10)
        self.pub = self.create_publisher(JointTrajectoryPoint, 'arm/reference', 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info('arm/joint_states 기다리는 중...')

    def on_js(self, msg):
        if self.center is None:
            self.center = list(msg.position)
            n = len(self.center)
            if len(self.A) != n:  # 관절 수와 다르면 0으로 채우거나 잘라냄
                self.get_logger().warn(f'amplitude 길이 {len(self.A)} != 관절 수 {n} -> 맞춤')
                self.A = (self.A + [0.0] * n)[:n]
            self.t0 = self.get_clock().now()
            self.get_logger().info(f'중심 자세 = {[round(c, 3) for c in self.center]}')

    def tick(self):
        if self.center is None:
            return
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        w = 2 * math.pi * self.f
        # 부드러운 시작: r = 0.5(1 - cos(pi t / T))
        if t < self.ramp:
            k = math.pi / self.ramp
            r, dr, ddr = 0.5 * (1 - math.cos(k * t)), 0.5 * k * math.sin(k * t), 0.5 * k * k * math.cos(k * t)
        else:
            r, dr, ddr = 1.0, 0.0, 0.0
        s, c = math.sin(w * t), math.cos(w * t)

        msg = JointTrajectoryPoint()
        for ci, a in zip(self.center, self.A):
            msg.positions.append(ci + a * r * s)
            msg.velocities.append(a * (dr * s + r * w * c))
            msg.accelerations.append(a * (ddr * s + 2 * dr * w * c - r * w * w * s))
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(SineReference())


if __name__ == '__main__':
    main()
