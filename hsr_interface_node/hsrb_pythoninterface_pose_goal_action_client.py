import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from airobot_interfaces.action import HsrbPythonInterfaceStringCommand

'''
実行方法：ros2 run hsr_interface_node hsrb_pythoninterface_pose_goal_action_client --ros-args -p x:=1.0 -p y:=0.0 -p z:=0.7 -p qx:=0.707 -p qy:=0.0 -p qz:=0.707 -p qw:=0.0 -p command:=true -p force_torque:=true
'''

class HsrbPythonInterfacePoseGoalActionClient(Node):
    def __init__(self):
        super().__init__('hsrb_pythoninterface_pose_goal_action_client')

        # パラメータ宣言
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('qx', 0.0)
        self.declare_parameter('qy', 0.0)
        self.declare_parameter('qz', 0.0)
        self.declare_parameter('qw', 1.0)
        self.declare_parameter('command', True)
        self.declare_parameter('force_torque', True)

        # パラメータ取得
        self.x  = self.get_parameter('x').get_parameter_value().double_value
        self.y  = self.get_parameter('y').get_parameter_value().double_value
        self.z  = self.get_parameter('z').get_parameter_value().double_value
        self.qx = self.get_parameter('qx').get_parameter_value().double_value
        self.qy = self.get_parameter('qy').get_parameter_value().double_value
        self.qz = self.get_parameter('qz').get_parameter_value().double_value
        self.qw = self.get_parameter('qw').get_parameter_value().double_value
        self.command = self.get_parameter('command').get_parameter_value().bool_value
        self.force_torque = self.get_parameter('force_torque').get_parameter_value().bool_value

        self._action_client = ActionClient(self, HsrbPythonInterfaceStringCommand, 'command')

        self.send_goal(self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw, self.command, self.force_torque)

    def send_goal(self, x, y, z, qx, qy, qz, qw, command, force_torque):
        goal_msg = HsrbPythonInterfaceStringCommand.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.z = z
        goal_msg.qx = qx
        goal_msg.qy = qy
        goal_msg.qz = qz
        goal_msg.qw = qw
        goal_msg.gripper = command  # グリッパ把持コマンド
        goal_msg.force_torque = force_torque

        self.get_logger().info('アクションサーバ待機')
        self._action_client.wait_for_server()
        self.get_logger().info('目標を送信')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goalがサーバに拒否されました。')
            return

        self.goal_handle = goal_handle  # ゴールの情報を更新
        self.get_logger().info('結果を待機')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'結果: {result.answer}')
        self.get_logger().info(f'力：{result.force_torque_answer}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = HsrbPythonInterfacePoseGoalActionClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
