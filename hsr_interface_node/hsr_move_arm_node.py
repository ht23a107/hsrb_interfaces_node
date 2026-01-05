import math
import rclpy
from rclpy.node import Node
import hsrb_interface
from hsrb_interface import geometry
from tf_transformations import euler_from_quaternion

'''
HSRの公式PythonAPIを使用してWholeBodyを動かすノード
実行方法：
ros2 run hsr_interface_node hsr_move_arm_node --ros-args -p x:=1.0 -p y:=0.0 -p z:=0.7 -p qx:=0.707 -p qy:=0.0 -p qz:=0.707 -p qw:=0.0

おそらく？
ref_frame_id=''が'odom'なら絶対値移動'base_link'なら相対値移動
'''

# TODO ステート化したい
# TODO 最大速度と加速度を指定できるようにする
# TODO rcjo2025_wsのパッケージでの動作確認（実行に必要なリポジトリもHSR公式からクローンする）

class HsrMoveArmNode(Node):
    def __init__(self):
        super().__init__('hsr_move_arm_node')

        # パラメータ宣言
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('qx', 0.0)
        self.declare_parameter('qy', 0.0)
        self.declare_parameter('qz', 0.0)
        self.declare_parameter('qw', 1.0)

        # パラメータ取得
        self.x  = self.get_parameter('x').get_parameter_value().double_value
        self.y  = self.get_parameter('y').get_parameter_value().double_value
        self.z  = self.get_parameter('z').get_parameter_value().double_value
        self.qx = self.get_parameter('qx').get_parameter_value().double_value
        self.qy = self.get_parameter('qy').get_parameter_value().double_value
        self.qz = self.get_parameter('qz').get_parameter_value().double_value
        self.qw = self.get_parameter('qw').get_parameter_value().double_value

        self.Conversion()
        self.Move()

    
    def Conversion(self):
        # クォータニオンをオイラー角に変換
        quat = [self.qx, self.qy, self.qz, self.qw]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(quat)

    def Move(self):
        '''
        with hsrb_interface.Robot() as robot:
            whole_body = robot.get('whole_body')
            pose = geometry.pose(x=self.x, y=self.y, z=self.z, ei=self.roll, ej=self.pitch, ek=self.yaw)
            whole_body.move_end_effector_pose(pose, ref_frame_id='odom')
        '''
        robot=hsrb_interface.Robot()
        whole_body = robot.get('whole_body')
        pose = geometry.pose(x=self.x, y=self.y, z=self.z, ei=self.roll, ej=self.pitch, ek=self.yaw)
        whole_body.move_end_effector_pose(pose, ref_frame_id='base_link')


def main(args=None):
    rclpy.init(args=args)
    node = HsrMoveArmNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
