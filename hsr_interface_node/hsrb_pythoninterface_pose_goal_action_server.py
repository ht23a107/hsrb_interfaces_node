import math
import rclpy
from rclpy.node import Node
import hsrb_interface
from hsrb_interface import geometry
from tf_transformations import euler_from_quaternion

from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import HsrbPythonInterfaceStringCommand
from airobot_interfaces.action import TestString

import threading
from threading import Thread, Lock
from rclpy.callback_groups import ReentrantCallbackGroup

import time

class HsrbPythonInterfacePoseGoalActionServer(Node):
    def __init__(self):
        super().__init__('hsrb_pythoninterface_pose_goal_action_server')
        self.goal_handle = None  # 処理中のゴールの情報を保持する変数
        self.goal_lock = Lock()  # 二重実行させないためのロック変数
        self.execute_lock = Lock()  # 二重実行させないためのロック変数
        # callback group をインスタンス化
        self.callback_group = ReentrantCallbackGroup()

        # 他のノードからの指令を受け付けるアクションサーバ
        self.action_server = ActionServer(
            self,
            HsrbPythonInterfaceStringCommand,
            'command',
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=self.callback_group
        )
        self.get_logger().info('アクションサーバー起動')

    def cancel_callback(self, goal_handle):
        self.get_logger().info('キャンセル受信')
        return CancelResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle):
        
        # 受理時はゴールの実行を新スレッドで起動して受理処理をブロックしない
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('前のゴールを中止')
                try:
                    self.goal_handle.abort()
                except Exception:
                    pass
            self.goal_handle = goal_handle
        # execute() を直接呼ぶ代わりにスレッドで実行
        t = Thread(target=lambda: goal_handle.execute())
        t.daemon = True
        t.start()

        '''
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('前のゴールを中止')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()
        '''
    

    def execute_callback(self, goal_handle):
        with self.execute_lock:  # 二重実行防止
            self.get_logger().info('目標受信:')
            feedback = HsrbPythonInterfaceStringCommand.Feedback()
            result = HsrbPythonInterfaceStringCommand.Result()

            goal=goal_handle.request
            position = [goal.x, goal.y, goal.z]
            quat = [goal.qx, goal.qy, goal.qz, goal.qw]

            # クォータニオンをオイラー角に変換
            self.roll, self.pitch, self.yaw = euler_from_quaternion(quat)

            with hsrb_interface.Robot() as robot:
                # APIを使用するための準備
                #obot=hsrb_interface.Robot()
                whole_body = robot.get('whole_body')
                collision_world = robot.get("global_collision_world")
                gripper = robot.get('gripper', robot.Items.END_EFFECTOR)
                wrist_wrench = robot.get('wrist_wrench', robot.Items.FORCE_TORQUE)

                # TODO 動かす前にインターフェイスのnutural()で初期姿勢に戻す、ちゃんと使うかどうかを選択できるようにする

                whole_body.collision_world = None  # 衝突回避を有効化
                #whole_body.looking_hand_constraint = True  # 動作後に手先を見続ける
                pose = geometry.pose(x=position[0], y=position[1], z=position[2], ei=self.roll, ej=self.pitch, ek=self.yaw)
                self.get_logger().info(f'目標位置: {position}, 目標姿勢: {quat}')

                if goal.force_torque is True:
                    time.sleep(1.0)
                    pre_force_torque = wrist_wrench._get_raw_wrench()
                    self.get_logger().info('把持前の軸の力[N]を取得')
                    pre_forces, pre_torques = pre_force_torque
                    '''
                    # ここからはテスト段階
                    pre_forces = [0.0, 0.0, 0.0]
                    for i in range(10):
                        pre_force_torque = wrist_wrench._get_raw_wrench()
                        pre_forces_temp, pre_torques = pre_force_torque

                        pre_forces[0] += pre_forces_temp[0]
                        pre_forces[1] += pre_forces_temp[1]
                        pre_forces[2] += pre_forces_temp[2]
                        time.sleep(0.5)

                    pre_forces = [x / 10 for x in pre_forces]
                    '''
                    

                MAX_RETRIES = 5  # リトライ回数
                for attempt in range(1, MAX_RETRIES + 1):
                    try:
                        whole_body.cancel_goal()
                        time.sleep(1.0)  # 安全のために追加
                        whole_body.move_end_effector_pose(pose, ref_frame_id='base_link')
                        whole_body.wait_goal()  # ブロッキング
                        self.get_logger().info(f"移動成功: {attempt}回目")
                        break  # 成功したらループを抜ける
                    except hsrb_interface.exceptions.FollowTrajectoryError as e:
                        self.get_logger().warn(f"移動失敗: {attempt}回目, エラー: {e}")
                        if attempt == MAX_RETRIES:
                            self.get_logger().error("最大リトライ回数に到達、移動中止")
                            goal_handle.abort()
                            result.answer = 'NG'
                        else:
                            self.get_logger().info("再試行します…")


                '''
                self.get_logger().info(f'目標位置: {position}, 目標姿勢: {quat}')
                whole_body.move_end_effector_pose(pose, ref_frame_id='base_link')

                #ブロッキング処理
                whole_body.wait_goal()
                self.get_logger().info('移動完了')
                #collision_world.remove_all(timeout=1.0)
                #self.get_logger().info('衝突オブジェクトを削除')
                '''

                # 力指定グリッパを使用する場合
                if goal.gripper is True:
                    gripper.apply_force(0.9)#, delicate=True)  # 1.0Nで把持
                    self.get_logger().info('グリッパ把持指令実行')
                
                # 物体は把持後に初期姿勢に戻るか
                if goal.neutral is True:
                    whole_body.move_to_neutral()  # 初期姿勢に戻る
                    self.get_logger().info('初期姿勢に戻る')
                    whole_body.wait_goal()  # ブロッキング


                # TODO　インターフェイスを使って6軸トルクのＺ軸の重さを取り出しアンサーに返す
                if goal.force_torque is True:
                    time.sleep(2.0)
                    post_force_torque = wrist_wrench._get_raw_wrench()
                    post_forces, post_torques = post_force_torque

                    '''
                    # ここからはテスト段階
                    post_forces = [0.0, 0.0, 0.0]
                    for i in range(10):
                        post_force_torque = wrist_wrench._get_raw_wrench()
                        post_forces_temp, pre_torques = post_force_torque

                        post_forces[0] += post_forces_temp[0]
                        post_forces[1] += post_forces_temp[1]
                        post_forces[2] += post_forces_temp[2]
                        time.sleep(0.5)

                    post_forces = [x / 10 for x in post_forces]
                    '''
                    

                    '''
                    #  重力軸（Z軸）の差分だけを見る
                    pre_fz = pre_forces[2]
                    post_fz = post_forces[2]
                    diff_fz = post_fz - pre_fz
                    self.get_logger().info(f'持つ前: {pre_fz}持ったあと：{post_fz}')

                    # 差が小さすぎる場合は「持ってない」と判定
                    if abs(diff_fz) < 0.1:   # 0.3N ≒ 30g (調整可)
                        weight = 0.0
                    else:
                        weight = round(diff_fz / 9.81 * 1000, 1)

                    result.force_torque_answer = [weight]
                    self.get_logger().info(f'推定重量: {weight} [g]')
                    '''
                    

                    
                    force_difference = self.compute_difference(pre_forces, post_forces)

                    weight = round(force_difference / 9.81 * 1000, 1)
                    force_torque_list=[weight]
                    result.force_torque_answer = force_torque_list
                    self.get_logger().info(f'持つ前: {pre_force_torque}持ったあと：{post_force_torque}')
                    self.get_logger().info(f'重さは{force_torque_list}')
                    

                # ロボットが動いている場合はキャンセル
                if whole_body.is_moving():
                    whole_body.cancel_goal()
                    self.get_logger().info('実行中のアクションをキャンセルしました')

                #robot.close()  # ロボットとの接続を切る

            goal_handle.succeed()
            result.answer = 'OK'
            return result
        
    
    def compute_difference(self, pre_data_list, post_data_list):
        if (len(pre_data_list) != len(post_data_list)):
            raise ValueError('Argument lists differ in length')
        # Calcurate square sum of difference
        square_sums = sum([math.pow(b - a, 2)
                        for (a, b) in zip(pre_data_list, post_data_list)])
        return math.sqrt(square_sums)
    


def main(args=None):
    rclpy.init(args=args)
    node = HsrbPythonInterfacePoseGoalActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    '''
    rclpy.init(args=args)
    node =  HsrbPythonInterfacePoseGoalActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
    '''

if __name__ == '__main__':
    main()
