import copy
from typing import Dict, Optional, Tuple, Union

import actionlib
import dynamic_reconfigure.client
import mbf_msgs.msg as mbf_msgs
import numpy as np
import rospy
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Point, Pose2D, PoseStamped
from hsrlib.rosif import ROSInterfaces
from hsrlib.utils import description, joints, locations
from nav_msgs.msg import Path
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Empty as EmptySrv
from tam_hsr_nav.msg import MotionSynthesisAction, MotionSynthesisGoal
from tamlib.rosutils import Action, Publisher, Subscriber
from tamlib.tf import Transform, euler2quaternion, quaternion2euler
from tamlib.utils import Logger


class LibHSRNavigation(Publisher, Subscriber, Action, Logger):
    def __init__(self) -> None:
        self.node_name = rospy.get_name()
        Publisher.__init__(self)
        Subscriber.__init__(self)
        Action.__init__(self)
        Logger.__init__(self)

        self.tamtf = Transform()
        self.rosif = ROSInterfaces()
        self.rosif.pub.auto_setup()
        self.rosif.sub.auto_setup()
        self._description = description.load_robot_description()

        self._fields_xyz = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        move_base = "/move_base_flex"
        planner = f"{move_base}/teb_local_planner/TebLocalPlannerROS"

        # Parameter
        self.p_max_vel_x = rospy.get_param(f"{planner}/max_vel_x", -1)
        self.p_max_vel_y = rospy.get_param(f"{planner}/max_vel_y", -1)
        self.p_acc_lim_theta = rospy.get_param(f"{planner}/acc_lim_theta", -1)
        self.p_xy_goal_tolerance = rospy.get_param(f"{planner}/xy_goal_tolerance", -1)
        self.p_yaw_goal_tolerance = rospy.get_param(
            f"{planner}/yaw_goal_tolerance",
            -1,
        )
        self.p_inflation_ratio = rospy.get_param("~inflation_ratio", 1.3)

        # Publisher
        Publisher.register(self, "via_points", f"{planner}/via_points", Path)
        Publisher.register(self, "custom_obstacles", "/custom_obstacles", PointCloud2)

        # Subscriber
        self.exe_path_status = GoalStatusArray()
        Subscriber.register(
            self,
            "exe_path_status",
            f"{move_base}/exe_path/status",
            callback_func=self.subf_exe_path_status,
        )

        # Action Client
        _timeout = 300.0
        Action.client_register(
            self,
            "get_path",
            f"{move_base}/get_path",
            mbf_msgs.GetPathAction,
            timeout=_timeout,
        )
        Action.client_register(
            self,
            "exe_path",
            f"{move_base}/exe_path",
            mbf_msgs.ExePathAction,
            timeout=_timeout,
        )
        Action.client_register(
            self,
            "motion_synthesis",
            "/motion_synthesis",
            MotionSynthesisAction,
            timeout=_timeout,
        )

        # Service
        self.is_clear_costmaps = False
        self.sp_clear_costmaps = rospy.ServiceProxy(
            "/move_base_flex/clear_costmaps", EmptySrv
        )

        # Dynamic Reconfigure
        self.dr_global_costmap = dynamic_reconfigure.client.Client(
            f"{move_base}/global_costmap"
        )
        self.dr_global_costmap_obstacles = dynamic_reconfigure.client.Client(
            f"{move_base}/global_costmap/obstacles"
        )
        self.dr_global_costmap_inflater = dynamic_reconfigure.client.Client(
            f"{move_base}/global_costmap/inflater"
        )
        self.dr_local_costmap = dynamic_reconfigure.client.Client(
            f"{move_base}/local_costmap"
        )
        self.dr_local_costmap_inflater = dynamic_reconfigure.client.Client(
            f"{move_base}/local_costmap/inflater"
        )
        self.dr_teb_local_planner = dynamic_reconfigure.client.Client(
            f"{move_base}/teb_local_planner/TebLocalPlannerROS"
        )

    def __del__(self) -> None:
        self.cancel_exe_path()

    def _reset(self) -> None:
        self.exe_path_status = GoalStatusArray()

    def subf_exe_path_status(self, status) -> None:
        """ナビゲーションステータスのコールバック関数

        Args:
            status (GoalStatusArray): ナビゲーションステータス．
        """
        if len(status.status_list) <= 0:
            self.status = 0
            return

        if status.status_list[-1].status == 0:  # 地点に移動前
            pass
        elif status.status_list[-1].status == 1:  # 地点に移動中
            pass
        elif status.status_list[-1].status == 2:  # アクションキャンセルに成功
            self.status = 1
        elif status.status_list[-1].status == 3:  # 地点移動成功
            self.status = 1
        elif status.status_list[-1].status == 4:  # 地点移動失敗
            self.status = -1
        elif status.status_list[-1].status == 5:  # 地点を選択できず失敗
            self.status = -1
        elif status.status_list[-1].status == 6:  # アクションキャンセルに失敗
            pass
        elif status.status_list[-1].status == 7:  # アクションキャンセルに失敗
            pass
        elif status.status_list[-1].status == 8:  # アクションキャンセルに成功
            self.status = -1
        elif status.status_list[-1].status == 9:  # 地点移動失敗
            self.status = -1

    def enable_global_costmap(self, enable: bool) -> None:
        """グローバルコストマップのON/OFF制御

        Args:
            enable (bool): Trueの場合，ON．
        """
        self.dr_global_costmap_inflater.update_configuration({"enabled": enable})

    def enable_local_costmap(self, enable: bool) -> None:
        """ローカルコストマップのON/OFF制御

        Args:
            enable (bool): Trueの場合，ON．
        """
        self.dr_local_costmap_inflater.update_configuration({"enabled": enable})

    def enable_global_costmap_obstacles(self, enable: bool) -> None:
        """グローバルコストマップ上障害物のON/OFF制御

        Args:
            enable (bool): Trueの場合，ON．
        """
        self.dr_global_costmap_obstacles.update_configuration({"enabled": enable})

    def set_global_costmap_radius(self, robot: float, inflation: float) -> None:
        """グローバルコストマップの半径を変更する

        Args:
            robot (float): ロボットの半径 [m]．
            inflation (float): インフレーション半径 [m]（ロボット半径の1.2〜1.5倍程度）．
        """
        self.dr_global_costmap.update_configuration({"robot_radius": robot})
        self.dr_global_costmap_inflater.update_configuration(
            {"inflation_radius": inflation}
        )

    def set_local_costmap_radius(self, robot: float, inflation: float) -> None:
        """ローカルコストマップの半径を変更する

        Args:
            robot (float): ロボットの半径 [m]．
            inflation (float): インフレーション半径 [m]（ロボット半径の1.2〜1.5倍程度）．
        """
        self.dr_local_costmap.update_configuration({"robot_radius": robot})
        self.dr_local_costmap_inflater.update_configuration(
            {"inflation_radius": inflation}
        )

    def set_teb_local_planner_config(self, config: Dict[str, float]) -> None:
        """Teb Local Plannerのパラメータを変更する

        Args:
            config (Dict[str, float]): パラメータ情報．
        """
        self.dr_teb_local_planner.update_configuration(config)

    def set_max_velocity(
        self, x: float, y: float, backward: float, theta: float
    ) -> None:
        """最大速度を変更する

        Args:
            x (float): 前方向の最大速度 [m/s]．
            y (float): 横方向の最大速度 [m/s]．
            backward (float): 後方向の最大速度 [m/s]．
            theta (float): 最大回転速度 [m/s]．
        """
        self.set_teb_local_planner_config(
            {
                "max_vel_x": x,
                "max_vel_y": y,
                "max_vel_x_backwards": backward,
                "max_vel_theta": theta,
            }
        )

    def set_max_acceleration(self, x: float, y: float, theta: float) -> None:
        """最大加速度を変更する

        Args:
            x (float): 前方向の最大加速度 [m/s]．
            y (float): 横方向の最大加速度 [m/s]．
            theta (float): 最大回転加速度 [m/s]．
        """
        self.set_teb_local_planner_config(
            {
                "acc_lim_x": x,
                "acc_lim_y": y,
                "acc_lim_theta": theta,
            }
        )



    def pose2d_to_pose_stamped(self, pose2d: Pose2D, frame_id: str) -> PoseStamped:
        """Pose2DからPoseStampedに変換する

        Args:
            pose2d (Pose2D): _description_
            frame_id (str): _description_

        Returns:
            PoseStamped: _description_
        """
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = pose2d.x
        pose.pose.position.y = pose2d.y
        pose.pose.orientation = euler2quaternion(0, 0, pose2d.theta)
        return pose





    def _success(self, is_wait_motion_synthesis: bool) -> bool:
        """ナビゲーション成功時の処理

        Args:
            is_wait_motion_synthesis (bool): 動作合成の完了を待つかどうか．

        Returns:
            bool: 実行結果．
        """
        self._clear_costmaps()
        if is_wait_motion_synthesis:
            state = self.action.client.motion_synthesis.get_state()
            if state == GoalStatus.ACTIVE:
                self.action.client.motion_synthesis.cancel_goal()
            self.action.client.motion_synthesis.wait_for_result()

        self.logsuccess(f"[{self.node_name}]: Navigation SUCCESS")
        self._reset()
        return True

    def _failure(self) -> bool:
        """ナビゲーション失敗時の処理

        Returns:
            bool: 実行結果．
        """
        self._clear_costmaps()
        self.logfatal(f"[{self.node_name}]: Navigation FAILURE")
        self._reset()
        return False

    def _navigation(
        self,
        pose: PoseStamped,
        goal_tolerance: Optional[Tuple[float, float]] = None,
        via_points: Optional[Tuple[float]] = None,
        is_wait_motion_synthesis=True,
    ) -> bool:
        """ナビゲーションを実行する

        Args:
            pose (PoseStamped): ゴール座標．
            goal_tolerance (Optional[Tuple[float, float]], optional): ゴールの許容値（xy, yaw）.
                Noneの場合，アクション固有の許容値が用いられる．Defaults to None.
            via_points (Optional[Tuple[float]], optional): 中継点の座標リスト. Defaults to None.
            is_wait_motion_synthesis (bool, optional): 動作合成の完了を待つかどうか. Defaults to True.

        Returns:
            bool: 実行結果．
        """
        self.loginfo(
            f"[{self.node_name}]: Attempting to reach ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})",
        )

        # Get path
        # self.enable_global_costmap_obstacles(False)
        path_result = self.get_path(pose)
        if path_result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
            self.logfatal(
                f"[{self.node_name}]: Unable to complete plan: {path_result.message}"
            )
            return False
        # self.enable_global_costmap_obstacles(True)

        # Turn path (if deg > 90)
        # x, y, _ = locations.get_robot_position()
        # if not locations.is_in_location((x, y, 0), (0.5, 0.5, 6.28)):
        # if self._face_forward(path_result.path):
        #     path_result = self.get_path(pose)
        #     if path_result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
        #         self.logfatal(
        #             f"[{self.node_name}]: Unable to complete plan: {path_result.message}"
        #         )
        #         return False

        # Exe path
        self.status = 0
        is_tolerance = True if goal_tolerance is not None else False
        if is_tolerance:
            path_goal = self.create_path_goal(
                path_result.path, is_tolerance, goal_tolerance[0], goal_tolerance[1]
            )
        else:
            path_goal = self.create_path_goal(path_result.path, is_tolerance)
        self.exe_path(path_goal, sync=False)

        # ナビゲーション中
        loop_wait = rospy.Rate(10.0)
        via_point_clear_flag = False
        while not rospy.is_shutdown():
            # 中継地点の設定
            if via_point_clear_flag is False and via_points is not None:
                via_point_clear_flag = self._set_via_points(pose, via_points)

            # 止まっているかどうか
            # cmd_vel = self.rosif.sub.command_velocity(latest=False)
            # if cmd_vel is not False:
            #     cmd_vel_arr = np.array(
            #         [cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z]
            #     )
            #     stop_vel_arr = np.array([0.0, 0.0, 0.0])
            #     if not np.allclose(cmd_vel_arr, stop_vel_arr):
            #         continue

            # 終了処理
            if self.status == 1:
                return self._success(is_wait_motion_synthesis)
            elif self.status == -1:
                return self._failure()
            loop_wait.sleep()
        return self._failure()


    def _exec_motion_synthesis(
        self, pose: Pose2D, config: Dict[Union[str, float], Dict[str, float]]
    ):
        """動作合成を実行する

        Args:
            pose (Pose2D): ゴール座標．
            config (Dict[Union[str, float], Dict[str, float]]): 動作合成の各種設定．
        """
        goal = MotionSynthesisGoal()
        goal.goal_location = pose
        goal.apply_start_pose = False
        if "start_pose" in config.keys():
            goal.apply_start_pose = True
            if config["start_pose"] == "auto":
                start_pose = joints.get_go_pose()
                start_pose["head_tilt_joint"] = np.deg2rad(-30.0)
                goal.start_pose = joints.dict_to_hsr_joints(start_pose)
            else:
                goal.start_pose = joints.dict_to_hsr_joints(config["start_pose"])
        goal.apply_goal_pose = False
        if "goal_pose" in config.keys():
            goal.apply_goal_pose = True
            goal.goal_pose = joints.dict_to_hsr_joints(config["goal_pose"])
            if "exec_from_first" in config.keys():
                if config["exec_from_first"] == "all":
                    goal.exec_from_first = config["goal_pose"].keys()
                else:
                    goal.exec_from_first = config["exec_from_first"]
            if "goal_tolerance" in config.keys():
                goal.goal_tolerance = config["goal_tolerance"]
            else:
                goal.goal_tolerance = Point(0.5, 0.5, 0.0)
        self.action.client.motion_synthesis.send_goal(goal)
        return

    def go_abs(
        self,
        pose: Union[Pose2D, Tuple[Pose2D]],
        frame_id="map",
        goal_tolerance: Optional[Tuple[float, float]] = None,
        motion_synthesis_config={},
    ) -> bool:
        """`frame_id`座標系を基準にナビゲーションを実行する

        Args:
            pose (Union[Pose2D, Tuple[Pose2D]]): ゴール座標（複数指定可能）．
            frame_id (str, optional): 基準座標系. Defaults to "map".
            goal_tolerance (Optional[Tuple[float, float]], optional): ゴールの許容値（xy, yaw）.
                Noneの場合，アクション固有の許容値が用いられる．Defaults to None.
            use_viapoint (bool, optional): 複数の座標指定の場合，最後以外の座標を中継地点として設定するかどうか.
                Trueの場合，スムーズ（途中で止まらない）なナビゲーションが可能．Defaults to True.
            obstacles (list, optional): カスタム障害物リスト．3次元座標で指定する. Defaults to [].
            motion_synthesis_config (dict, optional): 動作合成の設定. Defaults to {}.

        Returns:
            bool: 実行結果．
        """
        if "is_wait" not in motion_synthesis_config.keys():
            motion_synthesis_config["is_wait"] = True


        if isinstance(pose, (tuple, list)):
            _pose = self.pose2d_to_pose_stamped(pose[-1], frame_id)
            self.tamtf.send_transform("goal", frame_id, _pose.pose)

            self._exec_motion_synthesis(pose[-1], motion_synthesis_config)

        #    return self._navigation(_pose, goal_tolerance, None, motion_synthesis_config["is_wait"])

