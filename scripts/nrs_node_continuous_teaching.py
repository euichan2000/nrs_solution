import rospy
from geometry_msgs.msg import Point
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
from filterpy.kalman import KalmanFilter


import numpy as np
import numpy as np
import rospy

class DirectionalOutlierFilter:
    def __init__(self, radius=0.03, max_consecutive_outliers=10):
        self.radius = radius
        self.last_point = None
        self.second_last_point = None
        self.last_valid_direction = None
        self.consecutive_outlier_count = 0
        self.max_consecutive_outliers = max_consecutive_outliers

    def filter(self, x, y, z):
        p_new = np.array([x, y, z], dtype=np.float32)

        # 초기화 단계: 그냥 통과
        if self.last_point is None:
            self.last_point = p_new
            rospy.loginfo("[OutlierFilter] First point accepted.")
            return p_new
        if self.second_last_point is None:
            self.second_last_point = self.last_point
            self.last_point = p_new
            rospy.loginfo("[OutlierFilter] Second point accepted.")
            return p_new

        dist = np.linalg.norm(p_new - self.last_point)

        if dist > self.radius:
            # 이상치로 판단됨
            direction = self.last_point - self.second_last_point
            norm = np.linalg.norm(direction)

            if norm == 0:
                # 방향이 없다면 마지막 유효 방향 사용
                if self.last_valid_direction is not None:
                    direction = self.last_valid_direction
                    rospy.logwarn("[OutlierFilter] Zero direction. Using last valid direction.")
                else:
                    rospy.logwarn("[OutlierFilter] No direction to project. Returning last point.")
                    return self.last_point
            else:
                direction = direction / norm
                self.last_valid_direction = direction

            corrected = self.last_point + direction * self.radius

            rospy.logwarn(f"[OutlierFilter] Outlier detected (dist={dist:.4f})")
            rospy.logwarn(f"    Raw new point: {p_new}")
            rospy.logwarn(f"    last_point: {self.last_point}")
            rospy.logwarn(f"    direction: {direction}")
            rospy.logwarn(f"    corrected point: {corrected}")

            self.second_last_point = self.last_point
            self.last_point = corrected.copy()

            self.consecutive_outlier_count += 1
            if self.consecutive_outlier_count >= self.max_consecutive_outliers:
                rospy.logerr("[OutlierFilter] Too many consecutive outliers! Resetting filter.")
                self.last_point = None
                self.second_last_point = None
                self.last_valid_direction = None
                self.consecutive_outlier_count = 0
                return p_new  # 강제로 초기화

            return corrected

        # 정상 값 처리
        self.second_last_point = self.last_point
        self.last_point = p_new
        self.consecutive_outlier_count = 0  # 정상 값이 들어왔으므로 리셋
        rospy.loginfo(f"[OutlierFilter] Valid point accepted: {p_new}")
        return p_new




class ContinuousTeachingNode:
    def __init__(self):
        rospy.init_node("nrs_node_continuous_teaching")

        self.save_hz = rospy.get_param("~save_hz", 30.0)
        self.save_interval = rospy.Duration(1.0 / self.save_hz)
        self.last_save_time = rospy.Time(0)

        self.latest_point = None
        self.last_saved_point = None
        self.file_closed = False

        #self.DOF = DirectionalOutlierFilter()

        rospy.Subscriber("hand_pointed_position", Point, self.point_callback, queue_size=1)

        self.output_path = "/home/nrs/catkin_ws/src/nrs_path/data/selected_waypoints.txt"
        self.file = open(self.output_path, "w")
        rospy.loginfo(f"[ContinuousTeaching] Writing waypoints to {self.output_path}")

        rospy.Service("/continuous_teaching_save", Empty, self.handle_save)
        rospy.Service("/continuous_teaching_end", Empty, self.handle_end)

    def point_callback(self, msg):
        # Kalman Filter 적용
        #fx, fy, fz = self.DOF.filter(msg.x, msg.y, msg.z)
        fx, fy, fz = msg.x, msg.y, msg.z
        self.latest_point = np.array([fx, fy, fz])

    def handle_save(self, req):
        now = rospy.Time.now()

        if now - self.last_save_time < self.save_interval:
            rospy.logdebug("[ContinuousTeaching] Skipping save: interval not elapsed")
            return EmptyResponse()

        if self.latest_point is None:
            rospy.logwarn("[ContinuousTeaching] No point to save.")
            return EmptyResponse()

        if self.last_saved_point is not None and np.allclose(self.latest_point, self.last_saved_point, atol=1e-6):
            rospy.logdebug(f"[ContinuousTeaching] Skipping duplicate point: {self.latest_point}")
            return EmptyResponse()

        line = f"{self.latest_point[0]:.6f} {self.latest_point[1]:.6f} {self.latest_point[2]+0.03:.6f}\n"
        self.file.write(line)
        self.file.flush()
        rospy.loginfo(f"[ContinuousTeaching] Saved waypoint: {line.strip()}")

        self.last_saved_point = self.latest_point.copy()
        self.last_save_time = now
        return EmptyResponse()

    def handle_end(self, req):
        if not self.file_closed:
            self.file.close()
            self.file_closed = True
            rospy.loginfo("[ContinuousTeaching] Closed file.")
        return EmptyResponse()

    def run(self):
        rospy.spin()
        if not self.file_closed:
            self.file.close()
            rospy.loginfo("[ContinuousTeaching] File closed on shutdown.")

if __name__ == '__main__':
    node = ContinuousTeachingNode()
    node.run()
