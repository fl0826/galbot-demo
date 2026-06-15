import pinocchio as pin
import numpy as np
import re
np.int = int
from urdfpy import URDF

class ToolFK:
    def __init__(self,urdf_path):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

    def get_T_frame(self,q,frame_name):
        pin.forwardKinematics(self.model, self.data, q)

        out = []
        for i in range(len(frame_name)):
            ee_frame_id = self.model.getFrameId(frame_name[i])
            pin.updateFramePlacement(self.model, self.data, ee_frame_id)
            ee_pose = self.data.oMf[ee_frame_id]

            T = np.eye(4)
            T[:3, :3] = np.array(ee_pose.rotation)
            T[:3, 3] = np.array(ee_pose.translation)
            out.append(T)

        return out
    
class ToolFK_URDF:
    def __init__(self,urdf_path):
            
        output_path = urdf_path.replace(".urdf", "_changed.urdf")

        with open(urdf_path, "r") as f:
            text = f.read()

        text = re.sub(r"<visual[\s\S]*?</visual>", "", text)
        text = re.sub(r"<collision[\s\S]*?</collision>", "", text)

        with open(output_path, "w") as f:
            f.writelines(text)

        self.robot = URDF.load(output_path)

    def get_T_frame(self,q,frame_name):
        
        joints_list = [ "leg_joint1","leg_joint2","leg_joint3","leg_joint4","leg_joint5",
                        "head_joint1","head_joint2",
                        "left_arm_joint1","left_arm_joint2","left_arm_joint3","left_arm_joint4","left_arm_joint5","left_arm_joint6","left_arm_joint7",
                        "right_arm_joint1","right_arm_joint2","right_arm_joint3","right_arm_joint4","right_arm_joint5","right_arm_joint6","right_arm_joint7"]
        joints_dict = {}
        for i in range(5+2+7+7):
            joints_dict[joints_list[i]] = q[i]

        fk_results = self.robot.link_fk(joints_dict)
        out = []
        for i in range(len(frame_name)):
            out.append(fk_results[self.robot.link_map[frame_name[i]]])

        return out
        
if __name__ == "__main__":

    q = np.random.random(21)

    tool_fk = ToolFK("galbot_one_golf.urdf")
    tool_fk_urdf = ToolFK_URDF("galbot_one_golf.urdf")

    T = tool_fk.get_T_frame(q,["left_arm_end_effector_mount_link"])[0]
    T_urdf = tool_fk_urdf.get_T_frame(q,["left_arm_end_effector_mount_link"])[0]

    print(np.abs(T-T_urdf))
    print(np.max(np.abs(T-T_urdf)))
