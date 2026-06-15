import math
import numpy as np
from transforms3d import euler
import transforms3d.axangles as axangles

def get_mat_log(R):
    """Get the log(R) of the rotation matrix R.

    Args:
        R (3x3 numpy array): rotation matrix
    Returns:
        w (3, numpy array): log(R)
    """
    theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    w_hat = (R - R.T) * theta / (2 * np.sin(theta) + 1e-9)  # Skew symmetric matrix
    w = np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]])  # [w1, w2, w3]

    return w

def rotm2quat(R):
    """Get the quaternion from rotation matrix.

    Args:
        R (3x3 numpy array): rotation matrix
    Return:
        q (4, numpy array): quaternion, x, y, z, w
    """
    w = get_mat_log(R)
    theta = np.linalg.norm(w)

    if theta < 0.001:
        q = np.array([0, 0, 0, 1])
        return q

    axis = w / theta

    q = np.sin(theta / 2) * axis
    q = np.r_[q, np.cos(theta / 2)]

    return q

def quat2rotm(quat):
    """Quaternion to rotation matrix.

    Args:
        quat (4, numpy array): quaternion x, y, z, w
    Returns:
        rotm (3x3 numpy array): rotation matrix
    """
    w = quat[3]
    x = quat[0]
    y = quat[1]
    z = quat[2]

    s = w * w + x * x + y * y + z * z

    rotm = np.array(
        [
            [
                1 - 2 * (y * y + z * z) / s,
                2 * (x * y - z * w) / s,
                2 * (x * z + y * w) / s,
            ],
            [
                2 * (x * y + z * w) / s,
                1 - 2 * (x * x + z * z) / s,
                2 * (y * z - x * w) / s,
            ],
            [
                2 * (x * z - y * w) / s,
                2 * (y * z + x * w) / s,
                1 - 2 * (x * x + y * y) / s,
            ],
        ]
    )

    return rotm

def rpy2rotm(rpy):
    return euler.euler2mat(rpy[0], rpy[1], rpy[2], axes='sxyz')#绕坐标系自身ZYX顺序转动,Rz*Ry*Rx,,SO3(0.1,z)*SO3(0.2,y)*SO3(0.3,x) = rpy2rotm([0.3,0.2,0.1])

def rotm2rpy(rotm):
    return euler.mat2euler(rotm) 

def rotm2axis_angle(rotm):
    axis, angle = axangles.mat2axangle(rotm)
    return axis* angle      # 旋转向量 (方向=轴, 长度=角度)

def rotm2axis_angle_separate(rotm):
    return axangles.mat2axangle(rotm)

def skew_symmetric(axis):
    mat = np.zeros((3, 3))
    mat[0, 1] = -axis[2]
    mat[0, 2] = axis[1]
    mat[1, 0] = axis[2]
    mat[1, 2] = -axis[0]
    mat[2, 0] = -axis[1]
    mat[2, 1] = axis[0]
    return mat

def axis_angle2rotm(angle, axis):
    # Normalize axis
    axis = axis / np.linalg.norm(axis)
    axis_hat = skew_symmetric(axis)
    # Rodrigues' formula
    R = np.eye(3) + axis_hat * np.sin(angle) + (1 - np.cos(angle)) * axis_hat @ axis_hat
    return R
