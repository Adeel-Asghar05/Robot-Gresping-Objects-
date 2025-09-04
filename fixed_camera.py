# # camera_sensor.py

# import pybullet as p
# import numpy as np

# class CameraSensor:
#     def __init__(self, width=640, height=480, fov=60, near=0.01, far=10.0, relative_to=None):
#         self.width = width
#         self.height = height
#         self.fov = fov
#         self.near = near
#         self.far = far
#         self.relative_to = relative_to

#         # Fixed camera offset relative to the robot base (replaces sliders)
#         self.offset_pos = [-0.316, -0.284, 0.211]

#         # Fixed camera orientation (roll, pitch, yaw) in radians
#         self.roll = 0.0
#         self.pitch = 0.0
#         self.yaw = -1.587

#     def get_images(self):
#         """
#         Returns (rgb_img, depth_img, seg_img).
#         Camera is fixed relative to `relative_to` using self.offset_pos and self.roll/pitch/yaw.
#         """
#         offset_ori = [self.roll, self.pitch, self.yaw]
#         cam_quat = p.getQuaternionFromEuler(offset_ori)

#         if self.relative_to is not None:
#             base_pos, base_ori = p.getBasePositionAndOrientation(self.relative_to)
#             cam_pos, cam_ori = p.multiplyTransforms(base_pos, base_ori, self.offset_pos, cam_quat)
#             cam_rot = p.getMatrixFromQuaternion(cam_ori)
#             forward = [cam_rot[0], cam_rot[3], cam_rot[6]]
#             up = [cam_rot[2], cam_rot[5], cam_rot[8]]
#             target = [cam_pos[0] + forward[0], cam_pos[1] + forward[1], cam_pos[2] + forward[2]]
#         else:
#             cam_pos = [1, 1, 1]
#             target = [0, 0, 0]
#             up = [0, 0, 1]

#         view_matrix = p.computeViewMatrix(cam_pos, target, up)
#         aspect = self.width / self.height
#         projection_matrix = p.computeProjectionMatrixFOV(self.fov, aspect, self.near, self.far)

#         _, _, rgb, depth, seg = p.getCameraImage(
#             self.width, self.height,
#             viewMatrix=view_matrix,
#             projectionMatrix=projection_matrix,
#             renderer=p.ER_BULLET_HARDWARE_OPENGL
#         )

#         rgb_img = np.reshape(rgb, (self.height, self.width, 4))[:, :, :3]
#         depth_img = np.reshape(depth, (self.height, self.width))
#         seg_img = np.reshape(seg, (self.height, self.width))

#         return rgb_img, depth_img, seg_img

# fixed_camera.py
import pybullet as p
import numpy as np

class CameraSensor:
    def __init__(self, width=640, height=480, fov=60, near=0.01, far=10.0, relative_to=None):
        self.width = width
        self.height = height
        self.fov = fov
        self.near = near
        self.far = far
        self.relative_to = relative_to

        # Fixed camera offset relative to the robot base (replaces sliders)
        self.offset_pos = [-0.316, -0.284, 0.211]

        # Fixed camera orientation (roll, pitch, yaw) in radians
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = -1.587

    def get_images(self):
        """
        Returns (rgb_img, depth_img, seg_img, view_matrix, projection_matrix).
        Camera is fixed relative to `relative_to` using self.offset_pos and self.roll/pitch/yaw.
        """
        offset_ori = [self.roll, self.pitch, self.yaw]
        cam_quat = p.getQuaternionFromEuler(offset_ori)

        if self.relative_to is not None:
            base_pos, base_ori = p.getBasePositionAndOrientation(self.relative_to)
            cam_pos, cam_ori = p.multiplyTransforms(base_pos, base_ori, self.offset_pos, cam_quat)
            cam_rot = p.getMatrixFromQuaternion(cam_ori)
            forward = [cam_rot[0], cam_rot[3], cam_rot[6]]
            up = [cam_rot[2], cam_rot[5], cam_rot[8]]
            target = [cam_pos[0] + forward[0], cam_pos[1] + forward[1], cam_pos[2] + forward[2]]
        else:
            cam_pos = [1, 1, 1]
            target = [0, 0, 0]
            up = [0, 0, 1]

        view_matrix = p.computeViewMatrix(cam_pos, target, up)
        aspect = self.width / self.height
        projection_matrix = p.computeProjectionMatrixFOV(self.fov, aspect, self.near, self.far)

        # Request image
        img = p.getCameraImage(
            self.width, self.height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        # The returned tuple: (width, height, rgbPixels, depthBuffer, segMaskBuffer)
        _, _, rgb, depth, seg = img

        rgb_img = np.reshape(rgb, (self.height, self.width, 4))[:, :, :3]
        depth_img = np.reshape(depth, (self.height, self.width))
        seg_img = np.reshape(seg, (self.height, self.width))

        return rgb_img, depth_img, seg_img, view_matrix, projection_matrix
