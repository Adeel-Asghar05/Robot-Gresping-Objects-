import pybullet as p
import time

class CameraController:
    def __init__(self, target=[0, 0, 0], yaw=45, pitch=-30, distance=3.5):
        # Create sliders
        self.yaw_slider = p.addUserDebugParameter("Camera Yaw", -180, 180, yaw)
        self.pitch_slider = p.addUserDebugParameter("Camera Pitch", -89, 89, pitch)
        self.dist_slider = p.addUserDebugParameter("Camera Distance", 0.5, 10, distance)
        
        # self.target_x_slider = p.addUserDebugParameter("Target X", -5, 5, target[0])
        # self.target_y_slider = p.addUserDebugParameter("Target Y", -5, 5, target[1])
        # self.target_z_slider = p.addUserDebugParameter("Target Z", -5, 5, target[2])
        self.target=target
    
    def update(self):
        # Read slider values
        yaw = p.readUserDebugParameter(self.yaw_slider)
        pitch = p.readUserDebugParameter(self.pitch_slider)
        dist = p.readUserDebugParameter(self.dist_slider)
        
        # target_x = p.readUserDebugParameter(self.target_x_slider)
        # target_y = p.readUserDebugParameter(self.target_y_slider)
        # target_z = p.readUserDebugParameter(self.target_z_slider)
        
        # Update camera
        p.resetDebugVisualizerCamera(
            cameraDistance=dist,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=self.target
            # cameraTargetPosition=[target_x, target_y, target_z]
        )
