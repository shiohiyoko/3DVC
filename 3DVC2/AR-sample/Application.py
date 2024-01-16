import numpy as np
import datetime
import cv2
from OpenGL.GL import *
import glfw

import GLWindow
import PoseEstimation as ps
import USBCamera as cam

class Application:

    def __init__(self, title, width, height, deviceID, use_api):
        self.width   = width
        self.height  = height
        self.channel = 3
        
        self.camera = cam.USBCamera (deviceID, width, height, use_api)
        self.glwindow = GLWindow.GLWindow(title, width, height, self.display_func, self.keyboard_func)

        self.focus = 700.0
        self.u0    = width / 2.0
        self.v0    = height / 2.0

        scale = 0.01
        self.viewport_horizontal = self.u0 * scale
        self.viewport_vertical   = self.v0 * scale
        self.viewport_near       = self.focus * scale
        self.viewport_far        = self.viewport_near * 1.0e+6
        self.modelview           = (GLfloat * 16)()
        
        self.estimator = ps.PoseEstimation(self.focus, self.u0, self.v0)

        self.count = 0

        
    def display_func(self, window):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        success, self.image = self.camera.CaptureImage()
        if not success:
            return

        self.glwindow.draw_image(self.image)

        success = self.compute_camera_pose()
        success = True
        if success:
            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()            
            glFrustum(-self.viewport_horizontal, self.viewport_horizontal, -self.viewport_vertical, self.viewport_vertical, self.viewport_near, self.viewport_far)
            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()
            glLoadMatrixf(self.modelview)

            glScalef(self.model.scale, self.model.scale, self.model.scale)
            self.model.draw()

        glfw.swap_buffers(window)

    def keyboard_func(self, window, key, scancode, action, mods):
        if key == glfw.KEY_Q:
            glfw.set_window_should_close(self.glwindow.window, GL_TRUE)

        if action == glfw.PRESS and key == glfw.KEY_S:
            self.save_image(self.count)
            self.count += 1

    def save_image(self, count):
        filename = 'output_image-%05d.png' % count
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        glReadBuffer (GL_FRONT)
        glReadPixels(0, 0, self.width, self.height, GL_RGB, GL_UNSIGNED_BYTE, image.data)
        image = cv2.flip (image, -1)
        image = cv2.cvtColor (image, cv2.COLOR_BGR2RGB)
        cv2.imwrite(filename, image)

    #
    # カメラ姿勢を計算する関数
    #
    def compute_camera_pose(self):
        success, R, t = self.estimator.compute_camera_pose()
        if success:
            self.modelview[0] = R[0][0]
            self.modelview[1] = R[1][0]
            self.modelview[2] = R[2][0]
            self.modelview[3] = 0.0
            self.modelview[4] = R[0][1]
            self.modelview[5] = R[1][1]
            self.modelview[6] = R[2][1]
            self.modelview[7] = 0.0
            self.modelview[8] = R[0][2]
            self.modelview[9] = R[1][2]
            self.modelview[10] = R[2][2]
            self.modelview[11] = 0.0
            self.modelview[12] = t[0]
            self.modelview[13] = t[1]
            self.modelview[14] = t[2]
            self.modelview[15] = 1.0

        return success

    def set_mqo_model(self, model):
        self.model = model
