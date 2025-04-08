from direct.showbase.ShowBase import ShowBase
from panda3d.core import loadPrcFileData, WindowProperties
import numpy as np

class UAVVisualizer(ShowBase):

    def __init__(self):
        super().__init__()

        # Load the UAV model
        self.vehicle = self.loader.loadModel("egg.obj", noCache=True)
        self.vehicle.reparentTo(self.render)
        self.vehicle.setScale(1)
        self.vehicle.setPos(0, 0, 5000)

        # Initialize a shared state variable
        self.latest_state = None

        # Disable the default camera controls
        self.disableMouse()

        # Initialize camera modes
        self.camera_modes = ['third_person', 'fpv', "follow"]
        self.current_camera_mode = 0

        # Camera setup: adjust as necessary
        self.cam.setPos(2000, -2000, 1000)
        self.cam.lookAt(self.vehicle)

        # Set up the initial camera view
        self.set_camera_view()

        # Accept keyboard inputs to switch camera views and control zoom
        self.accept('c', self.switch_camera_view)
        self.accept('wheel_up', self.zoom_in)
        self.accept('wheel_down', self.zoom_out)

        # Start the task loop that checks for state updates
        self.taskMgr.add(self.update_task, "UpdateTask")

    def update_task(self, task):
        if self.latest_state is not None:
            try:
                if not np.all(np.isfinite(self.latest_state)):
                    print("Warning: Non-finite values in state, skipping frame.")
                    self.latest_state = None
                    return task.cont

                x, y, z = self.latest_state[0:3]
                phi, theta, psi = self.latest_state[6:9]

                self.vehicle.setPos(x, y, -z)
                self.vehicle.setHpr(np.degrees(psi), np.degrees(theta), np.degrees(phi))
                self.latest_state = None

            except Exception as e:
                print(f"Error updating visualization: {e}")
                self.latest_state = None

        return task.cont

    def update_visualization(self, state):
        """External method to update the current state data."""
        self.latest_state = state
        self.taskMgr.step()  # Immediately force a frame redraw

    def set_camera_view(self):
        mode = self.camera_modes[self.current_camera_mode]
        if mode == 'third_person':
            self.camera.reparentTo(self.vehicle)
            self.camera.setPos(0, -10, 3)
            self.camera.lookAt(self.vehicle)
        elif mode == 'fpv':
            self.camera.reparentTo(self.vehicle)
            self.camera.setPos(0, 0, 1)
            self.camera.lookAt(self.vehicle.getPos() + self.vehicle.getQuat().getForward())

    def switch_camera_view(self):
        self.current_camera_mode = (self.current_camera_mode + 1) % len(self.camera_modes)
        self.set_camera_view()

    def zoom_in(self):
        current_fov = self.camLens.getFov()[0]
        new_fov = max(current_fov - 5, 30)
        self.camLens.setFov(new_fov)

    def zoom_out(self):
        current_fov = self.camLens.getFov()[0]
        new_fov = min(current_fov + 5, 90)
        self.camLens.setFov(new_fov)

    def handle_window_resize(self, window):
        if window != self.win:
            return
        new_size = self.win.getSize()
        print(f"Window resized to: {new_size}")
