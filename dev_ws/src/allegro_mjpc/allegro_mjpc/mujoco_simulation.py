import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, MultiArrayDimension


def numpy_to_multiarray(arr: np.ndarray) -> Float64MultiArray:
    """Converts a numpy array to a Float64MultiArray with the right properties."""
    multiarray = Float64MultiArray()
    multiarray.layout.dim = [
        MultiArrayDimension(
            label=f"dim{i}",
            size=arr.shape[i],
            stride=int(np.prod(arr.shape[i:])),
        )
        for i in range(arr.ndim)
    ]
    multiarray.data = arr.flatten().tolist()
    return multiarray


class MujocoSimulator(Node):
    """Simulator for the cube environment.
    
    Subscribes to control actions and publishes state estimates.
    """

    def __init__(self):
        super().__init__('mujoco_simulator')

        # Create an internal system model
        # N.B. this path assumes we're running inside the docker container.
        model_path = 'mujoco_mpc/mjpc/tasks/allegro_cube/task.xml'
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Set up a viewer
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        # Set up a timer that steps the simulation forward at a fixed rate
        self.timer = self.create_timer(self.model.opt.timestep, self.step_simulation)

        # Set up a subscriber for control actions
        self.ctrl = np.zeros(self.model.nu)
        self.create_subscription(
            Float64MultiArray, 'allegro_cube_control', self.ctrl_callback, 1
        )

        # Set up a publisher for state estimates
        self.state_est_pub = self.create_publisher(
            Float64MultiArray, 'allegro_cube_state_estimate', 1
        )

    def ctrl_callback(self, msg: Float64MultiArray):
        """Callback for control actions."""
        self.ctrl = np.array(msg.data)

    def step_simulation(self):
        """Step the simulation forward.
        
        This is called periodically by the timer, reads the latest control
        action, and publishes state estimates.
        """
        # Step the simulation forward with the latest control action
        self.data.ctrl = self.ctrl
        mujoco.mj_step(self.model, self.data)

        # Sync mouse inputs/etc with the viewer
        self.viewer.sync()

        # Shut down the node if the viewer is closed
        if not self.viewer.is_running():
            self.get_logger().info("Viewer closed, shutting down")
            self.destroy_node()
            rclpy.shutdown()

        # Publish state estimates
        # Note: right now this includes the state of the "target" cube
        xhat = np.concatenate([self.data.qpos, self.data.qvel])
        self.state_est_pub.publish(numpy_to_multiarray(xhat))


def main(args=None):
    rclpy.init(args=args)

    simulator = MujocoSimulator()
    rclpy.spin(simulator)

    simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()