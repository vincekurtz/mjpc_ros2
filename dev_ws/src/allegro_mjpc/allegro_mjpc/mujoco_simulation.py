import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


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

        # Set up a timer that steps the simulation forward
        timer_period = self.model.opt.timestep
        self.timer = self.create_timer(timer_period, self.step_simulation)
        self.i = 0

    def step_simulation(self):
        """Step the simulation forward.
        
        This is called periodically by the timer, reads the latest control
        action, and publishes state estimates.
        """
        # Get the latest control action
        # TODO

        # Step the simulation forward
        mujoco.mj_step(self.model, self.data)

        # Sync mouse inputs/etc with the viewer
        self.viewer.sync()

        # Shut down the node if the viewer is closed
        if not self.viewer.is_running():
            self.get_logger().info("Viewer closed, shutting down")
            self.destroy_node()
            rclpy.shutdown()

        # Publish state estimates
        # TODO
        print("Stepped simulation: current time {}".format(self.data.time))


def main(args=None):
    rclpy.init(args=args)

    simulator = MujocoSimulator()
    rclpy.spin(simulator)

if __name__ == '__main__':
    main()