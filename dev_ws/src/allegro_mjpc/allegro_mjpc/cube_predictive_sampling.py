import mujoco
from mujoco_mpc import agent as agent_lib
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class PredictiveSampler(Node):
    """Predictive sampler for the cube environment.
    
    Subscribes to state estimates and publishes control actions.
    """

    def __init__(self):
        super().__init__('predictive_sampler')

        # [WIP]
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Set up an internal system model. The system path assumes we're
        # running inside the docker container.
        model_path = 'mujoco_mpc/mjpc/tasks/allegro_cube/task.xml'
        model = mujoco.MjModel.from_xml_path(model_path)

        # Set the number of rollouts for predictive sampling
        model.numeric("sampling_trajectories").data[0] = 10

        # Create the planning agent
        self.agent = agent_lib.Agent(task_id="AllegroCube", model=model)
        self.data = mujoco.MjData(model)

    def timer_callback(self):
        # [WIP]
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

        # Update self.data with the latest state estimates
        # TODO

        # Compute the next best action with predictive sampling
        self.agent.set_state(
            time=self.data.time,
            qpos=self.data.qpos,
            qvel=self.data.qvel,
            act=self.data.act,
            mocap_pos=self.data.mocap_pos,
            mocap_quat=self.data.mocap_quat,
            userdata=self.data.userdata,
        )
        self.agent.planner_step()
        u = self.agent.get_action()
        self.get_logger().info("u_shape: {}".format(u.shape))
        print(u)

        # Publish the action
        # TODO


def main(args=None):
    rclpy.init(args=args)

    controller = PredictiveSampler()
    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
