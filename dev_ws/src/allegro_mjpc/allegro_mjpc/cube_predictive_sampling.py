import mujoco
from mujoco_mpc import agent as agent_lib
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


class PredictiveSampler(Node):
    """Predictive sampler for the cube environment.
    
    Subscribes to state estimates and publishes control actions.
    """

    def __init__(self):
        super().__init__('predictive_sampler')

        # Set up an internal system model. The system path assumes we're
        # running inside the docker container.
        model_path = 'mujoco_mpc/mjpc/tasks/allegro_cube/task.xml'
        model = mujoco.MjModel.from_xml_path(model_path)

        # Set the number of rollouts for predictive sampling
        model.numeric("sampling_trajectories").data[0] = 10

        # Create the planning agent
        self.agent = agent_lib.Agent(task_id="AllegroCube", model=model)
        self.data = mujoco.MjData(model)

        # Create a subscriber for state estimates
        self.create_subscription(
            Float64MultiArray, 
            'allegro_cube_state_estimate', 
            self.state_est_callback, 
            10
        )

        # Create a publisher for control actions
        self.ctrl_pub = self.create_publisher(
            Float64MultiArray, 'allegro_cube_control', 10
        )

    def state_est_callback(self, msg: Float64MultiArray):
        """Callback for state estimates.

        Every time we get a new state estimate, we compute a new control action
        with predictive sampling and publish it.
        """
        # Update self.data with the latest state estimates
        xhat = np.array(msg.data)
        qpos = xhat[:self.data.qpos.shape[0]]
        qvel = xhat[self.data.qpos.shape[0]:]
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel

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

        # Publish the action
        self.ctrl_pub.publish(numpy_to_multiarray(u))


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
