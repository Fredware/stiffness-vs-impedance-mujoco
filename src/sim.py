import time
import numpy as np
import mujoco
import mujoco.viewer

class HelloMujoco:
    def __init__(self, xml_path):
        """
        Initialize the MuJoCo simulation.
        
        Args:
            xml_path (str): Path to the MJCF xml file.
        """
        # 1. Load the Model (Static description of the robot/world)
        print(f"Loading model from {xml_path}...")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        
        # 2. Create the Data (Dynamic state: positions, velocities, forces)
        # 
        self.data = mujoco.MjData(self.model)
        
    def controller(self, model, data):
        """
        A simple PD controller to move the robot.
        This function is called inside the control loop.
        """
        # Example: Sinusoidal reference trajectory for Joint 1 and 2
        t = data.time
        target_q1 = 1.0 * np.sin(2 * t)
        target_q2 = 0.5 * np.cos(2 * t)
        
        # Simple P-control (Stiffness) logic
        # Kp * (target - current)
        kp = 50.0
        
        # Joint 1 Control
        data.ctrl[0] = -kp * (data.qpos[0] - target_q1) 
        # Note: In a real impedance controller, you would set torque directly
        # based on interaction forces, here we just show basic actuation.

    def run(self):
        """
        Launch the passive viewer and run the simulation loop.
        """
        print("Launching viewer...")
        
        # Use launch_passive to keep control of the python thread
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            start_time = time.time()
            
            while viewer.is_running():
                step_start = time.time()
                
                # --- Control Step ---
                # Apply control signals (torques/positions) before stepping physics
                # (You can implement your custom controller logic here)
                self.controller(self.model, self.data)
                
                # --- Physics Step ---
                mujoco.mj_step(self.model, self.data)
                
                # --- Visualization Sync ---
                # Syncs the viewer with the latest physics state
                viewer.sync()
                
                # Time keeping to match real-time (rudimentary)
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

if __name__ == "__main__":
    # If run directly, assume assets are relative to where command is run
    sim = HelloMujoco("assets/scene.xml")
    sim.run()