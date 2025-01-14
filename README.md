 README: Running Webots Controllers for Quadrotor Simulation

 Overview
This repository contains three Webots controllers to simulate and control a quadrotor's behavior. Additionally, it includes supporting files for PID parameter tuning and analysis. Below are the details of the controllers and how to use them.

---

Controllers
1. `my_controller`
   - This controller runs PID parameter tuning using optimization techniques.
   - It generates optimal PID values for throttle, yaw, and positional controls.
   - Outputs:
     - Tuned values stored in the terminal.
     - Optionally stores results in tables or plots.

2. `pacman_path_following`
   - This controller makes the quadrotor follow a Pac-Man-shaped path.
   - Includes altitude control, yaw stabilization, and position tracking for smooth path-following behavior.
   - Outputs:
     - The actual path followed by the quadrotor compared with the reference path.
     - Plots illustrating path-tracking performance.

3. `infinity_loop_path`
   - This controller makes the quadrotor follow an infinity-loop path (a figure-eight pattern).
   - Starts from a specific position, completes the infinity loop, and returns to the starting point.
   - Outputs:
     - Similar path-following performance metrics as the Pac-Man controller.

---
 **Folders and Files**

 jupyter/` Folder
1. `get_tuned_table.ipynb`
   - Generates a table with tuned PID parameters.
   - Uses simulation data to optimize and store values for reuse.

2. `solve_parameters_numerically.ipynb`
   - Provides numerical solutions to derive tuned PID parameters.
   - Useful for analytical PID tuning and understanding parameter behavior.

---

`Plots/` Folder
- Contains all plots generated during simulation and optimization, including:
  - Performance metrics over time.
  - Gain values for PID tuning.
  - Path-following performance comparisons.

---

How to Run Webots Controllers

1. Launch the Simulation
   - Open the `.wbt` simulation file in Webots.

2. Select a Controller
   - In the Webots interface, locate the **Scene Tree**.
   - Navigate to: `mavic2pro -> controller`.
   - Click on the controller field, and select one of the following from the drop-down menu:
     - `my_controller` (for PID tuning).
     - `pacman_path_following` (for following a Pac-Man path).
     - `infinity_loop_path` (for following an infinity loop path).

3. Run the Simulation
   - Press the play button in Webots to start the simulation.
   - Observe the quadrotor behavior and outputs in the terminal and plots.

---

Simulation Workflow
1. Start with `my_controller` to get tuned PID values.
2. Use the tuned values for `pacman_path_following` and `infinity_loop_path`.
3. Compare performance metrics and plots in the `Plots/` folder for analysis.

---

Troubleshooting
- Ensure the correct controller is selected before running the simulation.
- If the quadrotor behaves erratically, verify PID values and retune using `my_controller`.
- Use the `Plots/` folder to analyze performance and identify areas for improvement.

---

