# UAV Collision Detection Project

This project simulates UAVs in a 3D environment and provides tools for creating and visualizing various bounding volumes and collision detection.

## Installation

### Prerequisites

- Python 3.10.13
- Conda (optional, but recommended for managing virtual environments)

### Setup

1. **Clone the Repository or Download the Project**

   - **Clone the repository (temporarily private)** to your local machine:
     ```bash
     git clone https://github.com/ultrongr/UAV
     cd UAV
     ```
   - **Download the project** as a ZIP file and extract it:
     ```bash
     unzip UAV.zip -d UAV
     cd UAV
     ```

2. **Create a Virtual Environment (Recommended Conda)**

   Create a virtual environment to manage your dependencies:
   ```bash
   conda create --name uav_project python=3.10.13
   conda activate uav_project
   ```

3. **Install Dependencies**

   Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```



## Usage

1. **Run the Project**

   Run the project:
   ```bash
   python main.py
   ```
    After you run the project, there will be several questions asked in the terminal, regarding the question number you want to run, as well as several other parameters. The default values are set to the values used in the report. You can change them if you want to test different scenarios.
    After you answer the questions, the project will run and display the 3D environment with the UAVs.
    

    By default, the times for collision detection, kdop generation and decision making are not printed to the console. 
    This above  behaviour can be changed via setting the global variable `show_times = True`.

2. **Interact with the Environment**

    - **Create and show the Axis Aligned Bounding Box (AABB) for the UAVs**
        - Press `B` to create and show or hide the AABB for all UAVs.

    - **Create and show the Unit Spheres for the UAVs**
        - Press `U` to create and show or hide the Unit Spheres for all UAVs.
    
    - **Create and show the Convex Hull for the UAVs**
        - Press `C` to create and show or hide the Convex Hull for all UAVs.

    - **Create and show the Kdop for the UAVs**
        - Press `K` to create and show or hide the Kdop for all UAVs (14Dop unless changed to 6).

    - **Detect and show the Collisions in the Airspace**
        - Press `L` to find momentary collisions in the airspace using the collision detection hierarchy defined in UAV.collides(). The first time this is used, the necessary bounding volumes will be generated but not shown (unless already visible).
        The collision detection method used for the visualization is the last collision detection method in the hierarchy (currently Kdop).
    
    - **Show the collisions of the continuous Kdops that are avoided**
        - Press `T` to show (and hide) the collisions of the continuous Kdops that are avoided. Keep in mind that the UAVs avoid these collisions on the same frame that they are detected, so they might be outside the continuous Kdop when the shape is shown.

    - **Move a specific UAV**
        - Use the arrow keys, space (up) and backspace (down) to move the UAV named v22_osprey_0 in the environment.
        - You can also rotate the UAV using the `R` Key.




## Communication

- If you **need help**, you can contact me at up1083865@ac.upatras.gr