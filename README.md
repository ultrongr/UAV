# UAV Collision Detection Project

This project simulates UAVs in a 3D environment and provides tools for creating and visualizing various bounding volumes and collision detection.

## Installation

### Prerequisites

- Python 3.10.13
- Conda (optional, but recommended for managing virtual environments)

### Setup

1. **Clone the Repository or Download the Project**

   - **Clone the repository (soon)** clone it to your local machine (Temporarily private):
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
    The project should open a window.
    After loading the UAVs it should display the 3D environment with the UAVs.

    By default, the times for collision detection and kdop generation are printed to the console. 
    This above  behaviour can be changed via setting `show_times = False`.

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
        - Press `L` to find collisions in the airspace using the collision detection hierarchy defined in UAV.collides(). The first time this is used, the necessary bounding volumes will be generated but not shown (unless already visible).
        The collision detection method used for the visualization is the last collision detection method in the hierarchy (currently Kdop).

    - **Move a specific UAV**
        - Use the arrow keys, space (up) and backspace (down) to move the UAV named v22_osprey_0 in the environment.
        - You can also rotate the UAV using the `R` Key.




## Communication

- If you **need help**, you can contact me at [up1083865@ac.upatras.gr]