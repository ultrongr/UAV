import open3d as o3d
import open3d.geometry as o3dg # type: ignore
import open3d.visualization.gui as gui # type: ignore
import open3d.visualization.rendering as rendering # type: ignore
from open3d.visualization.gui import MouseEvent, KeyEvent # type: ignore
from open3d.visualization.rendering import Camera # type: ignore
import numpy as np
import time
import trimesh
from vvrpywork.constants import Key, Mouse, Color
from vvrpywork.scene import Scene2D, Scene3D, get_rotation_matrix, world_space
from vvrpywork.shapes import (
    NDArray, Point2D, Line2D, Triangle2D, Circle2D, Rectangle2D,
    PointSet2D, LineSet2D, Polygon2D,
    Point3D, Line3D, Arrow3D, Sphere3D, Cuboid3D, Cuboid3DGeneralized,
    PointSet3D, LineSet3D, Mesh3D, Triangle3D, ConvexPolygon3D, Polyhedron3D,
    AabbNode,
)

defaultUnlit = rendering.MaterialRecord()
defaultUnlit.shader = "defaultUnlit"

unlitLine = rendering.MaterialRecord()
unlitLine.shader = "unlitLine"
unlitLine.line_width = 5


config={
    "Q1": False,
    "Q2": False,
    "Q3": False,
    "Q4": False,
    "Q5": False,
    "Q6": False,
    "Q7": False,
    "Q8": False,
    "Q9": True,
}

def get_question():
    for key in config:
        if config[key]:
            return int(key[-1])

config_info={
    "Q1": "Load 3D UAV models and visualize them in random positions",
    "Q2": "For every model, visualize the axis-aligned bounding box (AABB), convex hull and k-DOP",
    "Q3": "Check for collisions between UAVs using bounding volumes (AABB, convex hull, k-DOP)",
    "Q4": "Check for collisions between UAVs using a more precise method (AABB node) and a most precise method (mesh collision check)",
    "Q5": "Randomize the position and the orientation of the UAVs and check for collisions",
    "Q6": "Assume dt, as the time step, and create a bounding volume that represents the positions of the UAVs during the time step and visualize collisions",
    "Q7": "Check for collisions between UAVs and create a protocol that avoids them",
    "Q8": "Implement a protocol for take off and one for landing with the number of drones, their appearance, position and speed, random",
    "Q9": "Visualize the simulation and show interesting statistics",




}


models = ["F52",
        "twin_copter",
        "Helicopter",
        "v22_osprey",
        
        
        # "quadcopter_scifi", ## Huge amount of vertices makes moving it time consuming
        
        # "fight_drone", ## small kdop issue
        # "B2_Spirit", ## small kdop issue
        
    ]
show_times = False
kdop_number = 14
speed_constant = 1.3
np.random.seed(123456)
# np.random.seed(12345)
# np.random.seed(1234)




class UAV:
    classes = {}

    def __init__(self, scene:Scene3D, filename:str, position:np.ndarray = None, scale:float = None):
        
        self.mesh = Mesh3D(filename)
        self.scene = scene

        self._class = filename.split("/")[-1].split(".")[0]
        if not self._class in UAV.classes:
            UAV.classes[self._class] = 0
        self.name = f"{self._class}_{UAV.classes[self._class]}"
        print(self.name, f"{len(self.mesh.vertices)} vertices, {len(self.mesh.triangles)} triangles")
        UAV.classes[self._class] += 1



        self.scene.addUAV(self)

        self.boxes = {
            "aabb": None,
            "chull": None,
            "sphere": None,
            "kdop": None,
            "aabb_node": None,
            "continuous": None,
        }
        self.boxes_visibility = {
            "aabb": False,
            "chull": False,
            "sphere": False,
            "kdop": False,
            "aabb_node": False,
        }

        self.position = np.mean(self.mesh.vertices, axis=0)
        self.direction = np.array([0, 0, -1])
        if "Helicopter" in filename: # Correcting incorrect rotation of mesh to align with direction
            self.direction = np.array([1, 0, 0])
        self.speed = 0.5*self.direction
        self.number_of_stationary_frames = 0
        self.target = None
        position = np.array(position)
        if position.any():            
            self.move_to(position)
        
        self.scale(scale, fit_to_unit_sphere= (not scale))

    def __eq__(self, other):
        """Check if two UAVs are equal"""
        return self.name == other.name


                
        
    def move_to(self, new_position:np.ndarray|list, update = True):
        "Move the UAV to the specified position"
        new_position = np.array(new_position)
        self.position=np.array(self.position)
        dist = new_position - self.position
        self.mesh.vertices += dist
        self.position = new_position
        if  update:
           
            self.mesh._update(self.name, self.scene)

        for box_name in self.boxes.keys(): # Move all the boxes as well
            if not self.boxes.get(box_name):
                continue
            
            box=self.boxes[box_name]
            if box_name == "kdop":
                
                box.move_by(dist)
                if self.boxes_visibility[box_name] and update:
                    self.remove_kdop()
                    self.create_kdop(kdop_number)
            
            elif box_name == "aabb":
                box.translate(dist)
                if self.boxes_visibility[box_name] and update:
                    self.remove_aabb()
                    self.create_aabb()
            
            elif box_name == "chull":
                for vertex in box._shape.vertices:
                    vertex += dist
                if self.boxes_visibility[box_name] and update:
                    self.remove_convex_hull()
                    self.create_convex_hull()
            
            elif box_name == "aabb_node" and update:
                self.boxes["aabb_node"] = None

                    
    
    def move_by(self, dist:np.ndarray, update = True):
        "Move the UAV by the specified distance"
        self.move_to(self.position + dist, update = update)

    def rotate(self, angle:float, axis:np.ndarray):
        "Rotate the UAV by the speciied angle around the specified axis"
        old_position = self.position
        self.move_by(-self.position)
        R = get_rotation_matrix(angle, axis)
        self.mesh.vertices = np.dot(self.mesh.vertices, R)

        self.move_by(old_position)

        self.mesh._update(self.name, self.scene)
        self.position = old_position
        self.direction = np.dot(self.direction, R)
        self.speed = 0.5*self.direction


        for box_name in self.boxes.keys(): # Rotate all the boxes as well
            if not self.boxes.get(box_name):
                continue
            box = self.boxes[box_name]

            if box_name == "aabb":
                
                self.boxes["aabb"]=None           

                if self.boxes_visibility[box_name]:
                    self.remove_aabb()
                    self.create_aabb()
            
            elif box_name == "kdop":
                box.rotate(R, self.position)
                if self.boxes_visibility[box_name]:
                    self.remove_kdop()
                    self.create_kdop(kdop_number)
            
            elif box_name == "chull":
                box.vertices-=self.position
                box.vertices = np.dot(box.vertices, R)
                box.vertices+=self.position
                if self.boxes_visibility[box_name]:
                    self.remove_convex_hull()
                    self.create_convex_hull()

    def collides_dt(self, other: "UAV", dt:float, show:bool = False):
        """Check if the UAV collides with another UAV at the specified time step
        This method uses a continuous kdop that represents all the kdop positions during a continuous
        movement of the UAVs. For reasons of efficiency, for most examples only the continuous kdop is used,
        however, an extra method has been implemented which -after the continuous kdop has detected a collision- 
        it checks for a collision in discrete time steps smaller than dt. The method used, as well as the number of 
        timesteps is defined below.
        
        Inputs:
        - other: the other UAV
        - dt: the time step
        - show: whether to show the collision boxes
        
        Returns:
        - True if the UAVs collide, False otherwise"""

        # Define what method you want to use after the kdop continuous has detected collision:
        # Possible choices: None, collides_aabb_node, collides_aabb, Collides_kdop, collides_convex_hull, collides_mesh_random, collides_mesh
        # extra_method = self.collides_aabb_node # Quite slower but more accurate
        extra_method = None # No extra method, makes it way faster
        extra_method = self.collides_aabb # Quite fast and accurate
        number_of_extra_checks = 2 # Number of extra checks in smaller time steps

        def get_continuous_kdop(uav: "UAV", dt:float): # Create a continuous kdop for the UAV, which is the chull of the kdop of the initial and final position
            
            if not uav.boxes["kdop"]:
                uav.create_kdop(kdop_number)
                uav.remove_kdop() # Hiding it since it wasnt visible before
            
            kdop = uav.boxes["kdop"]
            
            triangles = kdop._shape.triangles
            vertices = kdop._shape.vertices
            initial_mesh = o3d.geometry.TriangleMesh()
            initial_mesh.vertices = o3d.utility.Vector3dVector(vertices)
            initial_mesh.triangles = o3d.utility.Vector3iVector(triangles)

            vertices_new = vertices + dt*uav.speed
            triangles_new = np.array(triangles)
            new_mesh = o3d.geometry.TriangleMesh()
            new_mesh.vertices = o3d.utility.Vector3dVector(vertices_new)
            new_mesh.triangles = o3d.utility.Vector3iVector(triangles_new)

            mesh_combined = initial_mesh + new_mesh
            hull, _ = mesh_combined.compute_convex_hull()
            hull_mesh = Mesh3D(color=Color.BLUE)
            hull_mesh._shape.vertices=o3d.utility.Vector3dVector(hull.vertices)
            hull_mesh._shape.triangles=o3d.utility.Vector3iVector(hull.triangles)
            # print(len(hull.vertices), len(hull.triangles))
            return hull_mesh
        



        self.scene.removeShape(self.name+other.name+"_continuous")
        self.scene.removeShape(other.name+self.name+"_continuous")


        if np.linalg.norm(self.position - other.position)>2: # Dont bother with very distant UAVs
            if np.linalg.norm(self.position+dt*self.speed - other.position-dt*other.speed)>2:

                return False
        
        if not self.boxes.get("continuous"):
            self.boxes["continuous"] = get_continuous_kdop(self, dt)

        if not other.boxes.get("continuous"):
            other.boxes["continuous"] = get_continuous_kdop(other, dt)


        continuous1 = self.boxes["continuous"]
        continuous2 = other.boxes["continuous"]



        if check_mesh_collision_trimesh(continuous1._shape, continuous2._shape):
            if extra_method: # Check for collision in smaller time steps, only checks for discrete times

                output = False
                other_position = np.array(other.position)
                self_position = np.array(self.position)
                self_dx = self.speed*dt/number_of_extra_checks
                other_dx = other.speed*dt/number_of_extra_checks
                for i in range(1, number_of_extra_checks+1):
                    self.move_by(self_dx, update = False)
                    other.move_by(other_dx, update = False)
                    if extra_method(other, show=False):
                        output = True
                        break
                self.move_to(self_position, update = False)
                other.move_to(other_position, update = False)
                

            if extra_method:
                if output and show:
                    self.scene.addShape(continuous1, self.name+other.name+"_continuous")
                    self.scene.addShape(continuous2, other.name+self.name+"_continuous")
                return output
            else:
                if show:
                    self.scene.addShape(continuous1, self.name+other.name+"_continuous")
                    self.scene.addShape(continuous2, other.name+self.name+"_continuous")
                return True               



    def collides(self, other: "UAV", show:bool = False):
        """Check if the UAV collides with another UAV
        The collision check is done using several methods following a hierarchical order
        
        
        Inputs:
        - other: the other UAV
        - show: whether to show the collision boxes
        
        Returns:
        - True if the UAVs collide, False otherwise"""
        hierarchy = None
        if get_question()==3:
            hierarchy = [
                self.collides_aabb,
                self.collides_convex_hull,
                self.collides_kdop,
                # self.collides_convex_hull,
            ]
        
        elif get_question()==4:
            hierarchy = [
                self.collides_aabb,
                self.collides_kdop_trimesh,
                self.collides_aabb_node,
                self.collides_mesh_trimesh,
            ]

        elif get_question()==5:
            hierarchy = [
                self.collides_aabb,
                self.collides_aabb_node,
                self.collides_kdop_trimesh,
                self.collides_mesh_trimesh,
                self.collides_kdop,
            ]

        if not hierarchy: # Used for the later questions, emphasises on speed
            hierarchy = [
                self.collides_aabb,
                # self.collides_aabb_node,
                self.collides_kdop_trimesh,
                # self.collides_kdop,        
                # self.collides_convex_hull_trimesh,    
                # self.collides_convex_hull,
                # self.collides_mesh_random,
                # self.collides_mesh_trimesh,
                # self.collides_mesh,
            ]
        for i, method in enumerate(hierarchy):
            is_last = (i==len(hierarchy)-1) # Only the last method will show the collision boxes
            if method(other, show= (show and is_last)):
                continue
            else:
                return False
            
        return True

    def collides_aabb(self, other: "UAV", show:bool = False):
        """
        Check if the UAV collides with another UAV using axis-aligned bounding boxes
        Inputs:
        - other: the other UAV
        - show: whether to show the collision boxes
        
        Returns:
        - True if the UAVs collide, False otherwise"""


        if not self.boxes["aabb"]:
            self.create_aabb()
            self.remove_aabb() # Hiding it since it wasnt visible before
        if not other.boxes["aabb"]:
            other.create_aabb()
            other.remove_aabb() # Hiding it since it wasnt visible before
        dist = np.linalg.norm(self.position - other.position)
        if dist>2:
            return False
        
        xmin1 = self.boxes["aabb"].x_min
        xmax1 = self.boxes["aabb"].x_max
        ymin1 = self.boxes["aabb"].y_min
        ymax1 = self.boxes["aabb"].y_max
        zmin1 = self.boxes["aabb"].z_min
        zmax1 = self.boxes["aabb"].z_max
        
        xmin2 = other.boxes["aabb"].x_min
        xmax2 = other.boxes["aabb"].x_max
        ymin2 = other.boxes["aabb"].y_min
        ymax2 = other.boxes["aabb"].y_max
        zmin2 = other.boxes["aabb"].z_min
        zmax2 = other.boxes["aabb"].z_max



        if xmax1 < xmin2 or xmin1 > xmax2:
            return False
        if ymax1 < ymin2 or ymin1 > ymax2:
            return False
        if zmax1 < zmin2 or zmin1 > zmax2:
            return False

        if show:
            pair = f"{self.name}_{other.name}"
            collision_cuboid1 = Cuboid3D(p1=[xmin1, ymin1, zmin1], p2=[xmax1, ymax1, zmax1], color=Color.BLUE, width = 5, filled=False)
            collision_cuboid2 = Cuboid3D(p1=[xmin2, ymin2, zmin2], p2=[xmax2, ymax2, zmax2], color=Color.BLUE, width = 5, filled=False)

            self.scene.addShape(collision_cuboid1, pair + "_collision_1")
            self.scene.addShape(collision_cuboid2, pair + "_collision_2")
        return True

    def collides_kdop(self, other: "UAV", show:bool = False):
        """Check if the UAV collides with another UAV using k-dops
        Inputs:
        - other: the other UAV
        - show: whether to show the collision boxes
        
        Returns:
        - True if the UAVs collide, False otherwise"""
        if not self.boxes["kdop"]:
            self.create_kdop(kdop_number)
            self.remove_kdop() # Hiding it since it wasnt visible before
        if not other.boxes["kdop"]:
            other.create_kdop(kdop_number)
            other.remove_kdop() # Hiding it since it wasnt visible before
        
        dist = np.linalg.norm(self.position - other.position)
        if dist>2:
            return False
        
        kdop1_points = self.boxes["kdop"].vertices
        kdop2_points = other.boxes["kdop"].vertices
        if np.any(np.max(kdop1_points, axis=0) < np.min(kdop2_points, axis=0)) or np.any(np.min(kdop1_points, axis=0) > np.max(kdop2_points, axis=0)):
            return False
        
        return self.boxes["kdop"].collides_lines(other.boxes["kdop"], show=show, scene=self.scene)
    
    def collides_kdop_trimesh(self, other: "UAV", show:bool = False):
        """Check if the UAV collides with another UAV using k-dops, and the mesh collision check is done using trimesh
        Inputs:
        - other: the other UAV
        - show: whether to show the collision boxes (not used)
        
        Returns:
        - True if the UAVs collide, False otherwise"""
        if not self.boxes["kdop"]:
            self.create_kdop(kdop_number)
            self.remove_kdop()
        if not other.boxes["kdop"]:
            other.create_kdop(kdop_number)
            other.remove_kdop()
        dist = np.linalg.norm(self.position - other.position)
        if dist>2:
            return False
        
        kdop1 = self.boxes["kdop"]
        kdop2 = other.boxes["kdop"]

        return check_mesh_collision_trimesh(kdop1._shape, kdop2._shape)

    def collides_aabb_node(self, other: "UAV", show:bool = False):
        """Check if the UAV collides with another UAV using axis-aligned bounding boxes and aabb nodes
        Inputs:
        - other: the other UAV
        - show: whether to show the collision boxes
        
        Returns:
        - True if the UAVs collide, False otherwise"""

        if not self.boxes["aabb_node"]:
            self.create_aabb_node()
        if not other.boxes["aabb_node"]:
            other.create_aabb_node()
        pair = f"{self.name}+{other.name}:"
        self.scene.removeShape(pair+"aabb_node_collision_1")
        self.scene.removeShape(pair+"aabb_node_collision_2")
        dist = np.linalg.norm(self.position - other.position)
        if dist>2:
            return False
        
        return self.boxes["aabb_node"].collides(other.boxes["aabb_node"], show=show, scene=self.scene)
        

       

    def collides_mesh(self, other: "UAV", show:bool = False):
        """Check if the UAV collides with another UAV using the mesh collision check, this method is quite slow
        Inputs:
        - other: the other UAV
        - show: whether to show the collision boxes
        
        Returns:
        - True if the UAVs collide, False otherwise"""
        dist = np.linalg.norm(self.position - other.position)
        if dist>2:
            return False
        mesh1 = self.mesh
        mesh2 = other.mesh
        max_cords1 = np.max(mesh1.vertices, axis=0)
        min_cords1 = np.min(mesh1.vertices, axis=0)
        max_cords2 = np.max(mesh2.vertices, axis=0)
        min_cords2 = np.min(mesh2.vertices, axis=0)
        triangles_3d1 = []
        triangles_3d2 = []
        for i in range(len(mesh1.triangles)):
            triangle = mesh1.triangles[i]
            vertices = mesh1.vertices[triangle]
            if np.any(vertices > max_cords2) or np.any(vertices < min_cords2):
                continue
            triangles_3d1.append(Triangle3D(vertices[0], vertices[1], vertices[2]))
        
        for i in range(len(mesh2.triangles)):
            triangle = mesh2.triangles[i]
            vertices = mesh2.vertices[triangle]
            if np.any(vertices > max_cords1) or np.any(vertices < min_cords1):
                continue
            triangles_3d2.append(Triangle3D(vertices[0], vertices[1], vertices[2]))

        dist_cutoff = 0.1
        for t1 in range(len(triangles_3d1)):
            for t2 in range(len(triangles_3d2)):
                
                triangle1 = triangles_3d1[t1]
                triangle2 = triangles_3d2[t2]
                v1 = triangle1.getPoints()
                v2 = triangle2.getPoints()
                dist = np.linalg.norm(np.mean(v1, axis=0) - np.mean(v2, axis=0))
                if dist > dist_cutoff:
                    continue
                if triangle1.collides_triangle(triangle2):
                    if show:
                        self.scene.addShape(Point3D(np.mean(v1, axis=0), color=Color.BLUE, size=5), "collision_point1")
                    return True
    
        return False
    
    def collides_mesh_trimesh(self, other: "UAV", show:bool = False):
        """Check if the UAV collides with another UAV using the mesh collision check (using the trimesh mesh collision check), this method is quite slow
        Inputs:
        - other: the other UAV
        - show: whether to show the collision boxes (not used)
        
        Returns:
        - True if the UAVs collide, False otherwise"""

        if show:
            self.scene.removeShape(self.name+other.name+"_collision")
            self.scene.removeShape(other.name+self.name+"_collision")

        dist = np.linalg.norm(self.position - other.position)
        if dist>2:
            return False
        mesh1 = self.mesh
        mesh2 = other.mesh

        if check_mesh_collision_trimesh(mesh1, mesh2):
            if show:
                sphere = Sphere3D(p=np.mean(mesh1.vertices, axis=0), radius=0.3, resolution=20, color=Color.BLUE)
                self.scene.addShape(sphere, self.name+other.name+"_collision")
                sphere = Sphere3D(p=np.mean(mesh2.vertices, axis=0), radius=0.3, resolution=20, color=Color.BLUE)
                self.scene.addShape(sphere, other.name+self.name+"_collision")
                
            return True
    

    def collides_convex_hull(self, other: "UAV", show:bool = False):
        """Check if the UAV collides with another UAV using the convex hull collision check
        Inputs:
        - other: the other UAV
        - show: whether to show the collision boxes
        
        Returns:
        - True if the UAVs collide, False otherwise"""



        if not self.boxes["chull"]:
            self.create_convex_hull()
            self.remove_convex_hull() # Hiding it since it wasnt visible before
        if not other.boxes["chull"]:
            other.create_convex_hull()
            other.remove_convex_hull() # Hiding it since it wasnt visible before
        dist = np.linalg.norm(self.position - other.position)
        if dist>2:
            return False
        
        chull1 = self.boxes["chull"]
        chull2 = other.boxes["chull"]

        min1 = np.min(chull1.vertices, axis=0)
        max1 = np.max(chull1.vertices, axis=0)
        min2 = np.min(chull2.vertices, axis=0)
        max2 = np.max(chull2.vertices, axis=0)

        if np.any(max1 < min2) or np.any(min1 > max2):
            return False
        
        triangles_3d1 = []
        triangles_3d2 = []

        for i in range(len(chull1._shape.triangles)):
            triangle = chull1._shape.triangles[i]
            vertices = chull1.vertices[triangle]
            for i in range(3):
                if np.all(vertices[:, i]<min2[i]) or np.all(vertices[:, i]>max2[i]):
                    break
            else:
                triangles_3d1.append(Triangle3D(vertices[0], vertices[1], vertices[2]))
        for i in range(len(chull2._shape.triangles)):
            triangle = chull2._shape.triangles[i]
            vertices = chull2.vertices[triangle]
            for i in range(3):
                if np.all(vertices[:, i]<min1[i]) or np.all(vertices[:, i]>max1[i]):
                    break
            else:
                triangles_3d2.append(Triangle3D(vertices[0], vertices[1], vertices[2]))

        for t1 in range(len(triangles_3d1)):
            for t2 in range(len(triangles_3d2)):
                
                triangle1 = triangles_3d1[t1]
                triangle2 = triangles_3d2[t2]
                v1 = triangle1.getPoints()
                v2 = triangle2.getPoints()
                dist = np.linalg.norm(np.mean(v1, axis=0) - np.mean(v2, axis=0))
                if triangle1.collides_triangle(triangle2):
                    if show:
                        pair = f"{self.name}_{other.name}"

                        collision_triangle1 = Triangle3D(v1[0], v1[1], v1[2], color=Color.BLUE)
                        collision_triangle2 = Triangle3D(v2[0], v2[1], v2[2], color=Color.BLUE)
                        self.scene.addShape(collision_triangle1, pair + "_collision_triangle1_"+str(t1))
                        self.scene.addShape(collision_triangle2, pair + "_collision_triangle2_"+str(t2))
                        
                        # for j,v in enumerate((v1, v2)):
                        #     self.scene.addShape(Point3D(np.mean(v, axis=0), color=Color.BLUE, size=5), pair + "_collision_point1_"+str(j))
                        #     for i in range(3):
                        #         start = np.mean(v, axis=0)
                        #         end = start+0.5*np.array([(i==0), (i==1), (i==2)])
                        #         arrow = Arrow3D(start, end, color=Color.BLUE, width=1)
                        #         self.scene.addShape(arrow, pair + f"_collision_arrow1_{j}{i}")
                    return True

    def collides_convex_hull_trimesh(self, other: "UAV", show:bool = False):
        """Check if the UAV collides with another UAV using the convex hull collision check (using the trimesh mesh collision check)
        Inputs:
        - other: the other UAV
        - show: whether to show the collision boxes (not used)
        
        Returns:
        - True if the UAVs collide, False otherwise"""
        if not self.boxes["chull"]:
            self.create_convex_hull()
            self.remove_convex_hull() # Hiding it since it wasnt visible before
        if not other.boxes["chull"]:
            other.create_convex_hull()
            other.remove_convex_hull() # Hiding it since it wasnt visible before
        dist = np.linalg.norm(self.position - other.position)
        if dist>2:
            return False

        chull1 = self.boxes["chull"]
        chull2 = other.boxes["chull"]

        return check_mesh_collision_trimesh(chull1._shape, chull2._shape)

    def collides_mesh_random(self, other: "UAV", show:bool = False):
        """Check if the UAV collides with another UAV using by checking a random subset of triangles
        Inputs:
        - other: the other UAV
        - show: whether to show the collision boxes (not used)
        
        Returns:
        - True if the UAVs collide, False otherwise"""
        dist = np.linalg.norm(self.position - self.position)
        if dist>2:
            return False
        
        number_of_partitions: int = 40
        percentage=0.2
        step = int(1/percentage)


        mesh1 = self.mesh
        mesh2 = other.mesh

        space1 = np.ndarray((number_of_partitions+1, number_of_partitions+1, number_of_partitions+1), dtype=object)
        space2 = np.ndarray((number_of_partitions+1, number_of_partitions+1, number_of_partitions+1), dtype=object)
        for i in range(number_of_partitions+1):
            for j in range(number_of_partitions+1):
                for k in range(number_of_partitions+1):
                    space1[i, j, k] = []
                    space2[i, j, k] = []
        
        min1 = np.min(mesh1.vertices, axis=0)
        max1 = np.max(mesh1.vertices, axis=0)
        min2 = np.min(mesh2.vertices, axis=0)
        max2 = np.max(mesh2.vertices, axis=0)

        max_total = np.max([max1, max2], axis=0)
        min_total = np.min([min1, min2], axis=0)

        partition_size = (max_total - min_total) / number_of_partitions


        for i in range(0, len(mesh1.triangles), step):
            triangle = mesh1.triangles[i]
            vertices = mesh1.vertices[triangle]
            if np.any(vertices > max2) or np.any(vertices < min2):
                continue
            min_vertex = np.min(vertices, axis=0)
            max_vertex = np.max(vertices, axis=0)
            min_index = ((min_vertex - min_total) / partition_size).astype(int)
            max_index = ((max_vertex - min_total) / partition_size).astype(int)
            for x in range(min_index[0], max_index[0]+1):
                for y in range(min_index[1], max_index[1]+1):
                    for z in range(min_index[2], max_index[2]+1):
                        space1[x, y, z].append(Triangle3D(vertices[0], vertices[1], vertices[2]))
        
        for i in range(0, len(mesh2.triangles), step):
            triangle = mesh2.triangles[i]
            vertices = mesh2.vertices[triangle]
            if np.any(vertices > max1) or np.any(vertices < min1):
                continue
            min_vertex = np.min(vertices, axis=0)
            max_vertex = np.max(vertices, axis=0)
            min_index = ((min_vertex - min_total) / partition_size).astype(int)
            max_index = ((max_vertex - min_total) / partition_size).astype(int)
            for x in range(min_index[0], max_index[0]+1):
                for y in range(min_index[1], max_index[1]+1):
                    for z in range(min_index[2], max_index[2]+1):
                        space2[x, y, z].append(Triangle3D(vertices[0], vertices[1], vertices[2]))

        for i in range(number_of_partitions):
            for j in range(number_of_partitions):
                for k in range(number_of_partitions):
                    for t1 in space1[i, j, k]:
                        for t2 in space2[i, j, k]:
                            triangle1 = t1
                            triangle2 = t2
                            
                            if triangle1.collides_triangle(triangle2):
                                return True
        return False
    
    def has_reached_beacon(self, beacon, threshold:float = 0.1):
        """Check if the UAV has reached the specified beacon
        Inputs:
        - beacon: the beacon
        - threshold: the distance threshold
        
        Returns:
        - True if the UAV has reached the beacon, False otherwise"""
        dist = np.linalg.norm(self.position - np.array([beacon.x, beacon.y, beacon.z]))
        return dist < threshold

    def has_reached_landing_spot(self, landing_spot, threshold:float = 0.1):
        """Check if the UAV has reached the specified landing spot
        Inputs:
        - landing_spot: the landing spot
        - threshold: the distance threshold

        Returns:
        - True if the UAV has reached the landing spot, False otherwise"""
        dist = np.linalg.norm(self.position - np.array([landing_spot.x, landing_spot.y, landing_spot.z]))
        return dist < threshold

    def has_reached_target(self, threshold:float = 0.1):

        target_pos = np.array([self.target.x, self.target.y, self.target.z])
        dist = np.linalg.norm(self.position - target_pos)
        return dist < threshold


    def scale(self, scale:float = None, fit_to_unit_sphere:bool = False):
        """
        Scale the UAV by the specified factor.
        If fit_to_unit_sphere is True, the UAV is scaled such that it fits inside a unit sphere
        """

        if fit_to_unit_sphere:
            self.mesh.vertices -= self.position
            self.mesh.vertices /= np.max(np.linalg.norm(self.mesh.vertices, axis=1))
            self.mesh.vertices += self.position
            self.mesh._update(self.name, self.scene)
            return
        self.mesh.vertices *= scale
        self.mesh._update(self.name, self.scene)
    
    def create_sphere(self, radius:float, resolution:int):
        "Create a sphere around the UAV with the specified radius and resolution"
        if not radius:
            radius = np.max(np.linalg.norm(self.mesh.vertices - self.position, axis=1))
        sphere = Sphere3D(p=self.position, radius=radius, resolution=resolution)
        self.scene.addShape(sphere, self.name+"_sphere")
        self.boxes["sphere"] = sphere
        self.boxes_visibility["sphere"] = True
    
    def remove_sphere(self):
        self.scene.removeShape(self.name+"_sphere")
        self.boxes_visibility["sphere"] = False

    def create_aabb(self):
        "Create an axis-aligned bounding box around the UAV"

        if self.boxes["aabb"]:
            self.boxes_visibility["aabb"] = True
            self.scene.addShape(self.boxes["aabb"], self.name+"_aabb")
            return


        min_x, min_y, min_z = np.min(self.mesh.vertices, axis=0)
        max_x, max_y, max_z = np.max(self.mesh.vertices, axis=0)
        box = Cuboid3D(p1=[min_x, min_y, min_z], p2=[max_x, max_y, max_z], color=Color.GREEN, filled=False)
        self.scene.addShape(box, self.name+"_aabb")
        self.boxes["aabb"] = box
        self.boxes_visibility["aabb"] = True
    
    def remove_aabb(self):
        self.scene.removeShape(self.name+"_aabb")
        self.boxes_visibility["aabb"] = False

    def create_aabb_node(self):
        "Create an axis-aligned bounding box around the UAV"

        if self.boxes["aabb_node"]:
            return

        aabb_node = AabbNode(self.mesh.vertices, max_depth = 4, uav_name=self.name)
        self.boxes["aabb_node"] = aabb_node

    def create_convex_hull(self):
        "Create a convex hull around the UAV"

        if self.boxes["chull"]:
            self.boxes_visibility["chull"] = True
            self.scene.addShape(self.boxes["chull"], self.name+"_chull")
            return


        


        hull = []
        import time
        start = time.time()
        triangle_mesh = o3d.geometry.TriangleMesh()
        triangle_mesh.vertices = o3d.utility.Vector3dVector(self.mesh.vertices)
        triangle_mesh.triangles = o3d.utility.Vector3iVector(self.mesh.triangles)
        hull, _ = triangle_mesh.compute_convex_hull()
        hull_mesh = Mesh3D()
        hull_mesh._shape.vertices=o3d.utility.Vector3dVector(hull.vertices)
        hull_mesh._shape.triangles=o3d.utility.Vector3iVector(hull.triangles)
        self.scene.addShape(hull_mesh, self.name+"_chull")
        self.boxes["chull"] = hull_mesh
        self.boxes_visibility["chull"] = True

    def remove_convex_hull(self):
        self.scene.removeShape(self.name+"_chull")
        self.boxes_visibility["chull"] = False
    
    def create_kdop(self, k:int):
        "Create a k-dop around the UAV"

        if not self.boxes.get("kdop"):
            vertices = self.mesh.vertices
            kdop = Kdop(vertices, k, self.scene, self.name + "_kdop", self)
            self.boxes["kdop"] = kdop
            self.boxes_visibility["kdop"] = True
        else:
            self.boxes_visibility["kdop"] = True
            self.scene.addShape(self.boxes["kdop"], self.name+"_kdop")
            return

    def remove_kdop(self):
        if not self.boxes.get("kdop"):
            return
        kdop = self.boxes["kdop"]
        kdop_name = self.name+"_kdop"
        self.scene.removeShape(self.name+"_kdop")
        
        for i, face in enumerate(kdop._polygons):
            self.scene.removeShape(kdop_name+f"_face_{i}")
        
        self.boxes_visibility["kdop"] = False
        
        # self.boxes["kdop"] = None


class LandingPad:
    def __init__(self, N:int, scene:Scene3D) -> None:

        self.N=N
        self.scene = scene

        self.pads = []
        self.create()
    
    def create(self):
        colors = [Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.ORANGE, Color.MAGENTA]
        for i in range(self.N):
            pads= []
            for j in range(self.N):
                color = colors[(i+j)%len(colors)]
                pad = Cuboid3D(p1=[2*i, 0, 2*j], p2=[2*i+2, -0.1, 2*j+2], color=color, filled = True)
                self.scene.addShape(pad, f"landing_pad_{i}_{j}")
                pads.append(pad)
            self.pads.append(pads)
        self.pads = np.array(self.pads)

class Airspace(Scene3D):

    def __init__(self, width, height, N, dt, window_name="UAV"):
        super().__init__(width, height, window_name)
        self.uavs: dict[UAV] = {}
        self.N = N
        self.landing_pad = LandingPad(N, self)
        self.dt = dt
        self.last_update = time.time()
        self.paused = True

        self.beacons: dict[str] = {}
        self.landing_spots: dict[str] = {}
        self.mode:str = None # Can be avoidance/landing/target_beacon/landing_take_off
        self.new_uavs_flow: float = 0 # Used to determine the flow of new UAVs in case of landing_take_off
        self.protocol = None # The protocol by which the UAV movement is decided

        # Stats:
        self.collisions_prevented = 0
        self.closest_distance_between_uavs = np.inf
        self.number_of_uav_frames_without_speed = 0
        self.number_of_frames_till_completion = 0
        self.starting_real_time = time.time()
        self.time_spent_moving_uavs = 0
        self.time_spent_checking_collisions = 0
        self.time_spent_adding_uavs = 0

        self.simulation_end = False
        self.show_time_collisions = False

        

    def create_uavs(self):
        """Create the UAVs in the airspace, creates a grid of UAVs"""
        for i in range(self.N):
            for j in range(self.N):
                model = models[(i+j)%len(models)]
                filename = f"models/{model}.obj"
                uav = UAV(self, filename, position=[2*i+1, 1, 2*j+1], scale=None)
                # uav.create_sphere(radius=None, resolution=30)
                # uav.create_convex_hull()
    
    def create_colliding_uavs(self):
        """Create 3 UAVs that are colliding"""
        model1= models[0]
        model2= models[1]
        filename1 = f"models/{model1}.obj"
        filename2 = f"models/{model2}.obj"
        uav1 = UAV(self, filename1, position=[1.5, 1, 0], scale=None)
        uav2 = UAV(self, filename2, position=[0, 1, 0], scale=None)
        uav3 = UAV(self, filename1, position=[1.5, 1, 1.0], scale=None)

    def create_random_uavs(self):
        """Create random UAVs in the airspace"""
        for i in range(self.N):
            for j in range(self.N):
                model = models[np.random.randint(0, len(models))]
                filename = f"models/{model}.obj"
                position = [np.random.uniform(-self.N, 3*self.N), 2, np.random.uniform(-self.N, 3*self.N)]
                rotation_angle = np.random.uniform(0, 2*np.pi)

                uav = UAV(self, filename, position=position, scale=None)
                uav.rotate(rotation_angle, [0, 1, 0])

    def create_random_uavs_non_colliding(self):
        """Create random UAVs in the airspace such that they dont collide"""

        def check_collision(p, positions):
            """Check if the position p collides with any of the positions in the list positions (meaning it is within 2 units of any of the positions)"""
            for pos in positions:
                if np.linalg.norm(np.array(p) - np.array(pos)) < 2:
                    return True
            return False
        
        positions = []
        for i in range(self.N):
            for j in range(self.N):
                
                position = [np.random.uniform(-self.N, 3*self.N), 2, np.random.uniform(-self.N, 3*self.N)]
                while check_collision(position, positions): # Make sure the UAVs dont collide
                    position = [np.random.uniform(-self.N, 3*self.N), 2, np.random.uniform(-self.N, 3*self.N)]
                positions.append(position)
                model = models[np.random.randint(0, len(models))]
                filename = f"models/{model}.obj"
                
                rotation_angle = np.random.uniform(0, 2*np.pi)

                uav = UAV(self, filename, position=position, scale=None)
                uav.rotate(rotation_angle, [0, 1, 0])
        self.find_collisions(show=False)

    def create_time_colliding_uavs(self):
        """Create 2 UAVs that are not colliding, but their paths will collide in the first time step"""
        model1 = np.random.choice(models)
        model2 = np.random.choice(models)
        filename1 = f"models/{model1}.obj"
        filename2 = f"models/{model2}.obj"
        uav1 = UAV(self, filename1, position=[4, 1, 0], scale=None)
        uav2 = UAV(self, filename2, position=[0, 1, 0], scale=None)

        uav1.rotate(-np.pi/2, [0, 1, 0])

        # speed = 9 # Not colliding
        speed = 30 # Colliding
        uav1.speed = np.array(speed*uav1.direction)
        uav2.speed = np.array(speed*uav2.direction)
        # print("Collision check:", self.find_collisions_dt(self.dt, show=True))
        self.find_collisions_dt(self.dt, show=True)

    def create_taking_off_uavs(self, dome_radius:float = 15):
        """Create UAVs that are taking off from the Landing Pad
        Inputs:
        - dome_radius: the radius of the dome around the UAVs
        
        Returns:
        Nothing"""
        time1 = time.time()
        self.protocol = self.protocol_target_beacon

        def random_point_on_upper_hemisphere(radius=15, center=[0, 0, 0]):
            # Generate random angles
            dth = np.pi/4
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, np.pi / 2 - dth)
            
            # Spherical to Cartesian conversion
            x = radius * np.sin(phi) * np.cos(theta)
            z = radius * np.sin(phi) * np.sin(theta)
            y = radius * np.cos(phi)


            
            return np.array([x, y, z]) + center

        def min_distance_from_beacons(beacons, position):
            if not beacons:
                return np.inf
            other_positions = [[beacon.x, beacon.y, beacon.z] for beacon in beacons.values()]
            min_dist = np.min(np.linalg.norm(np.array(other_positions) - np.array(position), axis=1))
            return min_dist

        colors = [Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.ORANGE, Color.MAGENTA]
        
        # self.dome = Sphere3D(p=[0, 0, 0], radius=dome_radius, resolution=30, color=Color.WHITE)
        # self.addShape(self.dome, "dome")
        center = [self.N, 0, self.N]
        self.dome = Sphere3D(p=center, radius=dome_radius, resolution=30, color=Color.WHITE)
        self.center = Sphere3D(p=center, radius=0.3, resolution=30, color=Color.WHITE)
        self.addShape(self.center, "center")
        self.addShape(self.dome, "dome")
        for i in range(self.N):
            for j in range(self.N):
                model = models[(i+j)%len(models)]
                filename = f"models/{model}.obj"
                uav = UAV(self, filename, position=[2*i+1, 1, 2*j+1], scale=None)
                rotation_angle = np.random.uniform(0, 2*np.pi)
                uav.rotate(rotation_angle, [0, 1, 0])

                beacon_pos = random_point_on_upper_hemisphere(dome_radius, center)
                while min_distance_from_beacons(self.beacons, beacon_pos) < 3:
                    beacon_pos = random_point_on_upper_hemisphere(dome_radius, center)
                self.beacons[uav.name] = Sphere3D(p=beacon_pos, radius=0.1, resolution=30, color = colors[(i+j)%len(colors)])
                self.addShape(self.beacons[uav.name], uav.name+"_beacon")
        self.time_spent_adding_uavs += time.time() - time1

    def create_landing_uavs(self, dome_radius:float = 15):
        """Create UAVs that are on the dome sphere and will be landing on the landing spots
        Inputs:
        - dome_radius: the radius of the dome around the UAVs"""

        time1 = time.time()
        self.protocol = self.protocol_landing

        def random_point_on_upper_hemisphere(radius=15, center=[0, 0, 0]):
            # Generate random angles
            dth = np.pi/4
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, np.pi / 2 - dth)
            
            # Spherical to Cartesian conversion
            x = radius * np.sin(phi) * np.cos(theta)
            z = radius * np.sin(phi) * np.sin(theta)
            y = radius * np.cos(phi)


            
            return np.array([x, y, z]) + center
        
        def min_distance_from_uavs(uavs, position):
            if not uavs:
                return np.inf
            other_positions = [uav.position for uav in uavs.values()]
            min_dist = np.min(np.linalg.norm(np.array(other_positions) - np.array(position), axis=1))
            return min_dist

        colors = [Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.ORANGE, Color.MAGENTA]
        center = [self.N, 0, self.N]
        self.dome = Sphere3D(p=center, radius=dome_radius, resolution=30, color=Color.WHITE)
        self.center = Sphere3D(p=center, radius=0.3, resolution=30, color=Color.WHITE)
        self.addShape(self.center, "center")
        self.addShape(self.dome, "dome")


        for i in range(self.N):
            for j in range(self.N):
                model = models[(i+j)%len(models)]
                filename = f"models/{model}.obj"
                position = random_point_on_upper_hemisphere(dome_radius, center)
                while min_distance_from_uavs(self.uavs, position) < 3:
                    position = random_point_on_upper_hemisphere(dome_radius, center)
                uav = UAV(self, filename, position=position, scale=None)
                rotation_angle = np.random.uniform(0, 2*np.pi)
                uav.rotate(rotation_angle, [0, 1, 0])
                self.landing_spots[uav.name] = Sphere3D(p=[2*i+1, 0.1, 2*j+1], radius=0.1, resolution=30, color = colors[(i+j)%len(colors)])
                self.addShape(self.landing_spots[uav.name], uav.name+"_landing_spot")
        self.time_spent_adding_uavs += time.time() - time1

      
    def create_landing_take_off_uavs(self, dome_radius:float = 15, flow:float = 0.5):
        """Create the dome and landing spots for future UAVs that will be landing and taking off"""
        self.new_uavs_flow = flow
        self.dome_radius = dome_radius

        center = [self.N, 0, self.N]
        self.dome = Sphere3D(p=center, radius=dome_radius, resolution=30, color=Color.WHITE)
        self.center = Sphere3D(p=center, radius=0.3, resolution=30, color=Color.WHITE)
        self.addShape(self.center, "center")
        self.addShape(self.dome, "dome")


    def create_new_taking_off_uav(self):
        """Create a new UAV that will be taking off from the landing pad"""


        other_uavs = self.uavs.values()

        def random_point_on_upper_hemisphere(center=[0, 0, 0]):
            # Generate random angles
            dth = np.pi/4
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, np.pi / 2 - dth)
            radius = 15

            # Spherical to Cartesian conversion
            x = radius * np.sin(phi) * np.cos(theta)
            z = radius * np.sin(phi) * np.sin(theta)
            y = radius * np.cos(phi)


            
            return np.array([x, y, z]) + center

        def min_distance_from_beacons(beacons, position):
            if not beacons:
                return np.inf
            other_positions = [[beacon.x, beacon.y, beacon.z] for beacon in beacons.values()]
            min_dist = np.min(np.linalg.norm(np.array(other_positions) - np.array(position), axis=1))
            return min_dist

        
        
        center = [self.N, 0, self.N]
        number_of_uavs = len(self.uavs)
        i = number_of_uavs//self.N
        j = number_of_uavs%self.N
        new_pos = [2*i+1, 0.1, 2*j+1]
        new_speed = np.random.uniform(0, speed_constant)
        new_direction = np.random.uniform(0, 2*np.pi)
        # new_model = np.random.choice(models)
        new_model_index = number_of_uavs%len(models)
        new_model = models[new_model_index]
        new_filename = f"models/{new_model}.obj"
        new_uav = UAV(self, new_filename, position=new_pos, scale=None)
        new_uav.rotate(new_direction, [0, 1, 0])
        new_uav.speed = new_speed*np.array(new_uav.direction)
        self.uavs[new_uav.name] = new_uav
        colors = [Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.ORANGE, Color.MAGENTA]
        beacon_pos = random_point_on_upper_hemisphere(center)
        while min_distance_from_beacons(self.beacons, beacon_pos) < 3:
            beacon_pos = random_point_on_upper_hemisphere(center)
        self.beacons[new_uav.name] = Sphere3D(p=beacon_pos, radius=0.1, resolution=30, color = colors[(i+j)%len(colors)])
        self.addShape(self.beacons[new_uav.name], new_uav.name+"_beacon")
        new_uav.target = self.beacons[new_uav.name]


    def create_new_landing_uav(self):
        """Create a new UAV that is inside the dome sphere and will be landing on the landing spots"""

        time1 = time.time()
        other_uavs = self.uavs.values()

        def random_point_on_upper_hemisphere(center=[0, 0, 0]):
            # Generate random angles
            dth = np.pi/4
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, np.pi / 2 - dth)
            radius = np.random.uniform(self.dome_radius/2, self.dome_radius)

            # Spherical to Cartesian conversion
            x = radius * np.sin(phi) * np.cos(theta)
            z = radius * np.sin(phi) * np.sin(theta)
            y = radius * np.cos(phi)


            
            return np.array([x, y, z]) + center

        def min_distance_from_uavs(uavs, position):
            if not uavs:
                return np.inf
            other_positions = [uav.position for uav in uavs]
            min_dist = np.min(np.linalg.norm(np.array(other_positions) - np.array(position), axis=1))
            return min_dist
        center = [self.N, 0, self.N]
        new_pos = random_point_on_upper_hemisphere(center)

        while min_distance_from_uavs(other_uavs, new_pos) < 3:
            new_pos = random_point_on_upper_hemisphere(center)


        number_of_uavs = len(self.uavs)
        new_speed = np.random.uniform(0, speed_constant)
        new_direction = np.random.uniform(0, 2*np.pi)
        # new_model = np.random.choice(models)
        new_model_index = number_of_uavs%len(models)
        new_model = models[new_model_index]
        new_filename = f"models/{new_model}.obj"
        new_uav = UAV(self, new_filename, position=new_pos, scale=None)
        new_uav.rotate(new_direction, [0, 1, 0])
        new_uav.speed = new_speed*np.array(new_uav.direction)
        self.uavs[new_uav.name] = new_uav
        colors = [Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.ORANGE, Color.MAGENTA]
        
        i = number_of_uavs//self.N
        j = number_of_uavs%self.N
        self.landing_spots[new_uav.name] = Sphere3D(p=[2*i+1, 0.1, 2*j+1], radius=0.1, resolution=30, color = colors[(i+j)%len(colors)])
        self.addShape(self.landing_spots[new_uav.name], new_uav.name+"_landing_spot")
        self.time_spent_adding_uavs += time.time() - time1
        new_uav.target = self.landing_spots[new_uav.name]

    def find_collisions(self, show):
        """Find the collisions between the UAVs in the airspace for a specific moment"""

        if show:
            geometry_names = self._shapeDict.keys()
            collision_geometries = []
            for geometry_name in geometry_names:
                if "collision" in geometry_name:
                    collision_geometries.append(geometry_name)
            for geometry_name in collision_geometries:
                self.removeShape(geometry_name)

        uav_names = list(self.uavs.keys())
        start_time = time.time()
        for i, uav1 in enumerate(uav_names):
            for j in range(i+1, len(uav_names)):
                uav2 = uav_names[j]
                if self.uavs[uav1].collides(self.uavs[uav2], show=show):
                    print(f"!!!{uav1} collides with {uav2}!!!")
                else:
                    # print(f"{uav1} does not collide with {uav2}")
                    continue


    
    def find_collisions_dt(self, dt:float, show:bool = False):
        """Find the collisions between the UAVs in the airspace for a specific time step
        Inputs:
        - dt: the time step
        - show: whether to show the collision boxes
        
        Returns:
        - a dictionary of the UAVs that are colliding at the specific time step"""

        

        time1 = time.time()
        colliding = {}
        for i, uav in enumerate(self.uavs.values()): # Resetting the continuous boxes
            uav.boxes["continuous"] = None

        for i, uav1 in enumerate(self.uavs.values()):
            colliding[i] = set()
            for j, uav2 in enumerate(self.uavs.values()):
                if i == j:
                    continue
                if colliding.get(j) and i in colliding[j]:
                    colliding[i].add(j)
                    continue
                if np.linalg.norm(uav1.position - uav2.position) < self.closest_distance_between_uavs:
                    self.closest_distance_between_uavs = np.linalg.norm(uav1.position - uav2.position)
                if uav1.collides_dt(uav2, dt, show=show):
                    # print(f"!!!{uav1.name} collides with {uav2.name}!!!")
                    colliding[i].add(j)
                    
                else:
                    continue
        self.time_spent_checking_collisions += time.time()-time1
        self.collisions_prevented += len(colliding.values())
        return colliding
        

    def on_key_press(self, symbol, modifiers):
        """Handle the key press events"""

        if symbol == Key.C:
            for uav in self.uavs.values():
                if uav.boxes_visibility["chull"]:
                    uav.remove_convex_hull()
                else:
                    uav.create_convex_hull()
        
        if symbol == Key.U:
            for uav in self.uavs.values():
                if uav.boxes_visibility["sphere"]:
                    uav.remove_sphere()
                else:
                    uav.create_sphere(radius=None, resolution=30)
        
        if symbol == Key.B:
            for uav in self.uavs.values():
                if uav.boxes_visibility["aabb"]:
                    uav.remove_aabb()
                else:
                    uav.create_aabb()
        
        if symbol == Key.K:
            for i,uav in enumerate(self.uavs.values()):
                # if  i==18 or i==17:
                if uav.boxes_visibility["kdop"]:
                    uav.remove_kdop()
                else:
                    uav.create_kdop(kdop_number)

        if symbol == Key.L:
            print("Start of collision check for the airspace")
            time1 = time.time()
            self.find_collisions(show=True)
            if show_times:
                print(f"End of collision check for the airspace: {time.time()-time1:.2f}s")
            else:
                print("End of collision check for the airspace")
        
        if symbol == Key.P:
            self.paused = not self.paused
            if self.paused:
                print("Paused")
            else:
                print("Unpaused")
        
        if symbol == Key.T:
            self.show_time_collisions = not self.show_time_collisions
            if self.show_time_collisions:
                print("Showing time collisions")
            else:
                print("Not showing time collisions")
        


        osprey:UAV = self.uavs.get("v22_osprey_0")
        if not osprey:
            return
        if symbol in [Key.UP, Key.DOWN, Key.LEFT, Key.RIGHT, Key.SPACE, Key.BACKSPACE, Key.R]:
            # osprey.remove_kdop()
            if symbol == Key.UP:
                osprey.move_by([0, 0, -0.1])
            if symbol == Key.DOWN:
                osprey.move_by([0, 0, 0.1])
            if symbol == Key.LEFT:
                osprey.move_by([-0.1, 0, 0])
            if symbol == Key.RIGHT:
                osprey.move_by([0.1, 0, 0])
            if symbol == Key.SPACE:
                osprey.move_by([0, 0.1, 0])
            if symbol == Key.BACKSPACE:
                osprey.move_by([0, -0.1, 0])
            if symbol == Key.R:
                osprey.rotate(np.pi/4, [0, 1, 0])
        
        
    
    def addUAV(self, uav:UAV):
        self.uavs[uav.name] = uav
        self.addShape(uav.mesh, uav.name)
    
    def removeUAV(self, uav:UAV):
        self.removeShape(uav.name)
        del self.uavs[uav.name]
    
    def updateUAV(self, uav:UAV):
        if uav.name not in self.uavs:
            self.addUAV(uav)
        else:
            self.updateShape(uav.mesh, uav.name)

    def on_idle(self):
        """The method that is called at every frame"""
        if self.paused or self.simulation_end:
            return
        if time.time() - self.last_update < self.dt:
            return
        if self.simulation_has_ended():
            print("Simulation has ended")
            self.simulation_end = True
            self.print_stats()

            return
        self.number_of_frames_till_completion += 1
        if show_times:
            print("new frame", time.time()-self.last_update)
        self.last_update = time.time()

        if hasattr(self, "method") and self.method == "landing_take_off": # If the method is landing_take_off, then new UAVs are added over time
            if len(self.uavs.values())<self.N*self.N:
                if np.random.rand()<self.new_uavs_flow:
                    if np.random.rand()<0.5:
                        self.create_new_landing_uav()
                    else:
                        self.create_new_taking_off_uav()



        if not self.protocol:
            print("No protocol set")
            self.protocol = self.protocol_avoidance
            self.protocol_avoidance()
        else:
            self.protocol(show=self.show_time_collisions)
        
        pass

    def protocol_avoidance(self, show = True):
        """A protocol for the UAVs to avoid each other"""
        self.mode = "avoidance"
        self.protocol = self.protocol_avoidance


        def find_best_direction(uav, other_positions, ): 
            """Find the best direction for the UAV to move in to avoid the other UAVs"""
            vectors = [other_pos-uav.position for other_pos in other_positions]
            vectors_normalized = [v/np.linalg.norm(v) for v in vectors if v.any()]
            sum_of_vectors = np.sum(vectors_normalized, axis=0)
            sum_of_vectors = sum_of_vectors/np.linalg.norm(sum_of_vectors)
            best_direction = -sum_of_vectors
            return best_direction
            
                
            
        time1 = time.time()
        collides = self.find_collisions_dt(self.dt, show=True) # Get the collision pairs
        if show_times:
            print(f"Time taken to find the collisions(1): {time.time()-time1:.2f}s")
        uavs = list(self.uavs.values())
        for i,uav in enumerate(self.uavs.values()): # For the UAVs that are colliding try to avoid the other UAVs
            if not i in collides:
                continue
            if not collides[i]:
                continue
            colliding = collides[i]
            other_positions = [uavs[j].position for j in colliding]
            best_direction = find_best_direction(uav, other_positions)
            uav.speed = speed_constant*best_direction
        time2 = time.time()
        collides = self.find_collisions_dt(self.dt, show=show) # Get the collision pairs again
        if show_times:
            print(f"Time taken to find the collisions(2): {time.time()-time2:.2f}s")
        time1 = time.time()
        for i,uav in enumerate(self.uavs.values()): # For the UAVs that are not colliding move them according to set speed
            if not i in collides or not collides[i]:
                uav.move_by(uav.speed*self.dt)
            else:
                # uav.speed = np.array([0, 0, 0])
                uav.number_of_stationary_frames += 1
                self.number_of_uav_frames_without_speed += 1
        self.time_spent_moving_uavs += time.time()-time1

    def protocol_target_beacon(self, show = True):
        """A protocol for the UAVs to reach the target beacon"""
        self.mode = "target_beacon"
        self.protocol = self.protocol_target_beacon

        def find_best_direction(uav, other_positions, beacon_position):
            """Find the best direction for the UAV to move in to avoid the other UAVs and reach the target beacon"""

            if uav.has_reached_beacon(self.beacons[uav.name], threshold=0.15):
                return np.array([0, 0, 0])
            
            vectors = [other_pos-uav.position for other_pos in other_positions]
            vectors_normalized = [v/np.linalg.norm(v) for v in vectors if v.any()]
            sum_of_vectors = np.sum(vectors_normalized, axis=0)
            sum_of_vectors = sum_of_vectors
            if np.allclose(sum_of_vectors, 0):
                avoid_direction = np.array([0, 0, 0])
            else:
                sum_of_vectors = sum_of_vectors/np.linalg.norm(sum_of_vectors)
            avoid_direction = -sum_of_vectors

            if uav.number_of_stationary_frames > 5: # To avoid getting stuck at the start
                uav.move_by(avoid_direction*self.dt*0.1)
            if uav.number_of_stationary_frames > 3: # To avoid getting stuck in general
                return avoid_direction


            target_direction = beacon_position-uav.position            
            target_direction = target_direction/np.linalg.norm(target_direction)

            best_direction = target_direction + avoid_direction
            best_direction = best_direction/np.linalg.norm(best_direction)
            return best_direction

        def find_target_direction(uav, beacon_position):
            """Find the direction in which the UAV should move to reach the target beacon"""
            target_direction = beacon_position-uav.position
            if np.linalg.norm(target_direction) < 0.15:
                return np.array([0, 0, 0])
            target_direction = target_direction/np.linalg.norm(target_direction)
            return target_direction
        
            
                
            
        time1 = time.time()
        collides = self.find_collisions_dt(self.dt, show=show) # Get the collision pairs
        if show_times:
            print(f"Time taken to find the collisions(1): {time.time()-time1:.2f}s")
        uavs = list(self.uavs.values())
        for i,uav in enumerate(self.uavs.values()):

            beacon = self.beacons[uav.name]
            beacon_position = np.array([beacon.x, beacon.y, beacon.z])
            if not collides[i]: # If the UAV is not colliding with any other UAV then try to move it towards the target beacon
                uav.speed = speed_constant*find_target_direction(uav, beacon_position)
                continue

            colliding = collides[i] # If the UAV is colliding with other UAVs then try to avoid the other UAVs and move towards the target beacon
            other_positions = [uavs[j].position for j in colliding]
            best_direction = find_best_direction(uav, other_positions, beacon_position=beacon_position)
            uav.speed = speed_constant*best_direction

        time2 = time.time()
        collides = self.find_collisions_dt(self.dt, show=show)
        if show_times:
            print(f"Time taken to find the collisions(2): {time.time()-time2:.2f}s")
        time1 = time.time()
        for i,uav in enumerate(self.uavs.values()): # For the UAVs that are not colliding move them according to set speed
            if not i in collides or not collides[i]:
                uav.number_of_stationary_frames = 0
                uav.move_by(uav.speed*self.dt)
            else:
                uav.number_of_stationary_frames += 1
                self.number_of_uav_frames_without_speed += 1
        self.time_spent_moving_uavs += time.time()-time1


    def protocol_landing(self, show = True):
        """A protocol for the UAVs to reach the landing spot"""

        self.mode = "landing"
        self.protocol = self.protocol_landing

        def find_best_direction(uav, other_positions, landing_spot_position):
            """Find the best direction for the UAV to move in to avoid the other UAVs and reach the landing spot"""

            if uav.has_reached_landing_spot(self.landing_spots[uav.name], threshold=0.15):
                return np.array([0, 0, 0])
            
            vectors = [other_pos-uav.position for other_pos in other_positions]
            vectors_normalized = [v/np.linalg.norm(v) for v in vectors if v.any()]
            sum_of_vectors = np.sum(vectors_normalized, axis=0)
            sum_of_vectors = sum_of_vectors
            if np.allclose(sum_of_vectors, 0):
                avoid_direction = np.array([0, 0, 0])
            else:
                sum_of_vectors = sum_of_vectors/np.linalg.norm(sum_of_vectors)
            avoid_direction = -sum_of_vectors

            if uav.number_of_stationary_frames > 5: # To avoid getting stuck at the start
                uav.move_by(avoid_direction*self.dt*0.1)
            if uav.number_of_stationary_frames > 3: # To avoid getting stuck in general
                return avoid_direction


            target_direction = landing_spot_position-uav.position            
            target_direction = target_direction/np.linalg.norm(target_direction)

            best_direction = target_direction + avoid_direction
            best_direction = best_direction/np.linalg.norm(best_direction)
            return best_direction

        def find_target_direction(uav, landing_spot_position):
            """Find the direction in which the UAV should move to reach the landing spot"""
            target_direction = landing_spot_position-uav.position
            if np.linalg.norm(target_direction) < 0.15:
                return np.array([0, 0, 0])
            target_direction = target_direction/np.linalg.norm(target_direction)
            return target_direction
        
            
                
            
        time1 = time.time()
        collides = self.find_collisions_dt(self.dt, show=show) # Get the collision pairs
        if show_times:
            print(f"Time taken to find the collisions(1): {time.time()-time1:.2f}s")
        uavs = list(self.uavs.values())

        for i,uav in enumerate(self.uavs.values()):

            landing_spot = self.landing_spots[uav.name]
            landing_spot_position = np.array([landing_spot.x, landing_spot.y, landing_spot.z])
            if not collides[i]: # If the UAV is not colliding with any other UAV then try to move it towards the landing spot
                uav.speed = speed_constant*find_target_direction(uav, landing_spot_position)
                continue

            colliding = collides[i] # If the UAV is colliding with other UAVs then try to avoid the other UAVs and move towards the landing spot
            other_positions = [uavs[j].position for j in colliding]
            best_direction = find_best_direction(uav, other_positions, landing_spot_position)
            uav.speed = speed_constant*best_direction

        time2 = time.time()
        collides = self.find_collisions_dt(self.dt, show=show) # Check for collisions again
        if show_times:
            print(f"Time taken to find the collisions(2): {time.time()-time2:.2f}s")
        time1 = time.time()
        for i,uav in enumerate(self.uavs.values()): # For the UAVs that are not colliding move them according to set speed
            if not i in collides or not collides[i]:
                uav.number_of_stationary_frames = 0
                uav.move_by(uav.speed*self.dt)
            else:
                uav.number_of_stationary_frames += 1
                self.number_of_uav_frames_without_speed += 1
        self.time_spent_moving_uavs += time.time()-time1
    
    def protocol_landing_take_off(self, show = True):
        """A protocol for the UAVs to reach the landing spot and then take off again"""

        self.mode = "landing_take_off"
        self.protocol = self.protocol_landing_take_off

        def find_best_direction(uav, other_positions, target_position):
            """Find the best direction for the UAV to move in to avoid the other UAVs and reach the target"""

            if uav.has_reached_target( threshold=0.15):
                return np.array([0, 0, 0])
            
            vectors = [other_pos-uav.position for other_pos in other_positions]
            vectors_normalized = [v/np.linalg.norm(v) for v in vectors if v.any()]
            sum_of_vectors = np.sum(vectors_normalized, axis=0)
            sum_of_vectors = sum_of_vectors
            if np.allclose(sum_of_vectors, 0):
                avoid_direction = np.array([0, 0, 0])
            else:
                sum_of_vectors = sum_of_vectors/np.linalg.norm(sum_of_vectors)
            avoid_direction = -sum_of_vectors

            if uav.number_of_stationary_frames > 5:
                uav.move_by(avoid_direction*self.dt*0.1)
            if uav.number_of_stationary_frames > 3:
                return avoid_direction
            
            target_direction = target_position-uav.position

            target_direction = target_direction/np.linalg.norm(target_direction)

            best_direction = target_direction + avoid_direction
            best_direction = best_direction/np.linalg.norm(best_direction)
            return best_direction
        
        def find_target_direction(uav, target_position):
            """Find the direction in which the UAV should move to reach the target position"""
            target_direction = target_position-uav.position
            if uav.has_reached_target( threshold=0.15):
                return np.array([0, 0, 0])
            target_direction = target_direction/np.linalg.norm(target_direction)
            return target_direction


        time1 = time.time()
        collides = self.find_collisions_dt(self.dt, show=show) # Get the collision pairs
        if show_times:
            print(f"Time taken to find the collisions(1): {time.time()-time1:.2f}s")
        uavs = list(self.uavs.values())

        for i,uav in enumerate(self.uavs.values()):

            target = uav.target
            target_position = np.array([target.x, target.y, target.z])  
            if not collides[i]: # If the UAV is not colliding with any other UAV then try to move it towards the landing spot
                uav.speed = speed_constant*find_target_direction(uav, target_position)
                continue

            colliding = collides[i] # If the UAV is colliding with other UAVs then try to avoid the other UAVs and move towards the landing spot
            other_positions = [uavs[j].position for j in colliding]
            best_direction = find_best_direction(uav, other_positions, target_position)
            uav.speed = speed_constant*best_direction

        time2 = time.time()
        collides = self.find_collisions_dt(self.dt, show=show) # Check for collisions again
        if show_times:
            print(f"Time taken to find the collisions(2): {time.time()-time2:.2f}s")
        time1 = time.time()
        for i,uav in enumerate(self.uavs.values()): # For the UAVs that are not colliding move them according to set speed
            if not i in collides or not collides[i]:
                uav.number_of_stationary_frames = 0
                uav.move_by(uav.speed*self.dt)
            else: # If the UAV is colliding with other UAVs then do not move it
                uav.number_of_stationary_frames += 1
                self.number_of_uav_frames_without_speed += 1
        self.time_spent_moving_uavs += time.time()-time1
        

    def simulation_has_ended(self):
        if self.mode == "landing":
            for uav in self.uavs.values():
                if not uav.has_reached_landing_spot(self.landing_spots[uav.name], threshold=0.15):
                    return False
            return True
        if self.mode == "target_beacon":
            for uav in self.uavs.values():
                if not uav.has_reached_beacon(self.beacons[uav.name], threshold=0.15):
                    return False
            return True
        if self.mode == "landing_take_off": 
            if len(self.uavs.values())<self.N*self.N:
                return False
            for uav in self.uavs.values():
                if not uav.has_reached_target( threshold=0.15):
                    return False
            return True

        return False

    def print_stats(self):
        """Print the statistics of the simulation"""
        print(f"Number of seconds till completion: {time.time()-self.starting_real_time:.2f}s")
        print(f"Number of frames till completion: {self.number_of_frames_till_completion}")
        print(f"Number of UAV frames without speed: {self.number_of_uav_frames_without_speed}")
        print(f"Number of collisions prevented: {self.collisions_prevented}")
        print(f"Closest distance between UAVs: {self.closest_distance_between_uavs}")
        print(f"Time spent adding UAVs: {self.time_spent_adding_uavs:.2f}s")
        print(f"Time spent moving UAVs: {self.time_spent_moving_uavs:.2f}s")
        print(f"Time spent checking collisions: {self.time_spent_checking_collisions:.2f}s")


class Kdop(Polyhedron3D):

    classes_to_kdop:dict[str, (np.ndarray, np.ndarray, "Kdop")] = {}

    def __init__(self, vertices:np.ndarray, k:int, scene:Scene3D, name:str, uav:UAV = None) -> None:
        self.pc_vertices = vertices
        self.k = k
        self.scene = scene
        self.name = name
        self.uav = uav


        self.create_kdop()

        if self.uav:
            
            Kdop.classes_to_kdop[self.uav._class] = (self.uav.position, self.uav.direction, self)
            return
        

        

    def create_kdop(self):

        if self.uav:
            if self.uav._class in Kdop.classes_to_kdop:
                self.create_from_class()
                return
        if self.k == 6:
            self.create_6dop()
            return
        if self.k == 14:
            self.create_14dop()
            return
    
    def create_6dop(self):
        "Create a 6-dop"

        min_x, min_y, min_z = np.min(self.pc_vertices, axis=0)
        max_x, max_y, max_z = np.max(self.pc_vertices, axis=0)
        faces = [
            [[min_x, min_y, min_z], [max_x, min_y, min_z], [max_x, max_y, min_z], [min_x, max_y, min_z]],
            [[min_x, min_y, min_z], [min_x, max_y, min_z], [min_x, max_y, max_z], [min_x, min_y, max_z]],
            [[min_x, min_y, min_z], [min_x, min_y, max_z], [max_x, min_y, max_z], [max_x, min_y, min_z]],
            [[max_x, max_y, min_z], [max_x, min_y, min_z], [max_x, min_y, max_z], [max_x, max_y, max_z]],
            [[max_x, max_y, min_z], [max_x, max_y, max_z], [min_x, max_y, max_z], [min_x, max_y, min_z]],
            [[max_x, max_y, max_z], [max_x, min_y, max_z], [min_x, min_y, max_z], [min_x, max_y, max_z]],
        ]
        polygons = []
        for face in faces:
            polygon = ConvexPolygon3D(np.array(face), color=Color.RED)
            polygons.append(polygon)
        
        super().__init__(polygons, color=Color.RED, name = self.name)
        if self.scene:
            self.scene.addShape(self, self.name)
    
    def create_14dop(self):
        "Create a 14-dop "

        import time

        def find_intersection(plane1, plane2, plane3):
            
            A = np.array([plane1[:3], plane2[:3], plane3[:3]])
            b = np.array([-plane1[3], -plane2[3], -plane3[3]])
            x = np.linalg.solve(A, b)
            return x

        def check_if_neighbor(corner_dir, face_dir):
            for i in range(3):
                if face_dir[i] != 0:
                    if corner_dir[i]==face_dir[i]:
                        return True
            return False
        
        def plane_equation_from_point_normal(point, normal):
            # Normalize the normal vector
            unit_normal = normal / np.linalg.norm(normal)
            
            # Extract components
            a, b, c = unit_normal
            x0, y0, z0 = point
            
            # Calculate d using the point on the plane
            d = -(a*x0 + b*y0 + c*z0)
            
            # Return the coefficients of the plane equation
            return [a, b, c, d]
        
        directions = [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            [1, 1, 1],
            [1, 1, -1],
            [1, -1, 1],
            [1, -1, -1],

            [-1, 0, 0],
            [0, -1, 0],
            [0, 0, -1],
            [-1, 1, 1],
            [-1, 1, -1],
            [-1, -1, 1],
            [-1, -1, -1],

        ]
        directions = np.array(directions)

        start = time.time()

        points = []
        dot_products = np.dot(self.pc_vertices, directions.T) # Dot product of the vertices with the directions
        
        max_indices = np.argmax(dot_products, axis=0)
        min_indices = np.argmin(dot_products, axis=0)

        # Retrieve the vertices corresponding to the maximum and minimum dot products
        max_vertices = self.pc_vertices[max_indices]
        min_vertices = self.pc_vertices[min_indices]

        # Map directions to corresponding, furthest away, vertices
        directions_to_vertices = {}
        for i, direction in enumerate(directions[:7]):
            directions_to_vertices[tuple(direction)] = max_vertices[i]
            directions_to_vertices[tuple(-direction)] = min_vertices[i]

        triangles = [] # Finding the triangles that are formed by the intersection of the corner planes with the faces
        for dir in directions_to_vertices.keys():
            if 0 in dir: # Only going for corners
                continue

            v = directions_to_vertices[dir] # Furthest vertex in the direction
            plane = plane_equation_from_point_normal(v, dir) # Plane equation of the corner
            face_planes = [] # Planes of the faces that are neighbors of the corner
            for dir2 in directions_to_vertices.keys():
                if not 0 in dir2: # Only going for faces
                    continue
                # Check if the corner is a neighbor of the face
                if not check_if_neighbor(dir, dir2):
                    continue
                plane2 = plane_equation_from_point_normal(directions_to_vertices[dir2], dir2)
                face_planes.append(plane2)

            temp_points=[] # To avoid duplicate points
            for plane2 in face_planes:
                for plane3 in face_planes:
                    if plane2 == plane3:
                        continue
                    point = find_intersection(plane, plane2, plane3) # Intersection of the corner with the faces
                    for temp_p in temp_points:
                        if np.linalg.norm(temp_p - point) < 0.001: # Dont add duplicate points
                            break
                    else:
                        points.append(point)
                        temp_points.append(point)
            triangle = Triangle3D(p1=temp_points[0], p2=temp_points[1], p3=temp_points[2], color=Color.RED)
            
            triangles.append(triangle)


        mesh_point = np.mean(self.pc_vertices, axis=0) # The center of the mesh, used to check if other points are inside the kdop
        points_to_remove = [] # Points of the triangles that are not inside the kdop
        valid_points = [] # Points that are inside the kdop
        for p in points:            
            for triangle in triangles:
                if not triangle.points_on_same_side(mesh_point, p): # Check if the point is inside the kdop
                    points_to_remove.append(p)
                    break
            else:
                valid_points.append(p)

        intersections=[] # The intersections of the triangles that define the kdop
        for triangle in triangles:

            lines = [Line3D(triangle.p1, triangle.p2), Line3D(triangle.p2, triangle.p3), Line3D(triangle.p3, triangle.p1)] 
            for other_triangle in triangles:
                if triangle == other_triangle:
                    continue
                
                for line in lines:
                    intersection = other_triangle.getLineIntersection(line)
                    
                    if intersection is not None:
                        intersections.append(intersection)

        for point in intersections: # Now valid_points will contain all the points that define the kdop
            valid_points.append(point)

        
        dop_faces=[]
        dop_polygons = []



        for direction in directions: # For each direction, find the points that are on the plane 
                                                           # defined by the direction and create a face of the kdop
            dop_face_points=[]
            v = directions_to_vertices[tuple(direction)]
            plane = plane_equation_from_point_normal(v, direction)
            a, b, c, d = plane
            for i, p in enumerate(valid_points):
                
                if np.abs(a*p[0] + b*p[1] + c*p[2] + d) < 0.01: # Point is on the plane defined by the direction
                    for other_p in dop_face_points:
                        if np.linalg.norm(p - other_p) < 0.0001: # Point is already in the face
                            break
                    else:
                        dop_face_points.append(p)


          
            dop_faces.append(dop_face_points)
            dop_face_points=np.array(dop_face_points)
            face_polygon = ConvexPolygon3D(dop_face_points, normal=direction,color=Color.RED) # The face of the kdop corresponding to the direction
            dop_polygons.append(face_polygon)
        if show_times:
            print(f"Creating kdop named {self.name} took {time.time()-start:.2f}s")
        super().__init__(dop_polygons, color=Color.RED, name = self.name) 
        if self.scene:
            self.scene.addShape(self, self.name)


        

    
    def create_from_class(self):
        """If a kdop is already created for the UAV class, then copy that kdop"""
        import time

        time1 = time.time()
        other_uav_position, other_uav_direction, other_kdop = Kdop.classes_to_kdop[self.uav._class]
        other_polygons = other_kdop._polygons
        new_polygons = []
        for polygon in other_polygons:
            new_vertices = list(polygon.points)[1:]
            new_polygon = ConvexPolygon3D(np.array(new_vertices), color=Color.RED)
            new_polygons.append(new_polygon)
            
        new_uav_position = self.uav.position

        super().__init__(new_polygons, color=Color.RED, name = self.name)
        self.move_by(new_uav_position - other_uav_position)
        angle = np.arccos(np.dot(self.uav.direction, other_uav_direction))
        axis = np.cross(self.uav.direction, other_uav_direction)
        R = get_rotation_matrix(angle, axis)
        self.rotate(R, new_uav_position)
        if self.scene:
            self.scene.addShape(self, self.name)
        if show_times:
            print(f"Cached kdop creation time: {time.time()-time1:.2f}s ({self.name})")

        

    def remove(self):
        if self.scene:
            self.scene.removeShape(self.name)     
    
    def move_by(self, distance: NDArray):
        super().move_by(distance)
        if self.uav:
            Kdop.classes_to_kdop[self.uav._class] = (self.uav.position, self.uav.direction, self)
    
    def rotate(self, R:NDArray , center:NDArray):
        super().rotate(R, center)
        if self.uav and "_0" in self.uav._class:
            Kdop.classes_to_kdop[self.uav._class] = (self.uav.position, self.uav.direction, self)

def check_mesh_collision_trimesh(mesh1, mesh2):
    """Check if two meshes collide using trimesh, an external library
    Inputs:
    - mesh1: the first mesh
    - mesh2: the second mesh
    Outputs:
    - True if the meshes collide, False otherwise
    """
    
    trimesh1 = trimesh.Trimesh(vertices=mesh1.vertices, faces=mesh1.triangles)
    trimesh2 = trimesh.Trimesh(vertices=mesh2.vertices, faces=mesh2.triangles)
    collision_manager = trimesh.collision.CollisionManager()
    collision_manager.add_object("trimesh1", trimesh1)
    collision_manager.add_object("trimesh2", trimesh2)
    
    return collision_manager.in_collision_internal()


def main():

    print("Select the question number to run (Q9 will run in any suitable case):")
    for i, info in enumerate(config_info.values()):
        print(f"{i}. {info}")
    
    choice = input("Enter the choice (1-8, default is 8): ")
    try:
        choice = int(choice)
    except:
        choice = 8
    while int(choice) not in range(9):
        if not choice:
            choice = 8
            break
        choice = int(input("Not an acceptable choice. Enter the choice (1-8, default is 8): "))
    config[f"Q{choice}"] = True

    N_2 = input("Enter the number of UAVs (default is 25) (must be a square number): ")
    try:
        if np.sqrt(int(N_2))%1 != 0:
            N_2 = 25
    except ValueError:
        N_2 = 25
    N = int(np.sqrt(int(N_2)))

    dt = 0.15
    if get_question()>=6:
        try:
            dt  = float(input("Enter the time step (default is 0.15, close to 0.15 is recommended): "))
        except:
            dt = 0.15
    
    
    
    if get_question() < 6:
        delattr(Airspace, "on_idle")
    
    airspace = Airspace(1920, 1080, N = 5, dt=dt)

    if get_question() == 1: #Random UAVs
        airspace.create_random_uavs()
    if get_question() == 2: # Uavs on the landing spot for better visualization of bounding boxes
        airspace.create_uavs()
        print("""Show bounding boxes: K(Kdop), U(Sphere), B(AABB), C(Convex hull)""")
    if get_question() == 3:
        airspace.create_random_uavs()
        airspace.find_collisions(show=True)
        print("Find collisions any time by pressing L")
    if get_question() == 4:
        airspace.create_random_uavs()
        airspace.find_collisions(show=True)
        print("Find collisions any time by pressing L")
    if get_question() == 5:
        airspace.create_random_uavs()
        airspace.find_collisions(show=True)
        print("Find collisions any time by pressing L")

    if get_question() == 6:
        airspace.create_random_uavs_non_colliding()
        airspace.protocol_avoidance(show=True)
        print("Press P to pause/unpause")
    if get_question() == 7:
        airspace.create_random_uavs_non_colliding()
        airspace.avoidance(show=False)
        print("Press P to pause/unpause")
    if get_question() == 8:
        print("Possible protocols for Q8:")
        print("1. Target beacon (UAVs take off and reach the target beacon)")
        print("2. Landing (UAVs start from their beacons and land on the landing spots)")
        print("3. Landing time (UAVs start from their beacons and land on the landing spots, new UAVs are added during the simulation in random positions)")
        choice = input("Select the protocol to run (1-3, default is 3): ")
        try:
            choice = int(choice)
        except:
            choice = 3
        while int(choice) not in range(1, 4):
            if not choice:
                choice = 3
                break
            choice = input("Not an acceptable choice. Select the protocol to run (1-3, default is 3): ")
        if int(choice) == 1:
            airspace.create_taking_off_uavs()
            airspace.protocol_target_beacon(show=False)
        if int(choice) == 2:
            airspace.create_landing_uavs()
            airspace.protocol_landing(show=False)
        if int(choice) == 3:
            flow = 0.5
            if get_question() == 8:
                try:
                    flow = float(input("Enter the flow of new UAVs (default is 0.5, close to 0.5 is recommended): "))
                except:
                    flow = 0.5
            airspace.create_landing_take_off_uavs( dome_radius=15, flow = flow)
            airspace.protocol = airspace.protocol_landing_take_off
            airspace.method = "landing_take_off"
        print("Press P to pause/unpause")


    


    



        
    airspace.mainLoop()

    


if __name__ == "__main__":
    main()





























