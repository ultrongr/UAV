import open3d as o3d
import open3d.geometry as o3dg # type: ignore
import open3d.visualization.gui as gui # type: ignore
import open3d.visualization.rendering as rendering # type: ignore
from open3d.visualization.gui import MouseEvent, KeyEvent # type: ignore
from open3d.visualization.rendering import Camera # type: ignore
import numpy as np
import time
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

models = ["v22_osprey",
        "twin_copter",
        "F52",
        "Helicopter",
        "quadcopter_scifi",
        
        # "fight_drone", ## small kdop issue
        # "B2_Spirit", ## small kdop issue
        
    ]

kdop_number = 14
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
        print(self.name, f"{len(self.mesh.vertices)} vertices")
        UAV.classes[self._class] += 1


        self.scene.addUAV(self)

        self.boxes = {
            "aabb": None,
            "chull": None,
            "sphere": None,
            "kdop": None,
            "aabb_node": None,
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
        
        if position:            
            self.move_to(position)
        
        self.scale(scale, fit_to_unit_sphere= (not scale))

    def __eq__(self, other):
        return self.name == other.name


                
        
    def move_to(self, new_position:np.ndarray|list):
        "Move the UAV to the specified position"
        new_position = np.array(new_position)
        self.position=np.array(self.position)
        dist = new_position - self.position
        self.mesh.vertices += dist
        self.position = new_position
        self.mesh._update(self.name, self.scene)

        for box_name in self.boxes.keys():
            # print(box_name)
            if not self.boxes.get(box_name):
                continue
            
            box=self.boxes[box_name]
            if box_name == "kdop":
                
                box.move_by(dist)
                if self.boxes_visibility[box_name]:
                    self.remove_kdop()
                    self.create_kdop(kdop_number)
            
            elif box_name == "aabb":
                box.translate(dist)
                if self.boxes_visibility[box_name]:
                    self.remove_aabb()
                    self.create_aabb()
            
            elif box_name == "chull":
                for vertex in box._shape.vertices:
                    vertex += dist
                if self.boxes_visibility[box_name]:
                    self.remove_convex_hull()
                    self.create_convex_hull()
            
            elif box_name == "aabb_node":
                self.boxes["aabb_node"] = None

                    
    
    def move_by(self, dist:np.ndarray):
        "Move the UAV by the specified distance"
        self.move_to(self.position + dist)

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


        for box_name in self.boxes.keys():
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

    def collides(self, other: "UAV", show:bool = False):
        """Check if the UAV collides with another UAV
        The collision check is done usng several methods following a hierarchical order"""
        


        hierarchy = [
            self.collides_aabb,
            self.collides_aabb_node,
            self.collides_kdop,            
            self.collides_convex_hull,
            self.collides_mesh_random,
            # self.collides_mesh,
        ]
        for i, method in enumerate(hierarchy):
            if method(other, show= (i==len(hierarchy)-1)):
                continue
            else:
                return False
            
        return True

    def collides_aabb(self, other: "UAV", show:bool = False):
        if not self.boxes["aabb"]:
            self.create_aabb()
        if not other.boxes["aabb"]:
            other.create_aabb()
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
        if not self.boxes["kdop"]:
            self.create_kdop(kdop_number)
        if not other.boxes["kdop"]:
            other.create_kdop(kdop_number)
        
        dist = np.linalg.norm(self.position - other.position)
        if dist>2:
            return False
        
        kdop1_points = self.boxes["kdop"].vertices
        kdop2_points = other.boxes["kdop"].vertices
        if np.any(np.max(kdop1_points, axis=0) < np.min(kdop2_points, axis=0)) or np.any(np.min(kdop1_points, axis=0) > np.max(kdop2_points, axis=0)):
            return False
        
        return self.boxes["kdop"].collides_lines(other.boxes["kdop"], show=show, scene=self.scene)

    def collides_aabb_node(self, other: "UAV", show:bool = False):
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
        
        return self.boxes["aabb_node"].collides(other.boxes["aabb_node"], show=True, scene=self.scene)
        

       

    def collides_mesh(self, other: "UAV", show:bool = False):
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
    
    def collides_convex_hull(self, other: "UAV", show:bool = False):

        if show:
            pair = f"{self.name}_{other.name}"
            self.scene.removeShape(pair + "_collision_point1")
        if not self.boxes["chull"]:
            self.create_convex_hull()
            
        if not other.boxes["chull"]:
            other.create_convex_hull()
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
        counter = 0
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
                        self.scene.addShape(Point3D(np.mean(v1, axis=0), color=Color.BLUE, size=5), pair + "_collision_point1")
                    return True

                        
    def collides_mesh_random(self, other: "UAV", show:bool = False):
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

        aabb_node = AabbNode(self.mesh.vertices, max_depth = 4)
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
            # for i, face in enumerate(self.boxes["kdop"]._polygons):
            #     self.scene.addShape(face, self.name+"_kdop_face_"+str(i))
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

    def __init__(self, width, height, N, window_name="UAV"):
        super().__init__(width, height, window_name)
        self.uavs: list[UAV] = {}
        self.N = N
        self.landing_pad = LandingPad(N, self)
        # self.create_uavs()
        # self.create_random_uavs()
        self.create_colliding_uavs()
        # self.find_kdop_collisions()

    def create_uavs(self):
        for i in range(self.N):
            for j in range(self.N):
                model = models[(i+j)%len(models)]
                filename = f"models/{model}.obj"
                uav = UAV(self, filename, position=[2*i+1, 1, 2*j+1], scale=None)
                # uav.create_sphere(radius=None, resolution=30)
                # uav.create_convex_hull()
    
    def create_colliding_uavs(self):
        model1= models[0]
        model2= models[1]
        filename1 = f"models/{model1}.obj"
        filename2 = f"models/{model2}.obj"
        uav1 = UAV(self, filename1, position=[1.5, 1, 0], scale=None)
        # uav2 = UAV(self, filename2, position=[0, 1, 0], scale=None)
        uav3 = UAV(self, filename1, position=[1.5, 1, 1.0], scale=None)

    def create_random_uavs(self):
        for i in range(self.N):
            for j in range(self.N):
                model = models[np.random.randint(0, len(models))]
                filename = f"models/{model}.obj"
                position = [np.random.uniform(-self.N, 3*self.N), 2, np.random.uniform(-self.N, 3*self.N)]
                rotation_angle = np.random.uniform(0, 2*np.pi)

                uav = UAV(self, filename, position=position, scale=None)
                uav.rotate(rotation_angle, [0, 1, 0])
    
    def find_collisions(self, show):

        if show:
            geometry_names = self._shapeDict.keys()
            for geometry_name in geometry_names:
                if "collision" in geometry_name:
                    self.removeShape(geometry_name)

        uav_names = list(self.uavs.keys())
        start_time = time.time()
        for i, uav1 in enumerate(uav_names):
            for j in range(i+1, len(uav_names)):
                uav2 = uav_names[j]
                if self.uavs[uav1].collides(self.uavs[uav2], show=show):
                    print(f"{uav1} collides with {uav2}")
                else:
                    print(f"{uav1} does not collide with {uav2}")
        print(f"Collision check: {time.time()-start_time:.2f}s")



    def find_kdop_collisions(self):
        collisions = {}
        checked_pairs = {}
        for uav1 in self.uavs.values():
            for uav2 in self.uavs.values():
                if uav1 == uav2:
                    continue
                if (uav1.name, uav2.name) in checked_pairs or (uav2.name, uav1.name) in checked_pairs:
                    continue
                checked_pairs[(uav1.name, uav2.name)] = True
                created1 = False
                created2 = False
                dist = np.linalg.norm(uav1.position - uav2.position)
                if dist>2:
                    continue
                if not uav1.boxes["kdop"]:
                    created1 = True
                    uav1.create_kdop(kdop_number)
                if not uav2.boxes["kdop"]:
                    created2 = True
                    uav2.create_kdop(kdop_number)
                import time
                time1 = time.time()
                if uav1.boxes["kdop"].collides_lines(uav2.boxes["kdop"], show=True, scene=self):
                    print(f"{uav1.name} collides with {uav2.name}")
                    print(f"Collision check: {time.time()-time1:.2f}s")

                    collisions[(uav1.name, uav2.name)] = True
                    

                if created1:
                    uav1.remove_kdop()
                if created2:
                    uav2.remove_kdop()

    def find_aabb_collisions(self):
        collisions = {}
        for uav1 in self.uavs.values():
            for uav2 in self.uavs.values():
                if uav1 == uav2:
                    continue
                if (uav1.name, uav2.name) in collisions or (uav2.name, uav1.name) in collisions:
                    continue

                import time
                time1 = time.time()
                uav1_aabb_node = AabbNode(uav1.mesh.vertices, max_depth = 3)
                uav2_aabb_node = AabbNode(uav2.mesh.vertices, max_depth = 3)
                if uav1_aabb_node.collides(uav2_aabb_node, show=True, scene=self):
                    print(f"{uav1.name} collides with {uav2.name}")
                    collisions[(uav1.name, uav2.name)] = True
                else:
                    self.removeShape("aabb_node_collision_1")
                    self.removeShape("aabb_node_collision_2")
                    print(f"{uav1.name} does not collide with {uav2.name}")
                print(f"Collision check: {time.time()-time1:.2f}s")

    def find_mesh_collisions_slow(self):
        import tqdm
        collisions = {}
        for uav1 in self.uavs.values():
            for uav2 in self.uavs.values():
                if uav1 == uav2:
                    continue
                if (uav1.name, uav2.name) in collisions or (uav2.name, uav1.name) in collisions:
                    continue
                import time
                colision = False
                dist = np.linalg.norm(uav1.position - uav2.position)
                if dist>2:
                    continue
                time1 = time.time()
                mesh1 = uav1.mesh
                mesh2 = uav2.mesh
                dist_cutoff = 0.1
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


                for t1 in tqdm.tqdm(range(len(triangles_3d1)), desc="outer loop"):
                    for t2 in range(len(triangles_3d2)):
                        
                        triangle1 = triangles_3d1[t1]
                        triangle2 = triangles_3d2[t2]
                        v1 = triangle1.getPoints()
                        v2 = triangle2.getPoints()
                        dist = np.linalg.norm(np.mean(v1, axis=0) - np.mean(v2, axis=0))
                        if dist > dist_cutoff:
                            continue
                        if triangles_3d1[t1].collides_triangle(triangles_3d2[t2]):
                            print(f"{uav1.name} collides with {uav2.name}")
                            collisions[(uav1.name, uav2.name)] = True
                            colision = True
                            break
                    else:
                        continue
                    break
                if not colision:
                    print(f"{uav1.name} does not collide with {uav2.name}")
                else:
                    print(f"{uav1.name} collides with {uav2.name}")
                print(f"Collision check: {time.time()-time1:.2f}s")

    def find_mesh_collisions_random(self):
        import tqdm
        collisions = {}
        checked_pairs = {}

        number_of_partitions: int = 20
        percentage=0.1
        step = int(1/percentage)
        print(f"Random selective collision check, percentage: {percentage*100:.2f}%")

        for uav1 in self.uavs.values():
            for uav2 in self.uavs.values():
                if uav1 == uav2:
                    continue
                if (uav1.name, uav2.name) in checked_pairs or (uav2.name, uav1.name) in checked_pairs:
                    continue
                checked_pairs[(uav1.name, uav2.name)] = True
                import time
                colision = False
                dist = np.linalg.norm(uav1.position - uav2.position)
                if dist>2:
                    continue
                time1 = time.time()


                mesh1 = uav1.mesh
                mesh2 = uav2.mesh

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
                                space1[x, y, z].append(i)
                
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
                                space2[x, y, z].append(i)

                for i in tqdm.tqdm(range(number_of_partitions), desc="outer loop"):
                    for j in range(number_of_partitions):
                        for k in range(number_of_partitions):
                            for t1 in space1[i, j, k]:
                                for t2 in space2[i, j, k]:
                                    triangle1 = mesh1.triangles[t1]
                                    triangle2 = mesh2.triangles[t2]
                                    v1 = mesh1.vertices[triangle1]
                                    v2 = mesh2.vertices[triangle2]
                                    if Triangle3D(v1[0], v1[1], v1[2]).collides_triangle(Triangle3D(v2[0], v2[1], v2[2])):
                                        collisions[(uav1.name, uav2.name)] = True
                                        colision = True
                                        break
                                if colision:
                                    break
                            if colision:
                                break
                        if colision:
                            break
                    if colision:
                        break
                if not colision:
                    print(f"{uav1.name} does not collide with {uav2.name}")
                else:
                    print(f"{uav1.name} collides with {uav2.name}")
                print(f"Collision check: {time.time()-time1:.2f}s")

    def find_mesh_collisions_opt(self):
        import tqdm
        collisions = {}
        checked_pairs = {}

        number_of_partitions: int = 20
        print(f"New optimized collision check, number of pieces: {number_of_partitions}")

        for uav1 in self.uavs.values():
            for uav2 in self.uavs.values():
                if uav1 == uav2:
                    continue
                if (uav1.name, uav2.name) in checked_pairs or (uav2.name, uav1.name) in checked_pairs:
                    continue
                checked_pairs[(uav1.name, uav2.name)] = True
                import time
                colision = False
                dist = np.linalg.norm(uav1.position - uav2.position)
                if dist>2:
                    continue
                time1 = time.time()


                mesh1 = uav1.mesh
                mesh2 = uav2.mesh

                min_cords1 = np.min(mesh1.vertices, axis=0)
                max_cords1 = np.max(mesh1.vertices, axis=0)
                min_cords2 = np.min(mesh2.vertices, axis=0)
                max_cords2 = np.max(mesh2.vertices, axis=0)

                space1 = np.ndarray((number_of_partitions+1, number_of_partitions+1, number_of_partitions+1), dtype=object)
                space2 = np.ndarray((number_of_partitions+1, number_of_partitions+1, number_of_partitions+1), dtype=object)
                for i in range(number_of_partitions+1):
                    for j in range(number_of_partitions+1):
                        for k in range(number_of_partitions+1):
                            space1[i, j, k] = []
                            space2[i, j, k] = []
                print(f"time1: {time.time()-time1:.2f}s")
                
                min1 = np.min(mesh1.vertices, axis=0)
                max1 = np.max(mesh1.vertices, axis=0)
                min2 = np.min(mesh2.vertices, axis=0)
                max2 = np.max(mesh2.vertices, axis=0)

                max_total = np.max([max1, max2], axis=0)
                min_total = np.min([min1, min2], axis=0)

                partition_size = (max_total - min_total) / number_of_partitions

                for i in range(len(mesh1.triangles)):
                    triangle = mesh1.triangles[i]
                    vertices = mesh1.vertices[triangle]
                    if np.any(vertices > max_cords2) or np.any(vertices < min_cords2):
                        continue
                    min_vertex = np.min(vertices, axis=0)
                    max_vertex = np.max(vertices, axis=0)
                    min_index = ((min_vertex - min_total) / partition_size).astype(int)
                    max_index = ((max_vertex - min_total) / partition_size).astype(int)
                    for x in range(min_index[0], max_index[0]+1):
                        for y in range(min_index[1], max_index[1]+1):
                            for z in range(min_index[2], max_index[2]+1):
                                space1[x, y, z].append(i)
                print(f"time2: {time.time()-time1:.2f}s")
                
                for i in range(len(mesh2.triangles)):
                    triangle = mesh2.triangles[i]
                    vertices = mesh2.vertices[triangle]
                    if np.any(vertices > max_cords1) or np.any(vertices < min_cords1):
                        continue
                    min_vertex = np.min(vertices, axis=0)
                    max_vertex = np.max(vertices, axis=0)
                    min_index = ((min_vertex - min_total) / partition_size).astype(int)
                    max_index = ((max_vertex - min_total) / partition_size).astype(int)
                    for x in range(min_index[0], max_index[0]+1):
                        for y in range(min_index[1], max_index[1]+1):
                            for z in range(min_index[2], max_index[2]+1):
                                space2[x, y, z].append(i)
                print(f"time3: {time.time()-time1:.2f}s")

                # for i in range(number_of_partitions):
                for i in tqdm.tqdm(range(number_of_partitions), desc="outer loop"):
                    for j in range(number_of_partitions):
                        for k in range(number_of_partitions):
                            for t1 in space1[i, j, k]:
                                for t2 in space2[i, j, k]:
                                    triangle1 = mesh1.triangles[t1]
                                    triangle2 = mesh2.triangles[t2]
                                    v1 = mesh1.vertices[triangle1]
                                    v2 = mesh2.vertices[triangle2]
                                    if Triangle3D(v1[0], v1[1], v1[2]).collides_triangle(Triangle3D(v2[0], v2[1], v2[2])):
                                        collisions[(uav1.name, uav2.name)] = True
                                        colision = True
                                        break
                                if colision:
                                    break
                            if colision:
                                break
                        if colision:
                            break
                    if colision:
                        break
                if not colision:
                    print(f"{uav1.name} does not collide with {uav2.name}")
                else:
                    print(f"{uav1.name} collides with {uav2.name}")
                print(f"Collision check: {time.time()-time1:.2f}s")

    def find_chull_collisions(self):
        import tqdm
        collisions = {}
        for uav1 in self.uavs.values():
            for uav2 in self.uavs.values():
                if uav1 == uav2:
                    continue
                if (uav1.name, uav2.name) in collisions or (uav2.name, uav1.name) in collisions:
                    continue
                created1 = False
                created2 = False
                if not uav1.boxes["chull"]:
                    created1 = True
                    uav1.create_convex_hull()
                if not uav2.boxes["chull"]:
                    created2 = True
                    uav2.create_convex_hull()
                import time
                time1 = time.time()
                collision = False

                # for t1 in uav1.boxes["chull"].triangles:
                print(f"Number of triangles1: {len(uav1.boxes['chull'].triangles)}")
                print(f"Number of triangles2: {len(uav2.boxes['chull'].triangles)}")
                triangle_3d_1 =[]
                triangle_3d_2 =[]
                max_cords_1 = np.max(uav1.mesh.vertices, axis=0)
                min_cords_1 = np.min(uav1.mesh.vertices, axis=0)
                max_cords_2 = np.max(uav2.mesh.vertices, axis=0)
                min_cords_2 = np.min(uav2.mesh.vertices, axis=0)
                for t1 in uav1.boxes["chull"].triangles:
                    v1 = uav1.boxes["chull"].vertices[t1]
                    if np.any(v1>max_cords_2) or np.any(v1<min_cords_2):
                        continue
                    triangle1 = Triangle3D(v1[0], v1[1], v1[2])
                    triangle_3d_1.append(triangle1)
                for t2 in uav2.boxes["chull"].triangles:
                    v2 = uav2.boxes["chull"].vertices[t2]
                    if np.any(v2>max_cords_1) or np.any(v2<min_cords_1):
                        continue
                    triangle2 = Triangle3D(v2[0], v2[1], v2[2])
                    triangle_3d_2.append(triangle2)
                for triangle1 in tqdm.tqdm(triangle_3d_1, desc="outer loop"):
                    for triangle2 in triangle_3d_2:
                        if triangle1.collides_triangle(triangle2):
                            print(f"{uav1.name} collides with {uav2.name}")
                            collisions[(uav1.name, uav2.name)] = True
                            collision = True
                            break
                    if collision:
                        break
                if not collision:
                    print(f"{uav1.name} does not collide with {uav2.name}")
                else:
                    print(f"{uav1.name} collides with {uav2.name}")


                print(f"Collision check: {time.time()-time1:.2f}s")
                if created1:
                    uav1.remove_convex_hull()
                if created2:
                    uav2.remove_convex_hull()
               

    def on_key_press(self, symbol, modifiers):

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
            for uav in self.uavs.values():
                if uav.boxes_visibility["kdop"]:
                    # print("removing kdop")
                    uav.remove_kdop()
                else:
                    # print("creating kdop")
                    uav.create_kdop(kdop_number)

        if symbol == Key.L:
            time1 = time.time()
            self.find_collisions(show=True)
            # osprey = self.uavs.get("v22_osprey_0")
            # other_uav = self.uavs.get("v22_osprey_1")
            # methods = [
            #     osprey.collides_aabb_node,
            #     osprey.collides_kdop,
            #     osprey.collides_convex_hull,
            #     # osprey.collides_mesh,
            #     osprey.collides_mesh_random,
            # ]
            # method_names = [
            #     "AABB Node",
            #     "Kdop",
            #     "Convex Hull",
            #     # "Mesh",
            #     "Mesh Random",
            # ]
            # for i, method in enumerate(methods):
            #     collision = False
            #     time2 = time.time()
            #     if method(other_uav):
            #         collision = True
            #     else:
            #         collision = False
            #     print(f"{method_names[i]}: {collision}, time: {time.time()-time2:.2f}s")


            # self.find_kdop_collisions()
            # self.find_aabb_collisions()
            # self.find_chull_collisions()
            # self.find_mesh_collisions_slow()
            # self.find_mesh_collisions_opt()
            # self.find_mesh_collisions_random()
            print(f"End of collision check: {time.time()-time1:.2f}s")
            
            

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

        start = time.time()

        points = []
        
        directions_to_vertices = {} # Mapping the each direction to the furthest vertex in that direction
        for direction in directions[:7]: # The other half will be the min vertices
            max_val = -np.inf
            max_vertex = None
            min_val = np.inf
            min_vertex = None
            for vertex in self.pc_vertices:
                val = np.dot(vertex, direction)
                if val > max_val:
                    max_val = val
                    max_vertex = vertex
                if val < min_val:
                    min_val = val
                    min_vertex = vertex
            directions_to_vertices[tuple(direction)] = max_vertex
            directions_to_vertices[tuple(np.array(direction)*-1)] = min_vertex
        

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
        print(f"Creating kdop named {self.name} took {time.time()-start:.2f}s")
        super().__init__(dop_polygons, color=Color.RED, name = self.name) 
        if self.scene:
            self.scene.addShape(self, self.name)


        
        

        print(f"14DOP: {time.time()-start:.2f}s")
    
    def create_from_class(self):
        import time

        print(f"Creating Kdop from class {self.uav._class}")
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
        print(f"Cached kdop creation time: {time.time()-time1:.2f}s")

        

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

def main():


    
    airspace = Airspace(1920, 1080, N = 5)




        
    airspace.mainLoop()

    


if __name__ == "__main__":
    main()