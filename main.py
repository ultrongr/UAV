import open3d as o3d
import open3d.geometry as o3dg # type: ignore
import open3d.visualization.gui as gui # type: ignore
import open3d.visualization.rendering as rendering # type: ignore
from open3d.visualization.gui import MouseEvent, KeyEvent # type: ignore
from open3d.visualization.rendering import Camera # type: ignore
import numpy as np
from vvrpywork.constants import Key, Mouse, Color
from vvrpywork.scene import Scene2D, Scene3D, get_rotation_matrix, world_space
from vvrpywork.shapes import (
    Point2D, Line2D, Triangle2D, Circle2D, Rectangle2D,
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





class UAV:
    classes = {}

    def __init__(self, scene:Scene3D, filename:str, position:np.ndarray = None, scale:float = None):
        
        self.mesh = Mesh3D(filename)
        self.scene = scene

        self._class = filename.split("/")[-1].split(".")[0]
        if not self._class in UAV.classes:
            UAV.classes[self._class] = 0
        self.name = f"{self._class}_{UAV.classes[self._class]}"
        print(self.name)
        UAV.classes[self._class] += 1


        self.scene.addUAV(self)

        self.boxes = {
            "aabb": None,
            "chull": None,
            "sphere": None,
            "kdop": None,
        }

        self.position = np.mean(self.mesh.vertices, axis=0)
        if position:            
            self.move_to(position)
        
        self.scale(scale, fit_to_unit_sphere= (not scale))

    def __eq__(self, other):
        return self.name == other.name

    def update_boxes(self):
        create_methods_dict = {
            "aabb": self.create_aabb,
            "chull": self.create_convex_hull,
            "sphere": self.create_sphere,
        }
        remove_methods_dict = {
            "aabb": self.remove_aabb,
            "chull": self.remove_convex_hull,
            "sphere": self.remove_sphere,
        }
        for box_name in self.boxes.keys():
            if box_name:
                self.scene.removeShape(self.name+"_"+box_name)
                create_methods_dict[box_name]()
            else:
                continue
                
        
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
                self.remove_kdop()
                self.scene.addShape(box, self.name+"_kdop")
                self.boxes["kdop"] = box
                    
    
    def move_by(self, dist:np.ndarray):
        "Move the UAV by the specified distance"
        # print(self.boxes["kdop"])
        self.move_to(self.position + dist)

    def rotate(self, angle:float, axis:np.ndarray):
        "Rotate the UAV by the specified angle around the specified axis"
        R = get_rotation_matrix(angle, axis)
        self.mesh.vertices = np.dot(self.mesh.vertices, R)
        self.mesh._update(self.name, self.scene)



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
    
    def remove_sphere(self):
        self.scene.removeShape(self.name+"_sphere")
        self.boxes["sphere"] = None

    def create_aabb(self):
        "Create an axis-aligned bounding box around the UAV"


        min_x, min_y, min_z = np.min(self.mesh.vertices, axis=0)
        max_x, max_y, max_z = np.max(self.mesh.vertices, axis=0)
        box = Cuboid3D(p1=[min_x, min_y, min_z], p2=[max_x, max_y, max_z], color=Color.GREEN, filled=False)
        self.scene.addShape(box, self.name+"_aabb")
        self.boxes["aabb"] = box
    
    def remove_aabb(self):
        self.scene.removeShape(self.name+"_aabb")
        self.boxes["aabb"] = None

    def create_convex_hull(self):
        "Create a convex hull around the UAV"


        


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
        print(f"Hull: {time.time()-start:.2f}s")

    def remove_convex_hull(self):
        self.scene.removeShape(self.name+"_chull")
        self.boxes["chull"] = None
    
    def create_kdop(self, k:int):
        "Create a k-dop around the UAV"
        
        implemented_values = [6, 14, 26]
        if k not in implemented_values:
            print(f"Only {implemented_values} are implemented")
            return
        if k == 6:
            self.create_6dop()
            return
        if k == 14:
            self.create_14dop()
            return
        
        


    def create_6dop(self):
        "Create a 6-dop around the UAV"

        min_x, min_y, min_z = np.min(self.mesh.vertices, axis=0)
        max_x, max_y, max_z = np.max(self.mesh.vertices, axis=0)
        # box = Cuboid3D(p1=[min_x, min_y, min_z], p2=[max_x, max_y, max_z], color=Color.GREEN, filled=False)
        # self.scene.addShape(box, self.name+"_kdop")
        # self.boxes["kdop"] = box
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
        kdop = Polyhedron3D(polygons, color=Color.RED, name = self.name+"_kdop")
        self.scene.addShape(kdop, self.name+"_kdop")
        self.boxes["kdop"] = kdop


    def create_14dop(self):
        "Create a 14-dop around the UAV"

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
            for vertex in self.mesh.vertices:
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


        mesh_point = np.mean(self.mesh.vertices, axis=0) # The center of the mesh, used to check if other points are inside the kdop
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
            face_polygon = ConvexPolygon3D(dop_face_points, normal=direction,color=Color.RED) # The dace of the kdop corresponding to the direction
            dop_polygons.append(face_polygon)
        
        kdop = Polyhedron3D(dop_polygons, color=Color.RED, name = self.name + "_kdop") # The kdop
        self.scene.addShape(kdop, self.name+"_kdop")
        self.boxes["kdop"] = kdop

                
        

        print(f"14DOP: {time.time()-start:.2f}s")

      
    def remove_kdop(self):
        if not self.boxes.get("kdop"):
            return
        kdop = self.boxes["kdop"]
        kdop_name = self.name+"_kdop"
        self.scene.removeShape(self.name+"_kdop")
        
        for i, face in enumerate(kdop._polygons):
            self.scene.removeShape(kdop_name+f"_face_{i}")
        
        self.boxes["kdop"] = None

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
        # self.landing_pad = LandingPad(N, self)
        # self.create_uavs()
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
        uav2 = UAV(self, filename2, position=[0, 1, 0], scale=None)

    
    def find_kdop_collisions(self):
        collisions = {}
        for uav1 in self.uavs.values():
            for uav2 in self.uavs.values():
                if uav1 == uav2:
                    continue
                if (uav1.name, uav2.name) in collisions or (uav2.name, uav1.name) in collisions:
                    continue
                created1 = False
                created2 = False
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
                    collisions[(uav1.name, uav2.name)] = True
                else:
                    print(f"{uav1.name} does not collide with {uav2.name}")
                print(f"Collision check: {time.time()-time1:.2f}s")
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
                for t1 in tqdm.tqdm(range(len(mesh1.triangles)), desc="outer loop"):
                    for t2 in tqdm.tqdm(range(len(mesh2.triangles)), desc="inner loop"):
                        
                        triangle1 = mesh1.triangles[t1]
                        triangle2 = mesh2.triangles[t2]
                        v1 = mesh1.vertices[triangle1]
                        v2 = mesh2.vertices[triangle2]
                        dist = np.linalg.norm(np.mean(v1, axis=0) - np.mean(v2, axis=0))
                        if dist > dist_cutoff:
                            continue
                        if Triangle3D(v1[0], v1[1], v1[2]).collides_triangle(Triangle3D(v2[0], v2[1], v2[2])):
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

    def find_mesh_collisions_opt(self):
        import tqdm
        collisions = {}
        checked_pairs = {}

        number_of_batches=2000
        print(f"Optimized collision check, number of pieces: {number_of_batches}")

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

                batch_size = len(mesh1.triangles)//number_of_batches
                batches1_min_cords=[]
                batches1_max_cords=[]
                batches2_min_cords=[]
                batches2_max_cords=[]
                for i in range(0, len(mesh1.triangles), batch_size):
                    batch_triangles = mesh1.triangles[i:i+batch_size]
                    indices_array = np.array(batch_triangles).flatten()
                    batch_vertices = mesh1.vertices[indices_array]
                    min_x = np.min(batch_vertices[:, 0])
                    min_y = np.min(batch_vertices[:, 1])
                    min_z = np.min(batch_vertices[:, 2])
                    max_x = np.max(batch_vertices[:, 0])
                    max_y = np.max(batch_vertices[:, 1])
                    max_z = np.max(batch_vertices[:, 2])
                    batches1_min_cords.append([min_x, min_y, min_z])
                    batches1_max_cords.append([max_x, max_y, max_z])

                for i in range(0, len(mesh2.triangles), batch_size):
                    batch_triangles = mesh2.triangles[i:i+batch_size]
                    indices_array = np.array(batch_triangles).flatten()
                    batch_vertices = mesh2.vertices[indices_array]
                    min_x = np.min(batch_vertices[:, 0])
                    min_y = np.min(batch_vertices[:, 1])
                    min_z = np.min(batch_vertices[:, 2])
                    max_x = np.max(batch_vertices[:, 0])
                    max_y = np.max(batch_vertices[:, 1])
                    max_z = np.max(batch_vertices[:, 2])
                    batches2_min_cords.append([min_x, min_y, min_z])
                    batches2_max_cords.append([max_x, max_y, max_z])
                
                batches1_min_cords = np.array(batches1_min_cords)
                batches1_max_cords = np.array(batches1_max_cords)
                batches2_min_cords = np.array(batches2_min_cords)
                batches2_max_cords = np.array(batches2_max_cords)


                # for i in tqdm.tqdm(range(len(batches1_min_cords)), desc="outer loop"):
                for i in range(len(batches1_min_cords)):
                    
                    # for j in tqdm.tqdm(range(len(batches2_min_cords)), desc="inner loop"):
                    for j in range(len(batches2_min_cords)):
                        min1 = batches1_min_cords[i]
                        max1 = batches1_max_cords[i]
                        min2 = batches2_min_cords[j]
                        max2 = batches2_max_cords[j]
                        if not (min1[0] < max2[0] and max1[0] > min2[0] and
                                min1[1] < max2[1] and max1[1] > min2[1] and
                                min1[2] < max2[2] and max1[2] > min2[2]):
                            continue
                        for t1 in range(i*batch_size, (i+1)*batch_size):
                            for t2 in range(j*batch_size, (j+1)*batch_size):
                                triangle1 = mesh1.triangles[t1]
                                triangle2 = mesh2.triangles[t2]
                                v1 = mesh1.vertices[triangle1]
                                v2 = mesh2.vertices[triangle2]
                                if Triangle3D(v1[0], v1[1], v1[2]).collides_triangle(Triangle3D(v2[0], v2[1], v2[2])):
                                    print(f"{uav1.name} collides with {uav2.name}")
                                    collisions[(uav1.name, uav2.name)] = True
                                    colision = True
                                    break
                            else:
                                continue
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
        print("End of optimized collision check")


    def find_mesh_collisions_opt_new(self):
        import tqdm
        collisions = {}
        checked_pairs = {}

        number_of_partitions: int = 2000
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

                
                partitions_space = np.linspace(0, 4, number_of_partitions+1)
                partitions1 = [[] for _ in range(number_of_partitions)]
                partitions2 = [[] for _ in range(number_of_partitions)]


                mesh1 = uav1.mesh
                mesh2 = uav2.mesh

                minx1 = np.min(mesh1.vertices[:, 0])
                minx2 = np.min(mesh2.vertices[:, 0])
                maxx1 = np.max(mesh1.vertices[:, 0])
                maxx2 = np.max(mesh2.vertices[:, 0])
                print(minx1, minx2)
                print(maxx1, maxx2)
                total_minx = min(minx1, minx2)
                step = 4/number_of_partitions

                for triangle in mesh1.triangles:
                    vertices = mesh1.vertices[triangle]
                    minx = np.min(vertices[:, 0])
                    maxx = np.max(vertices[:, 0])
                    partition_start = int((minx - total_minx)//step)
                    partition_end = int((maxx - total_minx)//step)
                    for i in range(partition_start, partition_end):
                        try:
                            partitions1[i].append(triangle)
                        except:
                            print(partition_start, partition_end)
                            raise
                
                for triangle in mesh2.triangles:
                    vertices = mesh2.vertices[triangle]
                    minx = np.min(vertices[:, 0])
                    maxx = np.max(vertices[:, 0])
                    partition_start = int((minx - total_minx)//step)
                    partition_end = int((maxx - total_minx)//step)
                    for i in range(partition_start, partition_end):
                        partitions2[i].append(triangle)
                
                # print(partitions1)
                # for p in partitions1:
                #     print(len(p))
                
                # return
                # for i in range(number_of_partitions):
                for i in tqdm.tqdm(range(number_of_partitions), desc="outer loop"):
                    # for t1 in partitions1[i]:
                    for t1 in tqdm.tqdm(partitions1[i], desc="inner loop"):
                        
                        for t2 in partitions2[i]:
                            triangle1 = mesh1.vertices[t1]
                            triangle2 = mesh2.vertices[t2]
                            if Triangle3D(triangle1[0], triangle1[1], triangle1[2]).collides_triangle(Triangle3D(triangle2[0], triangle2[1], triangle2[2])):
                                print(f"{uav1.name} collides with {uav2.name}")
                                collisions[(uav1.name, uav2.name)] = True
                                colision = True
                                break
                        if colision:
                            break
                    if colision:
                        break
                if not colision:
                    print(f"{uav1.name} does not collide with {uav2.name}")
                else:
                    print(f"{uav1.name} collides with {uav2.name}")


        







    def on_key_press(self, symbol, modifiers):

        if symbol == Key.C:
            print("c")
            for uav in self.uavs.values():
                if uav.boxes["chull"]:
                    uav.remove_convex_hull()
                else:
                    uav.create_convex_hull()
        
        if symbol == Key.U:
            for uav in self.uavs.values():
                if uav.boxes["sphere"]:
                    uav.remove_sphere()
                else:
                    uav.create_sphere(radius=None, resolution=30)
        
        if symbol == Key.B:
            for uav in self.uavs.values():
                if uav.boxes["aabb"]:
                    uav.remove_aabb()
                else:
                    uav.create_aabb()
        
        if symbol == Key.K:
            for uav in self.uavs.values():
                if uav.boxes["kdop"]:
                    # print("removing kdop")
                    uav.remove_kdop()
                else:
                    # print("creating kdop")
                    uav.create_kdop(kdop_number)

        if symbol == Key.L:
            # self.find_kdop_collisions()
            # self.find_aabb_collisions()
            # self.find_mesh_collisions_slow()
            # self.find_mesh_collisions_opt()
            self.find_mesh_collisions_opt_new()
            

        osprey = self.uavs["v22_osprey_0"]
        if symbol in [Key.UP, Key.DOWN, Key.LEFT, Key.RIGHT, Key.SPACE, Key.BACKSPACE]:
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
            # osprey.create_kdop(kdop_number)
            # self.find_kdop_collisions()
    
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


       
                
            

def main():


    
    airspace = Airspace(1920, 1080, N = 1)

    # for i,model in enumerate(models):
    #     uav = UAV(airspace, f"models/{model}.obj", position=[2*i, 1, 0], scale=None)
    #     # uav.create_sphere(radius=None, resolution=30)
    
    # landing_pad = LandingPad(N, airspace)


        
    airspace.mainLoop()

    


if __name__ == "__main__":
    main()