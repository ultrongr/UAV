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
    PointSet3D, LineSet3D, Mesh3D, Triangle3D
)

defaultUnlit = rendering.MaterialRecord()
defaultUnlit.shader = "defaultUnlit"

unlitLine = rendering.MaterialRecord()
unlitLine.shader = "unlitLine"
unlitLine.line_width = 5

models = ["v22_osprey",
        "B2_Spirit",
        "F52",
        "fight_drone",
        "Helicopter",
        "quadcopter_scifi",
        "twin_copter",
        
    ]





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
                
        
    def move_to(self, position:np.ndarray):
        "Move the UAV to the specified position"
        dist = position - self.position
        self.mesh.vertices += dist
        self.position = position
        self.mesh._update(self.name, self.scene)

    
    def move_by(self, dist:np.ndarray):
        "Move the UAV by the specified distance"
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


        def simple_hull():
            
            # Find one triangle that is part of the convex hull
            faces=[]
            for t in self.mesh.triangles:
                p1, p2, p3 = self.mesh.vertices[t]
                for v in self.mesh.vertices:
                    if np.all(v == p1) or np.all(v == p2) or np.all(v == p3):
                        continue
                    if np.linalg.det(np.array([p2-p1, p3-p1, v-p1])) > 0:
                        faces.append([p1, p2, p3])
                        break
                else:
                    continue
                break
            queue = []
            faces_points = {}
            for face in faces:
                for p in face:
                    faces_points[tuple(p)] = True
            first = faces[0]
            queue.append((first[0], first[1]))
            queue.append((first[1], first[2]))
            queue.append((first[2], first[0]))



            counter=0
            while queue:
                
                p1, p2 = queue.pop(0)
                counter=0
                for p3 in self.mesh.vertices:
                    counter+=1
                    if tuple(p3) in faces_points:
                        continue
                    if any([np.linalg.det(np.array([p2-p1, p3-p1, v-p1])) > 0 for v in self.mesh.vertices]):
                        if any([np.linalg.det(np.array([p1-p2, p3-p2, v-p2])) > 0 for v in self.mesh.vertices]):
                            continue
                    
                    faces.append([p1, p2, p3])
                    faces_points[tuple(p3)] = True
                    queue.append((p1, p3))
                    queue.append((p3, p2))
                    break
                else:
                    print("error")
                    break
            print(len(faces))
                    


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
        
        
        if k == 6:
            directions = [
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1],
                [-1, 0, 0],
                [0, -1, 0],
                [0, 0, -1],
            ]
        elif k == 14:
            directions = [
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1],
                [-1, 0, 0],
                [0, -1, 0],
                [0, 0, -1],
                [1, 1, 1],
                [1, 1, -1],
                [1, -1, 1],
                [1, -1, -1],
                [-1, 1, 1],
                [-1, 1, -1],
                [-1, -1, 1],
                [-1, -1, -1]
            ]
        elif k==26:
            directions = [
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1],
                [-1, 0, 0],
                [0, -1, 0],
                [0, 0, -1],
                [1, 1, 1],
                [1, 1, -1],
                [1, -1, 1],
                [1, -1, -1],
                [-1, 1, 1],
                [-1, 1, -1],
                [-1, -1, 1],
                [-1, -1, -1],
                [1, 1, 0],
                [1, -1, 0],
                [1, 0, 1],
                [1, 0, -1],
                [0, 1, 1],
                [0, 1, -1],
                [-1, 1, 0],
                [-1, -1, 0],
                [0, -1, 1],
                [0, -1, -1],
                [-1, 0, 1],
                [-1, 0, -1],
            ]
        
        edge_points = []
        
        for direction in directions:
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
            edge_points.append(min_vertex)
            edge_points.append(max_vertex)
    

    def create_6dop(self):
        "Create a 6-dop around the UAV"

        min_x, min_y, min_z = np.min(self.mesh.vertices, axis=0)
        max_x, max_y, max_z = np.max(self.mesh.vertices, axis=0)
        box = Cuboid3D(p1=[min_x, min_y, min_z], p2=[max_x, max_y, max_z], color=Color.GREEN, filled=False)
        self.scene.addShape(box, self.name+"_kdop")
        self.boxes["kdop"] = box


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
            [-1, 0, 0],
            [0, -1, 0],
            [0, 0, -1],
            [1, 1, 1],
            [1, 1, -1],
            [1, -1, 1],
            [1, -1, -1],
            [-1, 1, 1],
            [-1, 1, -1],
            [-1, -1, 1],
            [-1, -1, -1]
        ]

        start = time.time()

        points = []
        lines = []
        triangles = []
        directions_to_vertices = {}
        for direction in directions:
            max_val = -np.inf
            max_vertex = None
            for vertex in self.mesh.vertices:
                val = np.dot(vertex, direction)
                if val > max_val:
                    max_val = val
                    max_vertex = vertex
            points.append(max_vertex)
            directions_to_vertices[tuple(direction)] = max_vertex
        
        print("intermediate time", time.time()-start)
        
        for dir in directions_to_vertices.keys():
            
            if 0 in dir: # Only going for corners
                continue
            v = directions_to_vertices[dir]
            plane = plane_equation_from_point_normal(v, dir)
            face_planes = []
            for dir2 in directions_to_vertices.keys():
                if not 0 in dir2: # Only going for faces
                    continue
                # Check if the corner is a neighbor of the face
                if not check_if_neighbor(dir, dir2):
                    continue

                plane2 = plane_equation_from_point_normal(directions_to_vertices[dir2], dir2)
                face_planes.append(plane2)
            print(face_planes)
            temp_points=[]
            point = None
            for plane2 in face_planes:
                for plane3 in face_planes:
                    if plane2 == plane3:
                        continue
                    point = find_intersection(plane, plane2, plane3)
                    for temp_p in temp_points:
                        if np.linalg.norm(temp_p - point) < 0.01:
                            break
                    else:
                        points.append(point)
                        temp_points.append(point)
            print(len(temp_points), "temp", )
            # triangle = Mesh3D(None, color=Color.RED)
            # triangle._shape.vertices = o3d.utility.Vector3dVector(temp_points)
            # triangle._shape.triangles = o3d.utility.Vector3iVector([[0, 1, 2]])
            triangle = Triangle3D(p1=temp_points[0], p2=temp_points[1], p3=temp_points[2], color=Color.RED)
            triangles.append(triangle)
            self.scene.addShape(triangle, self.name+"_triangle"+str(len(triangles)-1))
            for p in temp_points:
                print(p)
            lines.append((len(points)-1, len(points)-2))
            lines.append((len(points)-1, len(points)-3))
            lines.append((len(points)-2, len(points)-3))

        # mesh_point=self.mesh.vertices[100]
        mesh_point = np.mean(self.mesh.vertices, axis=0)
        self.scene.addShape(Point3D(p=mesh_point, size=10, color=Color.RED), self.name+"_mesh_point")
        points_to_remove = []
        valid_points = []
        for p in points:
            
            for triangle in triangles:
                if not triangle.points_on_same_side(mesh_point, p):
                    points_to_remove.append(p)
                    break
            else:
                if len(valid_points)==18:
                    interested = []

                valid_points.append(p)
        pass
        for i,p in enumerate(valid_points):
            self.scene.addShape(Point3D(p=p, size=3, color=Color.BLACK), self.name+"_point_valid"+str(i))
        
        for i,p in enumerate(points_to_remove):
            self.scene.addShape(Point3D(p=p, size=3, color=Color.YELLOW), self.name+"_point_remove"+str(i))
        intersections=[]
        return
        for triangle in triangles:
            for other_triangle in triangles:
                if triangle == other_triangle:
                    continue
                if not triangle.intersects(other_triangle):
                    continue
                points1=[triangle.p1, triangle.p2, triangle.p3]
                points2=[other_triangle.p1, other_triangle.p2, other_triangle.p3]
                lines1 = [Line3D(p1=points1[i], p2=points1[j]) for i in range(3) for j in range(i+1, 3)]
                lines2 = [Line3D(p1=points2[i], p2=points2[j]) for i in range(3) for j in range(i+1, 3)]
                for line1 in lines1:
                    for line2 in lines2:
                        intersection = line1.intersectsLine(line2) 
                        if intersection is not None:
                            points.append(intersection)
                            intersections.append(intersection)
                            self.scene.addShape(Point3D(p=intersection, size=3, color=Color.GREEN), self.name+"_intersection"+str(len(intersections)-1))
        

        

        print(f"14DOP: {time.time()-start:.2f}s")

                    

                    
                    
                
                
                
        


                    

        
        
                


    def remove_kdop(self):
        self.scene.removeShape(self.name+"_kdop")
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
        self.uavs = {}
        self.N = N
        self.landing_pad = LandingPad(N, self)
        self.create_uavs()

    def create_uavs(self):
        for i in range(self.N):
            for j in range(self.N):
                model = models[(i+j)%len(models)]
                filename = f"models/{model}.obj"
                uav = UAV(self, filename, position=[2*i+1, 1, 2*j+1], scale=None)
                # uav.create_sphere(radius=None, resolution=30)
                # uav.create_convex_hull()
    





    def on_key_press(self, symbol, modifiers):
        # uav = self.uavs.get("v22_osprey_0")

        # if not uav:
        #     return


        # if symbol == Key.UP:
        #     uav.move_to(uav.position + np.array([0, 0, -1]))
        # elif symbol == Key.DOWN:
        #     uav.move_to(uav.position + np.array([0, 0, 1]))
        # elif symbol == Key.LEFT:
        #     uav.move_to(uav.position + np.array([-1, 0, 0]))
        # elif symbol == Key.RIGHT:
        #     uav.move_to(uav.position + np.array([1, 0, 0]))
        # elif symbol == Key.SPACE:
        #     uav.move_to(uav.position + np.array([0, 1, 0]))
        # elif symbol == Key.BACKSPACE:
        #     uav.move_to(uav.position + np.array([0, -1, 0]))
        
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
                    uav.remove_kdop()
                else:
                    uav.create_kdop(14)

    
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