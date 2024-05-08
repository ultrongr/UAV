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
    PointSet3D, LineSet3D, Mesh3D
)

defaultUnlit = rendering.MaterialRecord()
defaultUnlit.shader = "defaultUnlit"

unlitLine = rendering.MaterialRecord()
unlitLine.shader = "unlitLine"
unlitLine.line_width = 5

models = ["B2_Spirit",
        "F52",
        "fight_drone",
        "Helicopter",
        "quadcopter_scifi",
        "twin_copter",
        "v22_osprey",
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
                uav.create_convex_hull()
    





    def on_key_press(self, symbol, modifiers):
        uav = self.uavs.get("v22_osprey_0")

        if not uav:
            return


        if symbol == Key.UP:
            uav.move_to(uav.position + np.array([0, 0, -1]))
        elif symbol == Key.DOWN:
            uav.move_to(uav.position + np.array([0, 0, 1]))
        elif symbol == Key.LEFT:
            uav.move_to(uav.position + np.array([-1, 0, 0]))
        elif symbol == Key.RIGHT:
            uav.move_to(uav.position + np.array([1, 0, 0]))
        elif symbol == Key.SPACE:
            uav.move_to(uav.position + np.array([0, 1, 0]))
        elif symbol == Key.BACKSPACE:
            uav.move_to(uav.position + np.array([0, -1, 0]))
        
        if symbol == Key.C:
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

    N = 5
    
    airspace = Airspace(1920, 1080, N = 5)

    # for i,model in enumerate(models):
    #     uav = UAV(airspace, f"models/{model}.obj", position=[2*i, 1, 0], scale=None)
    #     # uav.create_sphere(radius=None, resolution=30)
    
    # landing_pad = LandingPad(N, airspace)


        
    airspace.mainLoop()

    


if __name__ == "__main__":
    main()