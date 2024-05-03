import open3d as o3d
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

        self.boxes=[]
        self.boxes_names = []

        self.position = np.mean(self.mesh.vertices, axis=0)
        if position:            
            self.move_to(position)
        
        self.scale(scale, fit_to_unit_sphere= (not scale))

        

        

        


        
    
    def move_to(self, position:np.ndarray):
        "Move the UAV to the specified position"
        dist = position - self.position
        self.mesh.vertices += dist
        self.position = position
        self.mesh._update(self.name, self.scene)

        for box, name in zip(self.boxes, self.boxes_names):
            [box.x, box.y, box.z] = self.position
            box._update(name, self.scene)
    
    def move_by(self, dist:np.ndarray):
        "Move the UAV by the specified distance"
        self.move_to(self.position + dist)

    def rotate(self, angle:float, axis:np.ndarray):
        "Rotate the UAV by the specified angle around the specified axis"
        R = get_rotation_matrix(angle, axis)
        self.mesh.vertices = np.dot(self.mesh.vertices, R)
        self.mesh._update(self.name, self.scene)

        for box, name in zip(self.boxes, self.boxes_names):
            if isinstance(box, Sphere3D):
                continue
            # Rotate boxes

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
        self.boxes.append(sphere)
        self.boxes_names.append(self.name+"_sphere")


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