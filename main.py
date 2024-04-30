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





class UAV:
    number_of_uavs = 0

    def __init__(self, scene:Scene3D, filename:str, position:np.ndarray = None, scale:float = None):
        
        self.mesh = Mesh3D(filename)
        self.scene = scene
        self.name = filename.split("/")[-1].split(".")[0]+str(UAV.number_of_uavs)
        UAV.number_of_uavs += 1

        self.boxes=[]
        self.boxes_names = []

        self.position = np.mean(self.mesh.vertices, axis=0)
        if position:            
            self.move_to(position)
        
        self.scale(scale, fit_to_unit_sphere= (not scale))

        self.scene.addUAV(self)

        

        


        
    
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

        
    



class Airspace(Scene3D):

    def __init__(self, width, height, window_name="UAV"):
        super().__init__(width, height, window_name)
        self.uavs = {}


    def on_key_press(self, symbol, modifiers):
        uav = self.uavs.get("v22_osprey0")
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
        self.updateShape(uav.mesh, uav.name)


       
                
            

def main():
    
    airspace = Airspace(1920, 1080)


    uav = UAV(airspace, "models/twin_copter.obj", position=[0, 0, 0], scale=None)
    

    uav.create_sphere(radius=None, resolution=30)

    uav.move_to(np.array((0, 0, 1)))

    airspace.mainLoop()

    


if __name__ == "__main__":
    main()