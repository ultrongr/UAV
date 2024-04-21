import open3d as o3d
import open3d.visualization.gui as gui # type: ignore
import open3d.visualization.rendering as rendering # type: ignore
from open3d.visualization.gui import MouseEvent, KeyEvent # type: ignore
from open3d.visualization.rendering import Camera # type: ignore
import numpy as np

defaultUnlit = rendering.MaterialRecord()
defaultUnlit.shader = "defaultUnlit"

unlitLine = rendering.MaterialRecord()
unlitLine.shader = "unlitLine"
unlitLine.line_width = 5



# obj_file_path = "models/F52.obj"
obj_file_path = "models/v22_osprey.obj"


mesh = o3d.io.read_triangle_mesh(obj_file_path)

# Check if the mesh was successfully loaded
if not mesh:
    print(f"Error: Unable to load mesh from {obj_file_path}")
else:
    print(f"Mesh successfully loaded from {obj_file_path}")


centroid = np.mean(mesh.vertices, axis=0)
translation_vector = -centroid  # Vector to move centroid to origin
mesh.translate(translation_vector)

# Print the new centroid
centroid = np.mean(mesh.vertices, axis=0)
print(f"Mesh centroid: {centroid}")

# Scale it down 
scale = float(1.2 / np.max(np.max(mesh.vertices, axis=0) - np.min(mesh.vertices, axis=0)))
mesh = mesh.scale(scale, center=centroid)




class Appwindow:

    def __init__(self, width, height, window_name="UAV"):
        self.w_width = width
        self.w_height = height
        self.first_click = True

        # initialize window & scene
        self.window = gui.Application.instance.create_window(window_name, width, height) 
        self._scene = gui.SceneWidget()
        self._scene.scene = rendering.Open3DScene(self.window.renderer)
        self._scene.scene.show_skybox(True)

        # basic layout
        self.window.set_on_layout(self._on_layout)
        self.window.add_child(self._scene)

        # set mouse and key callbacks
        self._scene.set_on_key(self._on_key_pressed)
        # self._scene.set_on_mouse(self._on_mouse_pressed)

        # set up camera
        self._set_camera()
        
        self.geometries = {}

        # parameters 
        self.model_on       = True
        self.wireframe_on   = False
        self.sphere_on      = False
        self.aabb_on        = False
        self.axis_on        = False
        self.plane_on       = False

        self.plane_pos_y    = 0.0
        self.plane_angle_x  = 0.0
        self.plane_angle_z  = 0.0
        
        self.plane_pos  = np.array([0,0,0])
        self.plane_dir  = np.array([0,1,0])

        self.task_executed = False
    
    def _on_layout(self, layout_context):
        
        r = self.window.content_rect
        self._scene.frame = r

    def _set_camera(self):

        bounds = self._scene.scene.bounding_box
        center = bounds.get_center()
        self._scene.look_at(center, center - [0, 0, 5], [0, 1, 0])

    def add_geometry(self, geometry, name, shader = defaultUnlit, visible = True):
    
        self._scene.scene.add_geometry(name, geometry, shader)
        self.geometries[name] = geometry

        self._scene.scene.show_geometry(name, visible)

    def remove_geometry(self, name):

        self._scene.scene.remove_geometry(name)

    def _on_key_pressed(self, event):

        if not event.type == event.DOWN:
            return gui.Widget.EventCallbackResult.IGNORED

        # U key 
        if event.key == 117: 
                
            if "unit_sphere" in self.geometries:

                self.sphere_on = not self.sphere_on
                self._scene.scene.show_geometry("unit_sphere", self.sphere_on)
                
                return gui.Widget.EventCallbackResult.HANDLED
        
        # M key 
        if event.key == 109: 
            print("M key pressed")
                
            self.model_on = not self.model_on
            self._scene.scene.show_geometry("model", self.model_on)
            
            return gui.Widget.EventCallbackResult.HANDLED
        
        return gui.Widget.EventCallbackResult.IGNORED

def main():
    global mesh

    

    gui.Application.instance.initialize()

    app = Appwindow(1920, 1080)

    unit_sphere = o3d.geometry.LineSet.create_from_triangle_mesh(
        o3d.geometry.TriangleMesh.create_sphere(radius=1.0)).paint_uniform_color(np.array([1,0,0]))


    app.add_geometry(unit_sphere, "unit_sphere", visible = True)


    app.add_geometry(mesh, "model")

    


    gui.Application.instance.run()

if __name__ == "__main__":
    main()