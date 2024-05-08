'''Implements objects to represent 2D and 3D scenes containing shapes.'''

import ctypes
from inspect import getfullargspec
from io import StringIO
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui # type: ignore
import open3d.visualization.rendering as rendering # type: ignore
import os
import pyglet

from vvrpywork.constants import Key

if os.name == "nt":
    ctypes.windll.user32.SetProcessDPIAware()

class Scene2D:
    '''A class representing a 2D Scene.'''
    def __init__(self, width=None, height=None, caption=None, resizable=False):
        self._window = pyglet.window.Window(width, height, caption, resizable)
        pyglet.gl.glClearColor(0.7,0.7,0.7,1)
        self._window.view = pyglet.math.Mat4((width/200, 0, 0, 0, 0, height/200, 0, 0, 0, 0, 1, 0, width/2, height/2, 0, 1))

        self._shapeDict = {}
        self._shapeBatch = pyglet.graphics.Batch()

        self._window.on_draw = self.__on_draw
        self._window.on_mouse_press = self.__on_mouse_press
        self._window.on_mouse_drag = self.__on_mouse_drag
        self._window.on_mouse_release = self.__on_mouse_release
        self._window.on_key_press = self.__on_key_press

        self.__on_draw = self._window.event(self.__on_draw)
        self.__on_mouse_press = self._window.event(self.__on_mouse_press)
        self.__on_mouse_drag = self._window.event(self.__on_mouse_drag)
        self.__on_mouse_release = self._window.event(self.__on_mouse_release)
        self.__on_key_press = self._window.event(self.__on_key_press)

    def __on_draw(self):
        self._window.clear()
        self._shapeBatch.draw()

    def __on_mouse_press(self, x, y, button, modifiers):
        x = x * 2 / self._window.width - 1
        y = y * 2 / self._window.height - 1
        self.on_mouse_press(x, y, button, modifiers)

    def on_mouse_press(self, x, y, button, modifiers):
        pass

    def __on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        x = x * 2 / self._window.width - 1
        y = y * 2 / self._window.height - 1
        self.on_mouse_drag(x, y, dx, dy, buttons, modifiers)

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        pass

    def __on_mouse_release(self, x, y, button, modifiers):
        x = x * 2 / self._window.width - 1
        y = y * 2 / self._window.height - 1
        self.on_mouse_release(x, y, button, modifiers)

    def on_mouse_release(self, x, y, button, modifiers):
        pass

    def __on_key_press(self, symbol, modifiers):
        if symbol == Key.ESCAPE:
            self._window.close()
        self.on_key_press(symbol, modifiers)

    def on_key_press(self, symbol, modifiers):
        pass

    def mainLoop(self, max_fps=60):
        pyglet.app.run(interval=1/max_fps)

    def addShape(self, shape, name=None):
        # check type of shape
        shape._addToScene(self, name)

    def updateShape(self, name):
        if name in self._shapeDict:
            self._shapeDict[name]["class"]._update(self._shapeDict[name]["shape"], self)

    def removeShape(self, name):
        if name in self._shapeDict:
            # self._shapeDict[name].delete()
            del self._shapeDict[name]


class Scene3D:
    # drawList = []
    def __init__(self, width, height, caption, output=False, n_sliders=0) -> None:
        gui.Application.instance.initialize()

        self.window = gui.Application.instance.create_window(caption, width, height)
        w = self.window

        em = w.theme.font_size
        
        self.scene_widget = gui.SceneWidget()
        self.scene_widget.scene = rendering.Open3DScene(w.renderer)
        self.scene_widget.scene.set_background((0.7, 0.7, 0.7, 1))

        self.scene_widget.set_view_controls(gui.SceneWidget.Controls.ROTATE_CAMERA)

        #set up camera
        bounds = self.scene_widget.scene.bounding_box
        center = bounds.get_center()
        self.scene_widget.look_at(center, center + [0, 0, 1.5], [0, 1, 0])

        self.scene_widget.scene.camera.set_projection(90, 1, 0.01, 3.75, rendering.Camera.FovType.Vertical)  # defaults except for near_plane


        if output or n_sliders > 0:
            self.scene_widget.frame = gui.Rect(200, w.content_rect.y,
                                            w.content_rect.width - 200, w.content_rect.height)
        else:
            self.scene_widget.frame = gui.Rect(w.content_rect.x, w.content_rect.y,
                                            w.content_rect.width, w.content_rect.height)

        self.scene_widget.scene.show_axes(True)

        if output or n_sliders > 0:
            gui_layout = gui.Vert(0.5 * em, gui.Margins(0.5 * em, 0.5 * em, 0.5 * em, 0.5 * em))
            gui_layout.frame = gui.Rect(w.content_rect.x, w.content_rect.height - (n_sliders * 26 + (n_sliders + 1) * 0.5 * em),
                                        200, n_sliders * 26 + (n_sliders + 1) * 0.5 * em)
            gui_layout.background_color = gui.Color(0.1, 0.1, 0.1)


            self._sliders = []
            for i in range(n_sliders):
                slider = gui.Slider(gui.Slider.Type.DOUBLE)
                slider.set_limits(0, 1)
                slider.set_on_value_changed(lambda v, i=i: self.on_slider_change(i, v))
                gui_layout.add_child(slider)
                self._sliders.append(slider)


            text_layout = gui.Vert(0.5 * em, gui.Margins(0.5 * em, 0.5 * em, 0.5 * em, 0.5 * em))
            text_layout.frame = gui.Rect(w.content_rect.x, w.content_rect.y,
                                        200, w.content_rect.height - (n_sliders * 26 + (n_sliders + 1) * 0.5 * em))
            text_layout.background_color = gui.Color(0.2, 0.2, 0.2)


            self.sio = StringIO()

            self.text_output = gui.Label("")
            self.text_output.text_color = gui.Color(1, 1, 1)
            text_layout.add_child(self.text_output)

            w.add_child(text_layout)
            w.add_child(gui_layout)
        w.add_child(self.scene_widget)

        self.scene_widget.set_on_mouse(lambda mouseEvent: self.mouseEventToFunction(mouseEvent))
        self.window.set_on_key(lambda keyEvent: gui.Application.instance.post_to_main_thread(self.window, lambda: self.keyEventToFunction(keyEvent)))
        self.window.set_on_tick_event(lambda: gui.Application.instance.post_to_main_thread(self.window, self.on_idle))
        # self.window.set_on_tick_event(self.on_idle)
        self.modifiers = gui.KeyModifier.NONE.value
        self.last_coords = np.array((0., 0.))

        self._shapeDict = {}

    
    def mainLoop(self):
        gui.Application.instance.run()

    def on_key_press(self, symbol, modifiers):
        pass
    
    def on_key_release(self, symbol, modifiers):
        pass
    
    def on_slider_change(self, slider_id, value):
        return True
    
    def on_idle(self):
        return False

    _key_to_symbol = {
        gui.KeyName.BACKSPACE: pyglet.window.key.BACKSPACE,
        gui.KeyName.TAB: pyglet.window.key.TAB,
        gui.KeyName.ENTER: pyglet.window.key.ENTER,
        gui.KeyName.ESCAPE: pyglet.window.key.ESCAPE,
        gui.KeyName.SPACE: pyglet.window.key.SPACE,
        gui.KeyName.EXCLAMATION_MARK: pyglet.window.key.EXCLAMATION,
        gui.KeyName.DOUBLE_QUOTE: pyglet.window.key.DOUBLEQUOTE,
        gui.KeyName.HASH: pyglet.window.key.HASH,
        gui.KeyName.DOLLAR_SIGN: pyglet.window.key.DOLLAR,
        gui.KeyName.PERCENT: pyglet.window.key.PERCENT,
        gui.KeyName.AMPERSAND: pyglet.window.key.AMPERSAND,
        gui.KeyName.QUOTE: pyglet.window.key.APOSTROPHE,
        gui.KeyName.LEFT_PAREN: pyglet.window.key.PARENLEFT,
        gui.KeyName.RIGHT_PAREN: pyglet.window.key.PARENRIGHT,
        gui.KeyName.ASTERISK: pyglet.window.key.ASTERISK,
        gui.KeyName.PLUS: pyglet.window.key.PLUS,
        gui.KeyName.COMMA: pyglet.window.key.COMMA,
        gui.KeyName.MINUS: pyglet.window.key.MINUS,
        gui.KeyName.PERIOD: pyglet.window.key.PERIOD,
        gui.KeyName.SLASH: pyglet.window.key.SLASH,
        gui.KeyName.ZERO: pyglet.window.key._0,
        gui.KeyName.ONE: pyglet.window.key._1,
        gui.KeyName.TWO: pyglet.window.key._2,
        gui.KeyName.THREE: pyglet.window.key._3,
        gui.KeyName.FOUR: pyglet.window.key._4,
        gui.KeyName.FIVE: pyglet.window.key._5,
        gui.KeyName.SIX: pyglet.window.key._6,
        gui.KeyName.SEVEN: pyglet.window.key._7,
        gui.KeyName.EIGHT: pyglet.window.key._8,
        gui.KeyName.NINE: pyglet.window.key._9,
        gui.KeyName.COLON: pyglet.window.key.COLON,
        gui.KeyName.SEMICOLON: pyglet.window.key.SEMICOLON,
        gui.KeyName.LESS_THAN: pyglet.window.key.LESS,
        gui.KeyName.EQUALS: pyglet.window.key.EQUAL,
        gui.KeyName.GREATER_THAN: pyglet.window.key.GREATER,
        gui.KeyName.QUESTION_MARK: pyglet.window.key.QUESTION,
        gui.KeyName.AT: pyglet.window.key.AT,
        gui.KeyName.LEFT_BRACKET: pyglet.window.key.BRACKETLEFT,
        gui.KeyName.BACKSLASH: pyglet.window.key.BACKSLASH,
        gui.KeyName.RIGHT_BRACKET: pyglet.window.key.BRACKETRIGHT,
        gui.KeyName.CARET: pyglet.window.key.ASCIICIRCUM,
        gui.KeyName.UNDERSCORE: pyglet.window.key.UNDERSCORE,
        gui.KeyName.BACKTICK: pyglet.window.key.GRAVE,
        gui.KeyName.A: pyglet.window.key.A,
        gui.KeyName.B: pyglet.window.key.B,
        gui.KeyName.C: pyglet.window.key.C,
        gui.KeyName.D: pyglet.window.key.D,
        gui.KeyName.E: pyglet.window.key.E,
        gui.KeyName.F: pyglet.window.key.F,
        gui.KeyName.G: pyglet.window.key.G,
        gui.KeyName.H: pyglet.window.key.H,
        gui.KeyName.I: pyglet.window.key.I,
        gui.KeyName.J: pyglet.window.key.J,
        gui.KeyName.K: pyglet.window.key.K,
        gui.KeyName.L: pyglet.window.key.L,
        gui.KeyName.M: pyglet.window.key.M,
        gui.KeyName.N: pyglet.window.key.N,
        gui.KeyName.O: pyglet.window.key.O,
        gui.KeyName.P: pyglet.window.key.P,
        gui.KeyName.Q: pyglet.window.key.Q,
        gui.KeyName.R: pyglet.window.key.R,
        gui.KeyName.S: pyglet.window.key.S,
        gui.KeyName.T: pyglet.window.key.T,
        gui.KeyName.U: pyglet.window.key.U,
        gui.KeyName.V: pyglet.window.key.V,
        gui.KeyName.W: pyglet.window.key.W,
        gui.KeyName.X: pyglet.window.key.X,
        gui.KeyName.Y: pyglet.window.key.Y,
        gui.KeyName.Z: pyglet.window.key.Z,
        gui.KeyName.LEFT_BRACE: pyglet.window.key.BRACELEFT,
        gui.KeyName.PIPE: pyglet.window.key.BAR,
        gui.KeyName.RIGHT_BRACE: pyglet.window.key.BRACERIGHT,
        gui.KeyName.TILDE: pyglet.window.key.ASCIITILDE,
        gui.KeyName.DELETE: pyglet.window.key.DELETE,
        gui.KeyName.LEFT_SHIFT: pyglet.window.key.LSHIFT,
        gui.KeyName.RIGHT_SHIFT: pyglet.window.key.RSHIFT,
        gui.KeyName.LEFT_CONTROL: pyglet.window.key.LCTRL,
        gui.KeyName.RIGHT_CONTROL: pyglet.window.key.RCTRL,
        gui.KeyName.ALT: pyglet.window.key.MOD_ALT,
        gui.KeyName.META: pyglet.window.key.MOD_WINDOWS,
        gui.KeyName.CAPS_LOCK: pyglet.window.key.CAPSLOCK,
        gui.KeyName.LEFT: pyglet.window.key.LEFT,
        gui.KeyName.RIGHT: pyglet.window.key.RIGHT,
        gui.KeyName.UP: pyglet.window.key.UP,
        gui.KeyName.DOWN: pyglet.window.key.DOWN,
        gui.KeyName.INSERT: pyglet.window.key.INSERT,
        gui.KeyName.HOME: pyglet.window.key.HOME,
        gui.KeyName.END: pyglet.window.key.END,
        gui.KeyName.PAGE_UP: pyglet.window.key.PAGEUP,
        gui.KeyName.PAGE_DOWN: pyglet.window.key.PAGEDOWN,
        gui.KeyName.F1: pyglet.window.key.F1,
        gui.KeyName.F2: pyglet.window.key.F2,
        gui.KeyName.F3: pyglet.window.key.F3,
        gui.KeyName.F4: pyglet.window.key.F4,
        gui.KeyName.F5: pyglet.window.key.F5,
        gui.KeyName.F6: pyglet.window.key.F6,
        gui.KeyName.F7: pyglet.window.key.F7,
        gui.KeyName.F8: pyglet.window.key.F8,
        gui.KeyName.F9: pyglet.window.key.F9,
        gui.KeyName.F10: pyglet.window.key.F10,
        gui.KeyName.F11: pyglet.window.key.F11,
        gui.KeyName.F12: pyglet.window.key.F12
    }
    
    def keyEventToFunction(self, keyEvent):
        if keyEvent.type == keyEvent.DOWN:
            if keyEvent.key == gui.KeyName.LEFT_SHIFT or keyEvent.key == gui.KeyName.RIGHT_SHIFT:
                self.modifiers |= gui.KeyModifier.SHIFT.value
            if keyEvent.key == gui.KeyName.LEFT_CONTROL or keyEvent.key == gui.KeyName.RIGHT_CONTROL:
                self.modifiers |= gui.KeyModifier.CTRL.value
            if keyEvent.key == gui.KeyName.ALT:
                self.modifiers |= gui.KeyModifier.ALT.value
            if keyEvent.key == gui.KeyName.META:
                self.modifiers |= gui.KeyModifier.META.value
            if keyEvent.key in self._key_to_symbol:
                self.on_key_press(self._key_to_symbol[keyEvent.key], self.modifiers)
                return True
        elif keyEvent.type == keyEvent.UP:
            if keyEvent.key == gui.KeyName.LEFT_SHIFT or keyEvent.key == gui.KeyName.RIGHT_SHIFT:
                self.modifiers &= ~gui.KeyModifier.SHIFT.value
            if keyEvent.key == gui.KeyName.LEFT_CONTROL or keyEvent.key == gui.KeyName.RIGHT_CONTROL:
                self.modifiers &= ~gui.KeyModifier.CTRL.value
            if keyEvent.key == gui.KeyName.ALT:
                self.modifiers &= ~gui.KeyModifier.ALT.value
            if keyEvent.key == gui.KeyName.META:
                self.modifiers &= ~gui.KeyModifier.META.value
            if keyEvent.key in self._key_to_symbol:
                self.on_key_release(self._key_to_symbol[keyEvent.key], self.modifiers)
                return True
        else:
            raise NotImplementedError("KeyEvent is neither of type UP nor DOWN")
        
    def mouseEventToFunction(self, mouseEvent):
        if mouseEvent.type in (gui.MouseEvent.BUTTON_DOWN, gui.MouseEvent.DRAG, gui.MouseEvent.BUTTON_UP):
            screen_x = mouseEvent.x - self.scene_widget.frame.x
            screen_y = mouseEvent.y - self.scene_widget.frame.y

            if screen_x < 0:
                screen_x = 0
            elif screen_x > self.scene_widget.frame.width - 1:
                screen_x = self.scene_widget.frame.width - 1

            if screen_y < 0:
                screen_y = 0
            elif screen_y > self.scene_widget.frame.height - 1:
                screen_y = self.scene_widget.frame.height - 1

            button = 0
            if mouseEvent.buttons & gui.MouseButton.LEFT.value:
                button = pyglet.window.mouse.LEFT
            elif mouseEvent.buttons & gui.MouseButton.RIGHT.value:
                button = pyglet.window.mouse.RIGHT
            elif mouseEvent.buttons & gui.MouseButton.MIDDLE.value:
                button = pyglet.window.mouse.MIDDLE
            elif mouseEvent.buttons & gui.MouseButton.BUTTON4.value:
                button = pyglet.window.mouse.MOUSE4
            elif mouseEvent.buttons & gui.MouseButton.BUTTON5.value:
                button = pyglet.window.mouse.MOUSE5
            
            if mouseEvent.type == gui.MouseEvent.BUTTON_DOWN:
                self.on_mouse_press(screen_x, screen_y, -np.inf, button, self.modifiers)
            elif mouseEvent.type == gui.MouseEvent.DRAG:                    
                self.on_mouse_drag(screen_x, screen_y, -np.inf, screen_x - self.last_coords[0], screen_y - self.last_coords[1], -np.inf, mouseEvent.buttons, self.modifiers)
            elif mouseEvent.type == gui.MouseEvent.BUTTON_UP:
                self.on_mouse_release(screen_x, screen_y, -np.inf, button, self.modifiers)
            else:
                # Unsupported mouse type; do nothing
                pass

            self.last_coords = (screen_x, screen_y)

        else:
                # Unsupported mouse type; do nothing
                pass

        return gui.Widget.EventCallbackResult.HANDLED
    
    def on_mouse_press(self, x, y, z, button, modifiers):
        pass

    def on_mouse_drag(self, x, y, z, dx, dy, dz, buttons, modifiers):
        pass

    def on_mouse_release(self, x, y, z, button, modifiers):
        pass
        

    def addShape(self, shape, name=None, quick=False):
        '''
        If this method is called in rapid succession e.g., inside Scene3d.on_idle,
        set quick=True, which prevents some crashes.
        '''
        # check type of shape
        if quick:
            gui.Application.instance.post_to_main_thread(self.window, lambda: shape._addToScene(self, name))
        else:
            shape._addToScene(self, name)
        

    def updateShape(self, name, quick=False):
        '''
        If this method is called in rapid succession e.g., inside Scene3d.on_idle,
        set quick=True, which prevents some crashes.
        '''
        if self.scene_widget.scene.has_geometry(name):
            if quick:
                # The documentation recommends this instead, but it seems to only update after another action
                # has been taken (moving the mouse, pressing a key, etc.)
                gui.Application.instance.post_to_main_thread(self.window, lambda: self._shapeDict[name]._update(name, self))
            else:
                self._shapeDict[name]._update(name, self)
        elif name in self._shapeDict:
            shape = self._shapeDict[name]
            self.removeShape(name)
            self.addShape(shape, name, quick)


            
            

    def removeShape(self, name):
        if self.scene_widget.scene.has_geometry(name):
            self.scene_widget.scene.remove_geometry(name)
        if name in self._shapeDict:
            del self._shapeDict[name]

    def print(self, *args, **kwargs):
        print(*args, **kwargs, file=self.sio)
        self.text_output.text = self.sio.getvalue()[-3072:]

    def set_slider_value(self, slider_id, value, no_callback=False):
        '''
        Programmatically sets the value of slider indexed `slider_id` as `value`. If `no_callback=True` the respective
        callback function will not be triggered after the slider is set.
        '''
        if slider_id > len(self._sliders):
            raise IndexError("slider_id too large!")
        
        self._sliders[slider_id].double_value = value
        if not no_callback:
            self.on_slider_change(slider_id, value)

    # def show_axes(self, enable=True):
    #     '''
    #     It doesn't work!
    #     '''
    #     self.scene_widget.scene.show_axes(enable)
        
        
def get_rotation_matrix(angle, axis) -> np.ndarray:
    if isinstance(axis, (np.ndarray, list, tuple)):
        axis = np.array(axis)
        axis = axis / np.linalg.norm(axis)
        return o3d.geometry.get_rotation_matrix_from_axis_angle(angle * axis)
    else:
        raise TypeError("Incorrect type for axis")
        
def world_space(func):
    argspec = getfullargspec(func)
    def wrapper(*args, **kwargs):
        try:
            scene = args[argspec.args.index("self")]
        except IndexError:
            scene = kwargs["self"]

        if "x" in argspec.args and "y" in argspec.args and "z" in argspec.args:
            try:
                x = args[argspec.args.index("x")]
            except IndexError:
                x = kwargs["x"]
            try:
                y = args[argspec.args.index("y")]
            except IndexError:
                y = kwargs["y"]

            def screen_to_world(depth_image):
                depth = np.asarray(depth_image)[y, x]
                world = scene.scene_widget.scene.camera.unproject(x, y, depth, scene.scene_widget.frame.width, scene.scene_widget.frame.height)

                if "dx" in argspec.args and "dy" in argspec.args and "dz" in argspec.args:
                    try:
                        dx = args[argspec.args.index("dx")]
                    except IndexError:
                        dx = kwargs["dx"]
                    try:
                        dy = args[argspec.args.index("dy")]
                    except IndexError:
                        dy = kwargs["dy"]

                    ddepth = np.asarray(depth_image)[y - dy, x - dx]
                    dworld = scene.scene_widget.scene.camera.unproject(x - dx, y - dy, ddepth, scene.scene_widget.frame.width, scene.scene_widget.frame.height)

                new_kwargs = {}
                for arg in argspec.args:
                    if arg == "x":
                        new_kwargs["x"] = world[0]
                    elif arg == "y":
                        new_kwargs["y"] = world[1]
                    elif arg == "z":
                        new_kwargs["z"] = world[2]
                    elif arg == "dx":
                        new_kwargs["dx"] = world[0] - dworld[0] if not np.isinf(world[0]) and not np.isinf(dworld[0]) else 0
                    elif arg == "dy":
                        new_kwargs["dy"] = world[1] - dworld[1] if not np.isinf(world[1]) and not np.isinf(dworld[1]) else 0
                    elif arg == "dz":
                        new_kwargs["dz"] = world[2] - dworld[2] if not np.isinf(world[2]) and not np.isinf(dworld[2]) else 0
                    else:
                        try:
                            value = args[argspec.args.index(arg)]
                        except IndexError:
                            value = kwargs[arg]
                        finally:
                            new_kwargs[arg] = value

                func(**new_kwargs)

            scene.scene_widget.scene.scene.render_to_depth_image(screen_to_world)

    return wrapper