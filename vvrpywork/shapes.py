'''Implements objects to represent common 2D and 3D shapes.'''

from __future__ import annotations

from abc import ABC
from beartype.vale import Is
from math import sin, cos, acos
from numbers import Number
import numpy as np
import open3d as o3d
import open3d.visualization.rendering as rendering # type: ignore
from pyglet import shapes
import random
from shapely import MultiPoint, concave_hull
from typing import Annotated

from vvrpywork.scene import Scene2D, Scene3D

# Type Aliases
NDArray = np.ndarray
List = list
Tuple = tuple

NDArray2 = Annotated[np.ndarray, Is[lambda array: array.shape == (2,)]]
NDArray3 = Annotated[np.ndarray, Is[lambda array: array.shape == (3,)]]
NDArray4 = Annotated[np.ndarray, Is[lambda array: array.shape == (4,)]]
List2 = Annotated[list[Number], 2]
List3 = Annotated[list[Number], 3]
List4 = Annotated[list[Number], 4]
Tuple2 = tuple[Number, Number]
Tuple3 = tuple[Number, Number, Number]
Tuple4 = tuple[Number, Number, Number, Number]

ColorType = NDArray3|NDArray4|List3|List4|Tuple3|Tuple4

class Shape(ABC):
    '''Abstract Base Class to represent shapes.'''
    pass

class ShapeSet(ABC):
    '''Abstract Base Class to represent shape sets.'''
    pass

class Point2D(Shape):
    '''A class used to represent a point in 2D space.'''

    def __init__(self, p:Point2D|NDArray2|List2|Tuple2, size:Number=1.0, resolution:None|int=None, color:ColorType=(0, 0, 0)):
        '''Inits Point2D from (x,y) coordinates.
        
        Args:
            p: The coordinates of the point.
            size: The size of the displayed point.
            resolution: The resolution of the displayed point.
                If `None`, it will be calculated automatically.
            color: The color of the displayed point (RGB or RGBA).
        '''
        
        if isinstance(p, Point2D):
            self._x = p.x
            self._y = p.y
        elif isinstance(p, (np.ndarray, list, tuple)):
            self._x = p[0]
            self._y = p[1]
        else:
            raise TypeError("Incorrect type for p")

        self._size = size
        self._resolution = resolution

        self._color = [*color, 1] if len(color) == 3 else [*color]

    def _addToScene(self, scene:Scene2D, name:None|str):
        name = str(id(self)) if name is None else name
        shape = shapes.Circle(100 * self.x, 100 * self.y, self.size, self.resolution, tuple(int(255 * _ + 0.5) for _ in self.color), batch=scene._shapeBatch)
        self._resolution = shape._segments
        scene._shapeDict[name] = {"class": self, "shape": shape}

    def _update(self, shape:shapes.Circle, scene:Scene2D):
        shape.position = (100 * self.x, 100 * self.y)
        shape.radius = self.size
        shape.color = tuple(int(255 * _ + 0.5) for _ in self.color)

    def __eq__(self, other:Point2D|NDArray2|List2|Tuple2):
        if isinstance(other, Point2D):
            return self.x == other.x and self.y == other.y
        elif isinstance(other, np.ndarray, list, tuple):
            return self.x == other[0] and self.y == other[1]
        else:
            return False

    @property
    def x(self) -> Number:
        '''The point's position on the x-axis.'''
        return self._x
    
    @x.setter
    def x(self, x:Number):
        self._x = x

    @property
    def y(self) -> Number:
        '''The point's position on the y-axis.'''
        return self._y
    
    @y.setter
    def y(self, y:Number):
        self._y = y

    @property
    def size(self) -> Number:
        '''The point's size.'''
        return self._size
    
    @size.setter
    def size(self, size:Number):
        self._size = size
    
    @property
    def resolution(self) -> None|int:
        '''The point's resolution.
        
        The point is drawn using triangles. `resolution` represents
        the amount of triangles that will be used.
        '''
        return self._resolution

    @property
    def color(self) -> ColorType:
        '''The point's color in RGBA format.'''
        return self._color
    
    @color.setter
    def color(self, color:ColorType):
        self._color = [*color, 1] if len(color) == 3 else [*color]

    def distanceSq(self, p:Point2D) -> Number:
        '''Calculates the squared distance from a second point.
        
        Calculates the squared Euclidean distance between this and
        another point. It doesn't take the square root of the result
        and is, therefore, faster than calling `distance`.

        Args:
            p: The second point, the squared distance to which will
                be calculated.

        Returns:
            The squared distance between this point and `p`.
        '''
        return (self.x - p.x) ** 2 + (self.y - p.y) ** 2
    
    def distance(self, p:Point2D) -> float:
        '''Calculates the distance from a second point.
        
        Calculates the Euclidean distance between this and another
        point. If you do not need the exact distance, you may want
        to look into using `distanceSq` instead.

        Args:
            p: The second point, the distance to which will be
                calculated.

        Returns:
            The distance between this point and `p`.
        '''
        return self.distanceSq(p) ** 0.5

class Line2D(Shape):
    '''A class used to represent a line segment in 2D space.'''

    def __init__(self, p1:Point2D|NDArray2|List2|Tuple2, p2:Point2D|NDArray2|List2|Tuple2, width:Number=1, color:ColorType=(0, 0, 0)):
        '''Inits Line2D given the line segment's 2 endpoints.

        Args:
            p1: The coordinates of the first endpoint.
            p2: The coordinates of the second endpoint.
            width: The width of the displayed line segment.
            color: The color of the displayed line segment (RGB or
                RGBA).
        '''
        
        if isinstance(p1, Point2D):
            self._x1 = p1.x
            self._y1 = p1.y
        elif isinstance(p1, (np.ndarray, list, tuple)):
            self._x1 = p1[0]
            self._y1 = p1[1]
        else:
            raise TypeError("Incorrect type for p1")
        
        if isinstance(p2, Point2D):
            self._x2 = p2.x
            self._y2 = p2.y
        elif isinstance(p2, (np.ndarray, list, tuple)):
            self._x2 = p2[0]
            self._y2 = p2[1]
        else:
            raise TypeError("Incorrect type for p2")
        
        self._width = width
        self._color = [*color, 1] if len(color) == 3 else [*color]
    
    def _addToScene(self, scene:Scene2D, name:None|str):
        name = str(id(self)) if name is None else name
        shape = shapes.Line(100 * self.x1, 100 * self.y1, 100 * self.x2, 100 * self.y2, self.width, tuple(int(255 * _ + 0.5) for _ in self.color), batch=scene._shapeBatch)
        scene._shapeDict[name] = {"class": self, "shape": shape}

    def _update(self, shape:shapes.Line, scene:Scene2D):
        shape.position = (100 * self.x1, 100 * self.y1)
        shape.x2 = 100 * self.x2
        shape.y2 = 100 * self.y2
        shape.width = self.width
        shape.color = tuple(int(255 * _ + 0.5) for _ in self.color)

    @property
    def x1(self) -> Number:
        '''The x-coordinate of the first endpoint.'''
        return self._x1
    
    @x1.setter
    def x1(self, x1:Number):
        self._x1 = x1

    @property
    def y1(self) -> Number:
        '''The y-coordinate of the first endpoint.'''
        return self._y1
    
    @y1.setter
    def y1(self, y1:Number):
        self._y1 = y1

    @property
    def x2(self) -> Number:
        '''The x-coordinate of the second endpoint.'''
        return self._x2
    
    @x2.setter
    def x2(self, x2:Number):
        self._x2 = x2

    @property
    def y2(self) -> Number:
        '''The y-coordinate of the second endpoint.'''
        return self._y2
    
    @y2.setter
    def y2(self, y2:Number):
        self._y2 = y2

    @property
    def width(self) -> Number:
        '''The line segment's width.'''
        return self._width
    
    @width.setter
    def width(self, width:Number):
        self._width = width

    @property
    def color(self) -> ColorType:
        '''The line segment's color in RGBA format.'''
        return self._color
    
    @color.setter
    def color(self, color:ColorType):
        self._color = color

    def getPointFrom(self) -> Point2D:
        '''Returns the line segment's first endpoint.
        
        Returns:
            The line segment's first endpoint as a `Point2D` object.
        '''
        return Point2D((self.x1, self.y1))
    
    def getPointTo(self) -> Point2D:
        '''Returns the line segment's second endpoint.
        
        Returns:
            The line segment's second endpoint as a `Point2D` object.
        '''
        return Point2D((self.x2, self.y2))

    def length(self) -> float:
        '''Calculates the length of the line segment.
        
        Returns:
            The length of the line segment.
        '''
        return ((self.x2-self.x1)**2 + (self.y2-self.y1)**2)**.5
    
    def isOnRight(self, point:Point2D) -> bool:
        '''Determines whether a point is to the right of the line.
        
        Determines whether a point is to the right of the line defined
        by this line segment.

        Args:
            point: The point to check (if it's on the right).

        Returns:
            `True` if the point is on the right, `False` otherwise
                (incl. if it's on the line itself).
        '''
        return ((self.x2 - self.x1)*(point.y - self.y1) - (self.y2 - self.y1)*(point.x - self.x1)) < 0
    
class Triangle2D(Shape):
    '''A class used to represent a triangle in 2D space.'''

    def __init__(self, p1:Point2D|NDArray2|List2|Tuple2, p2:Point2D|NDArray2|List2|Tuple2, p3:Point2D|NDArray2|List2|Tuple2, width:Number=1, color:ColorType=(0, 0, 0), filled:bool=False):
        '''Inits Triangle2D given the triangle's 3 vertices.

        Args:
            p1: The coordinates of the first vertex.
            p2: The coordinates of the second vertex.
            p3: The coordinates of the third vertex.
            width: The width of the displayed triangle (if not filled).
            color: The color of the displayed triangle (RGB or RGBA).
            filled: Whether to fill in the triangle or draw only its
                outline.
        '''

        if isinstance(p1, Point2D):
            self._x1 = p1.x
            self._y1 = p1.y
        elif isinstance(p1, (np.ndarray, list, tuple)):
            self._x1 = p1[0]
            self._y1 = p1[1]
        else:
            raise TypeError("Incorrect type for p1")
        
        if isinstance(p2, Point2D):
            self._x2 = p2.x
            self._y2 = p2.y
        elif isinstance(p2, (np.ndarray, list, tuple)):
            self._x2 = p2[0]
            self._y2 = p2[1]
        else:
            raise TypeError("Incorrect type for p2")
        
        if isinstance(p3, Point2D):
            self._x3 = p3.x
            self._y3 = p3.y
        elif isinstance(p3, (np.ndarray, list, tuple)):
            self._x3 = p3[0]
            self._y3 = p3[1]
        else:
            raise TypeError("Incorrect type for p3")

        self._width = width
        self._color = [*color, 1] if len(color) == 3 else [*color]
        self._filled = filled

    def _addToScene(self, scene:Scene2D, name:None|str):
        if self.filled:
            name = str(id(self)) if name is None else name
            shape = shapes.Triangle(100 * self.x1, 100 * self.y1, 100 * self.x2, 100 * self.y2, 100 * self.x3, 100 * self.y3, tuple(int(255 * _ + 0.5) for _ in self.color), batch=scene._shapeBatch)
            scene._shapeDict[name] = {"class": self, "shape": shape}
        else:
            line1 = shapes.Line(100 * self.x1, 100 * self.y1, 100 * self.x2, 100 * self.y2, self.width, tuple(int(255 * _ + 0.5) for _ in self.color), batch=scene._shapeBatch)
            line2 = shapes.Line(100 * self.x2, 100 * self.y2, 100 * self.x3, 100 * self.y3, self.width, tuple(int(255 * _ + 0.5) for _ in self.color), batch=scene._shapeBatch)
            line3 = shapes.Line(100 * self.x3, 100 * self.y3, 100 * self.x1, 100 * self.y1, self.width, tuple(int(255 * _ + 0.5) for _ in self.color), batch=scene._shapeBatch)
            scene._shapeDict[name] = {"class": self, "shape": (line1, line2, line3)}

    def _update(self, shape:shapes.Triangle|tuple[shapes.Line, shapes.Line, shapes.Line], scene:Scene2D):
        if self.filled:
            shape.position = (100 * self.x1, 100 * self.y1)
            shape.x2 = 100 * self.x2
            shape.y2 = 100 * self.y2
            shape.x3 = 100 * self.x3
            shape.y3 = 100 * self.y3
            shape.color = tuple(int(255 * _ + 0.5) for _ in self.color)
        else:
            line1, line2, line3 = shape

            line1.position = (100 * self.x1, 100 * self.y1)
            line1.x2 = 100 * self.x2
            line1.y2 = 100 * self.y2
            line1.width = self.width
            line1.color = tuple(int(255 * _ + 0.5) for _ in self.color)

            line2.position = (100 * self.x2, 100 * self.y2)
            line2.x2 = 100 * self.x3
            line2.y2 = 100 * self.y3
            line2.width = self.width
            line2.color = tuple(int(255 * _ + 0.5) for _ in self.color)

            line3.position = (100 * self.x3, 100 * self.y3)
            line3.x2 = 100 * self.x1
            line3.y2 = 100 * self.y1
            line3.width = self.width
            line3.color = tuple(int(255 * _ + 0.5) for _ in self.color)

    @property
    def x1(self) -> Number:
        '''The x-coordinate of the first vertex.'''
        return self._x1
    
    @x1.setter
    def x1(self, x:Number):
        self._x1 = x

    @property
    def y1(self) -> Number:
        '''The y-coordinate of the first vertex.'''
        return self._y1
    
    @y1.setter
    def y1(self, y:Number):
        self._y1 = y
    
    @property
    def x2(self) -> Number:
        '''The x-coordinate of the second vertex.'''
        return self._x2
    
    @x2.setter
    def x2(self, x:Number):
        self._x2 = x

    @property
    def y2(self) -> Number:
        '''The y-coordinate of the second vertex.'''
        return self._y2
    
    @y2.setter
    def y2(self, y:Number):
        self._y2 = y
    
    @property
    def x3(self) -> Number:
        '''The x-coordinate of the third vertex.'''
        return self._x3
    
    @x3.setter
    def x3(self, x:Number):
        self._x3 = x

    @property
    def y3(self) -> Number:
        '''The y-coordinate of the third vertex.'''
        return self._y3
    
    @y3.setter
    def y3(self, y:Number):
        self._y3 = y
    
    @property
    def width(self) -> Number:
        '''The triangle's width (if not filled).'''
        return self._width

    @width.setter
    def width(self, width:Number):
        self._width = width

    @property
    def color(self) -> ColorType:
        '''The triangle's color in RGBA format.'''
        return self._color
    
    @color.setter
    def color(self, color:ColorType):
        self._color = color

    @property
    def filled(self) -> bool:
        '''Whether to fill in the triangle or draw only its outline.'''
        return self._filled

    def getPoint1(self) -> Point2D:
        '''Returns the triangle's first vertex.
        
        Returns:
            The triangle's first vertex as a `Point2D` object.
        '''
        return Point2D((self.x1, self.y1))
    
    def getPoint2(self) -> Point2D:
        '''Returns the triangle's second vertex.
        
        Returns:
            The triangle's second vertex as a `Point2D` object.
        '''
        return Point2D((self.x2, self.y2))
    
    def getPoint3(self) -> Point2D:
        '''Returns the triangle's third vertex.
        
        Returns:
            The triangle's third vertex as a `Point2D` object.
        '''
        return Point2D((self.x3, self.y3))
    
    def getCircumCircle(self) -> Circle2D:
        '''Returns the triangle's circumcircle.
        
        Returns:
            The triangle's circumcircle as a `Circle2D` object.
        '''
        # https://en.wikipedia.org/wiki/Circumcircle#Circumcenter_coordinates
        ax = self.x1
        ay = self.y1
        bx = self.x2
        by = self.y2
        cx = self.x3
        cy = self.y3

        D = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        ux = 1 / D * ((ax ** 2 + ay ** 2) * (by - cy) + (bx ** 2 + by **2) * (cy - ay) + (cx ** 2 + cy **2) * (ay - by))
        uy = 1 / D * ((ax ** 2 + ay ** 2) * (cx - bx) + (bx ** 2 + by **2) * (ax - cx) + (cx ** 2 + cy **2) * (bx - ax))
        r = ((ax - ux) ** 2 + (ay - uy) ** 2) ** .5

        return Circle2D((ux, uy), r)
    
    def contains(self, point:Point2D) -> bool:
        '''Determines whether a point is inside the triangle.

        Args:
            point: The point to check (if it's inside the triangle).

        Returns:
            `True` if the point is inside the triangle (incl. the
                edges), `False` otherwise.
        '''
        # https://totologic.blogspot.com/2014/01/accurate-point-in-triangle-test.html
        x = point.x
        y = point.y
        x1 = self.x1
        y1 = self.y1
        x2 = self.x2
        y2 = self.y2
        x3 = self.x3
        y3 = self.y3

        a = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / ((y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3))
        b = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / ((y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3))
        c = 1 - a - b

        return (a >= 0 and b >= 0 and c >= 0 and a <= 1 and b <= 1 and c <= 1)
    
class Circle2D(Shape):
    '''A class used to represent a circle in 2D space.'''

    def __init__(self, p:Point2D|NDArray2|List2|Tuple2, radius:Number, resolution:None|int=None, width:Number=1, color:ColorType=(0, 0, 0), filled:bool=False):
        '''Inits Circle2D given the circle's center and radius.

        Args:
            p: The coordinates of the center.
            radius: The circle's radius.
            resolution: The resolution of the displayed circle.
                If `None`, it will be calculated automatically.
            width: The width of the displayed circle (if not filled).
            color: The color of the displayed circle (RGB or RGBA).
            filled: Whether to fill in the circle or draw only its
                outline.
        '''

        if isinstance(p, Point2D):
            self._x = p.x
            self._y = p.y
        elif isinstance(p, (np.ndarray, list, tuple)):
            self._x = p[0]
            self._y = p[1]
        else:
            raise TypeError("Incorrect type for p")
        
        self._radius = radius
        self._resolution = resolution
        self._width = width
        self._color = [*color, 1] if len(color) == 3 else [*color]
        self._filled = filled

    def _addToScene(self, scene:Scene2D, name:None|str):
        if self.filled:
            name = str(id(self)) if name is None else name
            shape = shapes.Circle(100 * self.x, 100 * self.y, 100 * self.radius, self.resolution, tuple(int(255 * _ + 0.5) for _ in self.color), batch=scene._shapeBatch)
            scene._shapeDict[name] = {"class": self, "shape": shape}
        else:
            name = str(id(self)) if name is None else name
            shape = shapes.Arc(100 * self.x, 100 * self.y, 100 * self.radius, self.resolution, thickness=self.width, color=tuple(int(255 * _ + 0.5) for _ in self.color), batch=scene._shapeBatch)
            scene._shapeDict[name] = {"class": self, "shape": shape}

    def _update(self, shape:shapes.Circle|shapes.Arc, scene:Scene3D):
        if self.filled:
            shape.position = (100 * self.x, 100 * self.y)
            shape.radius = 100 * self.radius
            shape.color = tuple(int(255 * _ + 0.5) for _ in self.color)
        else:
            shape.position = (100 * self.x, 100 * self.y)
            shape.radius = 100 * self.radius
            shape.thickness = self.width
            shape.color = tuple(int(255 * _ + 0.5) for _ in self.color)

    @property
    def x(self) -> Number:
        '''The x-coordinate of the circle's center point.'''
        return self._x
    
    @x.setter
    def x(self, x:Number):
        self._x = x

    @property
    def y(self) -> Number:
        '''The y-coordinate of the circle's center point.'''
        return self._y
    
    @y.setter
    def y(self, y:Number):
        self._y = y

    @property
    def radius(self) -> Number:
        '''The circle's radius.'''
        return self._radius
    
    @radius.setter
    def radius(self, r:Number):
        self._radius = r

    @property
    def resolution(self) -> None|int:
        '''The circle's resolution.
        
        The circle is drawn using triangles. `resolution` represents
        the amount of triangles that will be used.
        '''
        return self._resolution
    
    @property
    def width(self) -> Number:
        '''The circle's width (if not filled).'''
        return self._width
    
    @width.setter
    def width(self, width:Number):
        self._width = width

    @property
    def color(self) -> ColorType:
        '''The circle's color in RGBA format.'''
        return self._color
    
    @color.setter
    def color(self, color:ColorType):
        self._color = color

    @property
    def filled(self) -> bool:
        '''Whether to fill in the circle or draw only its outline.'''
        return self._filled
    
    def getPointCenter(self) -> Point2D:
        '''Returns the circle's center.
        
        Returns:
            The circle's center point as a `Point2D` object.
        '''
        return Point2D((self.x, self.y))
    
    def contains(self, point:Point2D) -> bool:
        '''Determines whether a point is inside the circle.

        Args:
            point: The point to check (if it's inside the circle).

        Returns:
            `True` if the point is inside the circle (incl. the
                outline), `False` otherwise.
        '''
        return (self.x - point.x) ** 2 + (self.y - point.y) ** 2 <= self.radius ** 2

class Rectangle2D(Shape):
    '''A class used to represent a rectangle in 2D space.'''

    def __init__(self, p1:Point2D|NDArray2|List2|Tuple2, p2:Point2D|NDArray2|List2|Tuple2, width:Number=1, color:ColorType=(0, 0, 0), filled:bool=False):
        '''Inits Rectangle2D given 2 vertices of the rectangle.

        Args:
            p1: The coordinates of one vertex.
            p2: The coordinates of another vertex.
            width: The width of the displayed rectangle (if not
                filled).
            color: The color of the displayed rectangle (RGB or RGBA).
            filled: Whether to fill in the rectangle or draw only its
                outline.
        '''

        if isinstance(p1, Point2D):
            x1 = p1.x
            y1 = p1.y
        elif isinstance(p1, (np.ndarray, list, tuple)):
            x1 = p1[0]
            y1 = p1[1]
        else:
            raise TypeError("Incorrect type for p1")
        
        if isinstance(p2, Point2D):
            x2 = p2.x
            y2 = p2.y
        elif isinstance(p2, (np.ndarray, list, tuple)):
            x2 = p2[0]
            y2 = p2[1]
        else:
            raise TypeError("Incorrect type for p2")

        self._x_min = min(x1, x2)
        self._y_min = min(y1, y2)
        self._x_max = max(x1, x2)
        self._y_max = max(y1, y2)

        self._width = width
        self._color = [*color, 1] if len(color) == 3 else [*color]
        self._filled = filled

    def _addToScene(self, scene:Scene2D, name:None|str):
        if self.filled:
            name = str(id(self)) if name is None else name
            shape = shapes.Rectangle(100 * self.x_min, 100 * self.y_min, 100 * (self.x_max-self.x_min), 100 * (self.y_max-self.y_min), tuple(int(255 * _ + 0.5) for _ in self.color), batch=scene._shapeBatch)
            scene._shapeDict[name] = {"class": self, "shape": shape}
        else:
            name = str(id(self)) if name is None else name
            shape = shapes.Box(100 * self.x_min, 100 * self.y_min, 100 * (self.x_max-self.x_min), 100 * (self.y_max-self.y_min), self.width, tuple(int(255 * _ + 0.5) for _ in self.color), batch=scene._shapeBatch)
            scene._shapeDict[name] = {"class": self, "shape": shape}

    def _update(self, shape:shapes.Rectangle|shapes.Box, scene:Scene2D):
        if self.filled:
            shape.position = (100 * self.x_min, 100 * self.y_min)
            shape.width = 100 * (self.x_max-self.x_min)
            shape.height = 100 * (self.y_max-self.y_min)
            shape.color = tuple(int(255 * _ + 0.5) for _ in self.color)
        else:
            shape.position = (100 * self.x_min, 100 * self.y_min)
            shape.width = 100 * (self.x_max-self.x_min)
            shape.height = 100 * (self.y_max-self.y_min)
            shape._thickness = self.width
            shape.color = tuple(int(255 * _ + 0.5) for _ in self.color)

    @property
    def x_min(self) -> Number:
        '''The x-coordinate of the bottom-left vertex.'''
        return self._x_min
    
    @x_min.setter
    def x_min(self, x:Number):
        self._x_min = x
        if self._x_min > self._x_max:
            self._x_min, self._x_max = self._x_max, self._x_min

    @property
    def y_min(self) -> Number:
        '''The y-coordinate of the bottom-left vertex.'''
        return self._y_min
    
    @y_min.setter
    def y_min(self, y:Number):
        self._y_min = y
        if self._y_min > self._y_max:
            self._y_min, self._y_max = self._y_max, self._y_min

    @property
    def x_max(self) -> Number:
        '''The x-coordinate of the top-right vertex.'''
        return self._x_max
    
    @x_max.setter
    def x_max(self, x:Number):
        self._x_max = x
        if self._x_max < self._x_min:
            self._x_max, self._x_min = self._x_min, self._x_max

    @property
    def y_max(self) -> Number:
        '''The y-coordinate of the top-right vertex.'''
        return self._y_max
    
    @y_max.setter
    def y_max(self, y:Number):
        self._y_max = y
        self._y_max = y
        if self._y_max < self._y_min:
            self._y_max, self._y_min = self._y_min, self._y_max
    
    @property
    def width(self) -> Number:
        '''The rectangle's width (if not filled).'''
        return self._width

    @width.setter
    def width(self, width:Number):
        self._width = width

    @property
    def color(self) -> ColorType:
        '''The rectangle's color in RGBA format.'''
        return self._color
    
    @color.setter
    def color(self, color:ColorType):
        self._color = color

    @property
    def filled(self) -> bool:
        '''Whether to fill in the rectangle or draw only its outline.'''
        return self._filled
    
    def translate(self, translation:Point2D|NDArray2|List2|Tuple2):
        '''Translates the rectangle by a vector.

        Translates the rectangle by `translation`. This is mostly
        useful when the min/max coordinate values would switch if you
        applied the translation individually to each vertex.

        Args:
            translation: The translation vector. Its coordinates will
                be added to the rectangle's coordinates.
        '''
        if isinstance(translation, Point2D):
            x = translation.x
            y = translation.y
        elif isinstance(translation, (np.ndarray, list, tuple)):
            x = translation[0]
            y = translation[1]
        else:
            raise TypeError("Incorrect type for translation")
        
        self._x_min += x
        self._y_min += y
        self._x_max += x
        self._y_max += y

class PointSet2D(ShapeSet):
    '''A class used to represent a set of points in 2D space.'''

    def __init__(self, points:None|NDArray|List|Tuple=None, size:Number=1, color:ColorType=(0, 0, 0)):
        '''Inits PointSet2D.

        Inits a PointSet2D containing `points`. If `points` is `None`,
        the pointset will be initialized empty.

        Args:
            points: The points of the pointset.
            size: The size of the displayed points.
            color: The color of the displayed points (RGB or RGBA).
        '''
        self._points:list[List2] = []
        self._size = size
        self._colors:list[ColorType] = []

        if points is not None:
            if isinstance(points, (np.ndarray, list, tuple)):
                self._points = [list(_) for _ in points]
                self._colors = [[*color, 1] if len(color) == 3 else [*color] for _ in points]
            else:
                raise TypeError(f"Unsupported type for points: {type(points)}")

    def __len__(self):
        return len(self._points)
    
    def __getitem__(self, idx):
        return self.getPointAt(idx)

    def _addToScene(self, scene:Scene2D, name:None|str):
        name = str(id(self)) if name is None else name
        points = []
        for p, c in zip(self._points, self._colors):
            points.append(shapes.Circle(100 * p[0], 100 * p[1], self.size, color=tuple(int(255 * _ + 0.5) for _ in c), batch=scene._shapeBatch))
        scene._shapeDict[name] = {"class": self, "shape": points}

    def _update(self, shape:list[shapes.Circle], scene:Scene2D):
            if len(shape) == len(self._points):
                for i, p in enumerate(shape):
                    p.position = (100 * self._points[i][0], 100 * self._points[i][1])
                    p.radius = self.size
                    p.color = tuple(int(255 * _ + 0.5) for _ in self._colors[i])

            elif len(shape) < len(self._points):
                for i, p in enumerate(shape):
                    p.position = (100 * self._points[i][0], 100 * self._points[i][1])
                    p.radius = self.size
                    p.color = tuple(int(255 * _ + 0.5) for _ in self._colors[i])
                for p, c in zip(self._points[len(shape):], self._colors[len(shape):]):
                    shape.append(shapes.Circle(100 * p[0], 100 * p[1], self.size, color=tuple(int(255 * _ + 0.5) for _ in c), batch=scene._shapeBatch))
            
            else:  # len(shape) > len(self._points)
                for i, p in enumerate(self._points):
                    shape[i].position = (100 * p[0], 100 * p[1])
                    shape[i].radius = self.size
                    shape[i].color = tuple(int(255 * _ + 0.5) for _ in self._colors[i])
                del shape[len(self._points):]

    @property
    def points(self) -> NDArray:
        '''The points of the pointset.'''
        return np.array(self._points)
    
    @points.setter
    def points(self, points:NDArray|List|Tuple):
        if isinstance(points, (np.ndarray, list, tuple)):
            self._points = [list(_) for _ in points]
    
    @property
    def size(self) -> Number:
        '''The size of the displayed points.'''
        return self._size
    
    @size.setter
    def size(self, size:Number):
        self._size = size

    @property
    def colors(self) -> NDArray:
        '''The points' colors in RGBA format.'''
        return np.array(self._colors)
    
    @colors.setter
    def colors(self, colors:NDArray|List|Tuple):
        if isinstance(colors, (np.ndarray, list, tuple)):
            self._colors = [list(_) for _ in colors]

    def getPointAt(self, index:int) -> Point2D:
        '''Returns the point at the specified index.

        Args:
            index: The index at which the desired point is placed
                inside the pointset.
        
        Returns:
            The point at the specified index as a `Point2D` object. It
                retains its size and color.
        '''
        return Point2D(self._points[index], self.size, color=self._colors[index])

    def add(self, point:Point2D):
        '''Appends a point to the pointset.
        
        Args:
            point: The `Point2D` object to append.
        '''
        self._points.append([point.x, point.y])
        self._colors.append([*point.color])

    def createRandom(self, bound:Rectangle2D|Circle2D, num_points:int, seed:None|int|str=None, color:ColorType=(0, 0, 0)):
        '''Appends random points to the pointset.

        Uniformly generates random points inside a region and appends
        them to the pointset.
        
        Args:
            bound: The area inside of which the random points will be
                generated.
            num_points: How many points to generate.
            seed: An optional seed for the RNG.
            color: The color of the generated points.
        '''
        if len(self._points) > 0:
            print('Point Set is not empty; random points will be appended to the existing ones.')

        if seed:
            random.seed(seed)

        if isinstance(bound, Rectangle2D):
            x1 = bound.x_min
            y1 = bound.y_min
            x2 = bound.x_max
            y2 = bound.y_max
            
            for _ in range(num_points):
                x = random.uniform(x1, x2)
                y = random.uniform(y1, y2)
                self._points.append([x, y])
                self._colors.append([*color, 1] if len(color) == 3 else [*color])

        elif isinstance(bound, Circle2D):
            # https://stackoverflow.com/a/50746409
            centerX = bound.x
            centerY = bound.y
            R = bound.radius

            for _ in range(num_points):
                r = R * random.uniform(0, 1) ** .5
                theta = random.uniform(0, 1) * 2 * np.pi

                x = centerX + r * cos(theta)
                y = centerY + r * sin(theta)
                self._points.append([x, y])
                self._colors.append([*color, 1] if len(color) == 3 else [*color])

        else:
            raise TypeError("Unsupported bound type")
        
    def remove(self, index:int):
        '''Removes a point from the pointset.

        Removes a point from the pointset's specified index.
        
        Args:
            index: The index at which the to-be-removed point is placed
                inside the pointset.
        '''
        self._points.pop(index)
        self._colors.pop(index)

    def clear(self):
        '''Clears the pointset.

        Clears the pointset, completely removing all points and
        information about them (e.g., color).
        '''
        self._points.clear()
        self._colors.clear()

class LineSet2D(ShapeSet):
    '''A class used to represent a set of lines in 2D space.'''

    def __init__(self, points:None|NDArray|List|Tuple=None, lines:None|NDArray|List|Tuple=None, width:Number=1, color:ColorType=(0, 0, 0)):
        '''Inits LineSet2D.

        Inits a LineSet2D containing `points` connected according to
        `lines`.
        
        If `lines` is `None`, the `points` will be connected in pairs
        i.e., (0, 1), (2, 3), etc.
        
        If `points` is `None`, the lineset will be initialized empty.

        Args:
            points: The points of the lineset.
            lines: The indices in `points` that are connected by a
                line.
            width: The width of the displayed lines.
            color: The color of the displayed lines (RGB or RGBA).
        '''
        self._points:list[List2] = []
        self._lines:list[List2] = []

        self._width = width
        self._colors:list[ColorType] = []

        if isinstance(points, PointSet2D):
            points = points.points

        if points is not None and lines is None:
            if isinstance(points, (np.ndarray, list, tuple)) and len(points) % 2 == 0:
                lines = [[i,i+1] for i in range(0, len(points), 2)]
            else:
                raise RuntimeError("Attempted connecting point pairs, but points is not divisible by 2.")

        if points is not None and lines is not None:
            if isinstance(points, (np.ndarray, list, tuple)):
                self._points = [list(_) for _ in points]
            if isinstance(lines, (np.ndarray, list, tuple)):
                self._lines = [list(_) for _ in lines]

            self._colors = [[*color, 1] if len(color) == 3 else [*color] for _ in lines]

    def __len__(self):
        return len(self._lines)
    
    def __getitem__(self, idx):
        return self.getLineAt(idx)

    def _addToScene(self, scene:Scene2D, name:None|str):
        name = str(id(self)) if name is None else name
        lines = []
        for i, l in enumerate(self._lines):
            lines.append(shapes.Line(100 * self._points[l[0]][0], 100 * self._points[l[0]][1], 100 * self._points[l[1]][0], 100 * self._points[l[1]][1], self.width, color=tuple(int(255 * _ + 0.5) for _ in self.colors[i]), batch=scene._shapeBatch))
        scene._shapeDict[name] = {"class": self, "shape": lines}

    def _update(self, shape:list[shapes.Line], scene:Scene2D):
            if len(shape) == len(self._lines):
                for i, l in enumerate(shape):
                    l.position = (100 * self._points[self._lines[i][0]][0], 100 * self._points[self._lines[i][0]][1])
                    l.x2 = 100 * self._points[self._lines[i][1]][0]
                    l.y2 = 100 * self._points[self._lines[i][1]][1]
                    l.width = self.width
                    l.color = tuple(int(255 * _ + 0.5) for _ in self.colors[i])

            elif len(shape) < len(self._lines):
                for i, l in enumerate(shape):
                    l.position = (100 * self._points[self._lines[i][0]][0], 100 * self._points[self._lines[i][0]][1])
                    l.x2 = 100 * self._points[self._lines[i][1]][0]
                    l.y2 = 100 * self._points[self._lines[i][1]][1]
                    l.width = self.width
                    l.color = tuple(int(255 * _ + 0.5) for _ in self.colors[i])
                for i, l in enumerate(self._lines[len(shape):]):
                    
                    shape.append(shapes.Line(100 * self._points[l[0]][0], 100 * self._points[l[0]][1], 100 * self._points[l[1]][0], 100 * self._points[l[1]][1], self.width, color=tuple(int(255 * _ + 0.5) for _ in self.colors[len(shape) + i]), batch=scene._shapeBatch))
            
            else:  # len(shape) > len(self._lines)
                for i, l in enumerate(self._lines):
                    shape[i].position = (100 * self._points[l[0]][0], 100 * self._points[l[0]][1])
                    shape[i].x2 = 100 * self._points[l[1]][0]
                    shape[i].y2 = 100 * self._points[l[1]][1]
                    shape[i].width = self.width
                    shape[i].color = tuple(int(255 * _ + 0.5) for _ in self.colors[i])
                del shape[len(self._lines):]

    @property
    def points(self) -> NDArray:
        '''The points of the lineset.'''
        return np.array(self._points)
    
    @points.setter
    def points(self, pts:NDArray|List|Tuple):
        if isinstance(pts, (np.ndarray, list, tuple)):
            self._points = [list(_) for _ in pts]
    
    @property
    def lines(self) -> NDArray:
        '''The point indices indicating lines of the lineset.'''
        return np.array(self._lines)
    
    @lines.setter
    def lines(self, lns:NDArray|List|Tuple):
        if isinstance(lns, (np.ndarray, list, tuple)):
            self._lines = [list(_) for _ in lns]

    @property
    def width(self) -> Number:
        '''The width of the displayed lines.'''
        return self._width
    
    @width.setter
    def width(self, width:Number):
        self._width = width
    
    @property
    def colors(self) -> NDArray:
        '''The lines' colors in RGBA format.'''
        return np.array(self._colors)
    
    @colors.setter
    def colors(self, colors:NDArray|List|Tuple):
        if isinstance(colors, (np.ndarray, list, tuple)):
            self._colors = [list(_) for _ in colors]
        
    def getLineAt(self, index:int) -> Line2D:
        '''Returns the line at the specified index.

        Args:
            index: The index at which the desired line is placed
                inside the lineset.
        
        Returns:
            The line at the specified index as a `Line2D` object. It
                retains its width and color.
        '''
        return Line2D(self._points[self._lines[index][0]], self._points[self._lines[index][1]], self.width, self.colors[index])
        
    def add(self, line:Line2D):
        '''Appends a line to the lineset.
        
        Args:
            line: The `Line2D` object to append.
        '''
        idx1 = idx2 = None
        if len(self._points) == 0:
            self._points.append([line.x1, line.y1])

        for i, p in enumerate(self._points):
            if p[0] == line.x1 and p[1] == line.y1:
                idx1 = i
            if p[0] == line.x2 and p[1] == line.y2:
                idx2 = i

        if idx1 == None:
            self._points.append([line.x1, line.y1])
            idx1 = len(self._points) - 1
        if idx2 == None:
            self._points.append([line.x2, line.y2])
            idx2 = len(self._points) - 1

        self._lines.append([idx1, idx2])
        self._colors.append([*line.color])

    def remove(self, index:int):
        '''Removes a line from the lineset.

        Removes a line from the lineset's specified index (does not
        affect LineSet2D.points in any way).
        
        Args:
            index: The index at which the to-be-removed line is placed
                inside the lineset.
        '''
        self._lines.pop(index)
        self._colors.pop(index)

    def clear(self):
        '''Clears the lineset.

        Clears the lineset, completely removing all points, lines and
        information about them (e.g., color).
        '''
        self._points.clear()
        self._lines.clear()
        self._colors.clear()

class Polygon2D(LineSet2D):
    '''A class used to represent an arbitrary polygon in 2D space.'''
    
    def __init__(self, points:None|PointSet2D|NDArray|List|Tuple=None, lines:None|PointSet2D|NDArray|List|Tuple=None, width:Number=1, color:ColorType=(0, 0, 0), reorderIfNecessary:bool=False):
        '''Inits Polygon2D.

        Inits a Polygon2D containing `points` connected according to
        `lines`.
        
        If `lines` is `None`, the `points` will be connected in
        sequence i.e., (0, 1), (1, 2), ..., (n-1, n), (n, 0).
        
        If `points` is `None`, the polygon will be initialized empty.

        Args:
            points: The points of the polygon.
            lines: The indices in `points` that are connected by a
                line.
            width: The width of the displayed lines.
            color: The color of the displayed lines (RGB or RGBA).
            reorderIfNecessary: If `True` (and `lines` is `None`), the
                points will be reordered to attempt to make a
                non - self-intersecting polygon.
        '''

        if isinstance(points, PointSet2D):
            points = points.points

        if points is not None and lines is None:
            if isinstance(points, (np.ndarray, list, tuple)):
                if reorderIfNecessary:
                    mp = MultiPoint(tuple(map(tuple, points)))
                    ch = concave_hull(mp, ratio=0)
                    points = tuple(ch.exterior.coords)[:-1]
                line_amount = len(points)
                lines = [[i, i + 1] for i in range(len(points) - 1)]
                lines.append([len(points) - 1, 0])
        
        super().__init__(points, lines, width, color)

    @staticmethod
    def create_from_lineset(lineset:LineSet2D, width:Number=1, color:ColorType=(0, 0, 0)) -> Polygon2D:
        '''Creates a Polygon2D object from a Lineset2D object.

        Args:
            lineset: The lineset to be turned into a polygon.
            width: The width of the displayed lines.
            color: The color of the displayed lines (RGB or RGBA).

        Returns:
            The Polygon2D object created from the lineset.
        '''
        return Polygon2D(lineset._points, lineset._lines, width, color)


class Point3D(Shape):
    '''A class used to represent a point in 3D space.'''

    def __init__(self, p:Point3D|NDArray3|List3|Tuple3, size:Number=1, resolution:int=20, color:ColorType=(0, 0, 0)):
        '''Inits Point3D from (x,y,z) coordinates.
        
        Args:
            p: The coordinates of the point.
            size: The size of the displayed point.
            resolution: The resolution of the displayed point.
            color: The color of the displayed point (RGB or RGBA).
        '''
        if isinstance(p, Point3D):
            self._x = p.x
            self._y = p.y
            self._z = p.z
        elif isinstance(p, (np.ndarray, list, tuple)):
            self._x = p[0]
            self._y = p[1]
            self._z = p[2]
        else:
            raise TypeError("Incorrect type for p")
        
        self._size = size
        self._resolution = resolution

        self._color = [*color, 1] if len(color) == 3 else [*color]

    def _addToScene(self, scene:Scene3D, name:None|str):
        name = str(id(self)) if name is None else name
        scene._shapeDict[name] = self
        shape = o3d.geometry.TriangleMesh.create_sphere(0.02, self.resolution)
        shape.compute_vertex_normals()
        material = rendering.MaterialRecord()
        material.shader = "defaultLitTransparency"
        color = self.color
        color = tuple((*color, 1)) if len(color) == 3 else color
        material.base_color = color
        scene.scene_widget.scene.add_geometry(name, shape, material)
        scene.scene_widget.scene.set_geometry_transform(name, ((self.size, 0, 0, self.x), (0, self.size, 0, self.y), (0, 0, self.size, self.z), (0, 0, 0, 1)))


    def _update(self, name:str, scene:Scene3D):
        translation = np.array(((self.size, 0, 0, self._x), (0, self.size, 0, self._y), (0, 0, self.size, self._z), (0, 0, 0, 1)))
        scene.scene_widget.scene.set_geometry_transform(name, translation)
        color = self.color
        color = tuple((*color, 1)) if len(color) == 3 else color
        material = rendering.MaterialRecord()
        material.shader = "defaultLitTransparency"
        material.base_color = color
        scene.scene_widget.scene.modify_geometry_material(name, material)

    def __eq__(self, other:Point3D|NDArray3|List3|Tuple3):
        if isinstance(other, Point2D):
            return self.x == other.x and self.y == other.y and self.z == other.z
        elif isinstance(other, np.ndarray, list, tuple):
            return self.x == other[0] and self.y == other[1] and self.x == other[2]
        else:
            return False

    def __str__(self):
        return f"Point3D({self.x}, {self.y}, {self.z})"

    @property
    def x(self):
        '''The point's position on the x-axis.'''
        return self._x
    
    @x.setter
    def x(self, x:Number):
        self._x = x

    @property
    def y(self):
        '''The point's position on the y-axis.'''
        return self._y
    
    @y.setter
    def y(self, y:Number):
        self._y = y

    @property
    def z(self):
        '''The point's position on the z-axis.'''
        return self._z
    
    @z.setter
    def z(self, z:Number):
        self._z = z

    @property
    def size(self):
        '''The point's size.'''
        return self._size
    
    @size.setter
    def size(self, size:Number):
        self._size = size
    
    @property
    def resolution(self):
        '''The point's resolution.
        
        The point is drawn using triangles. `resolution` represents
        the amount of triangles that will be used.
        '''
        return self._resolution

    @property
    def color(self):
        '''The point's color in RGBA format.'''
        return self._color
    
    @color.setter
    def color(self, color:ColorType):
        self._color = [*color, 1] if len(color) == 3 else [*color]

    def distanceSq(self, p:Point3D) -> Number:
        '''Calculates the squared distance from a second point.
        
        Calculates the squared Euclidean distance between this and
        another point. It doesn't take the square root of the result
        and is, therefore, faster than calling `distance`.

        Args:
            p: The second point, the squared distance to which will
                be calculated.

        Returns:
            The squared distance between this point and `p`.
        '''
        return (self.x - p.x) ** 2 + (self.y - p.y) ** 2 + (self.z - p.z) ** 2
    
    def distance(self, p:Point3D) -> float:
        '''Calculates the distance from a second point.
        
        Calculates the Euclidean distance between this and another
        point. If you do not need the exact distance, you may want
        to look into using `distanceSq` instead.

        Args:
            p: The second point, the distance to which will be
                calculated.

        Returns:
            The distance between this point and `p`.
        '''
        return self.distanceSq(p) ** 0.5

class Line3D(Shape):
    '''A class used to represent a line segment in 3D space.'''

    def __init__(self, p1:Point3D|NDArray3|List3|Tuple3, p2:Point3D|NDArray3|List3|Tuple3, width:Number=1, resolution:int=20, color:ColorType=(0, 0, 0)):
        '''Inits Line3D given the line segment's 2 endpoints.

        Args:
            p1: The coordinates of the first endpoint.
            p2: The coordinates of the second endpoint.
            width: The width of the displayed line segment.
            resolution: The resolution of the displayed line.
            color: The color of the displayed line segment (RGB or
                RGBA).
        '''
        
        if isinstance(p1, Point3D):
            self._x1 = p1.x
            self._y1 = p1.y
            self._z1 = p1.z
        elif isinstance(p1, (np.ndarray, list, tuple)):
            self._x1 = p1[0]
            self._y1 = p1[1]
            self._z1 = p1[2]
        else:
            raise TypeError("Incorrect type for p1")
        
        if isinstance(p2, Point3D):
            self._x2 = p2.x
            self._y2 = p2.y
            self._z2 = p2.z
        elif isinstance(p2, (np.ndarray, list, tuple)):
            self._x2 = p2[0]
            self._y2 = p2[1]
            self._z2 = p2[2]
        else:
            raise TypeError("Incorrect type for p2")

        self._width = width
        self._resolution = resolution

        self._color = [*color, 1] if len(color) == 3 else [*color]

        self._length = self.length()
        self._rot = None
    
    def _addToScene(self, scene:Scene3D, name:None|str):
        name = str(id(self)) if name is None else name
        scene._shapeDict[name] = self
        shape = o3d.geometry.TriangleMesh.create_cylinder(0.005 * self.width, self.length(), self.resolution)
        shape.compute_vertex_normals()

        v = np.array(((self.x2-self.x1)/self.length(), (self.y2-self.y1)/self.length(), (self.z2-self.z1)/self.length()))
        z = np.array((0., 0., 1.))
        a = np.cross(z, v)
        div = (a**2).sum()**0.5
        
        if div != 0:
            a /= div
            aa = a * acos(np.dot(z, v))

            self._rot = o3d.geometry.get_rotation_matrix_from_axis_angle(aa)
            rotation = np.zeros((4, 4))
            rotation[:3, :3] = self._rot
            rotation[3, 3] = 1
        else:
            self._rot = None
            rotation = np.array(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))

        translation = np.array(((1, 0, 0, (self.x1+self.x2)/2), (0, 1, 0, (self.y1+self.y2)/2), (0, 0, 1, (self.z1+self.z2)/2), (0, 0, 0, 1)))

        material = rendering.MaterialRecord()
        material.shader = "defaultLitTransparency"
        color = self.color
        color = tuple((*color, 1)) if len(color) == 3 else color
        material.base_color = color
        scene.scene_widget.scene.add_geometry(name, shape, material)
        scene.scene_widget.scene.set_geometry_transform(name, translation @ rotation)


    def _update(self, name:str, scene:Scene3D):
        old_length = self._length
        new_length = ((self.x2-self.x1)**2 + (self.y2-self.y1)**2 + (self.z2-self.z1)**2)**.5
        self._length = new_length
        scale = np.array(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, new_length/old_length, 0), (0, 0, 0, 1)))
        v = np.array(((self.x2-self.x1)/new_length, (self.y2-self.y1)/new_length, (self.z2-self.z1)/new_length))
        z = np.array((0., 0., 1.))
        a = np.cross(z, v)
        div = (a**2).sum()**0.5
        
        if div != 0:
            a /= div
            aa = a * acos(np.dot(z, v))

            self._rot = o3d.geometry.get_rotation_matrix_from_axis_angle(aa)
            rotation = np.zeros((4, 4))
            rotation[:3, :3] = self._rot
            rotation[3, 3] = 1
        else:
            self._rot = None
            rotation = np.array(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))

        translation = np.array(((1, 0, 0, (self.x1+self.x2)/2), (0, 1, 0, (self.y1+self.y2)/2), (0, 0, 1, (self.z1+self.z2)/2), (0, 0, 0, 1)))

        scene.scene_widget.scene.set_geometry_transform(name, translation @ rotation @ scale)
        color = self.color
        color = tuple((*color, 1)) if len(color) == 3 else color
        material = rendering.MaterialRecord()
        material.shader = "defaultLitTransparency"
        material.base_color = color
        scene.scene_widget.scene.modify_geometry_material(name, material)

    @property
    def x1(self) -> Number:
        '''The x-coordinate of the first endpoint.'''
        return self._x1
    
    @x1.setter
    def x1(self, x1:Number):
        self._x1 = x1

    @property
    def y1(self) -> Number:
        '''The y-coordinate of the first endpoint.'''
        return self._y1
    
    @y1.setter
    def y1(self, y1:Number):
        self._y1 = y1

    @property
    def z1(self) -> Number:
        '''The z-coordinate of the first endpoint.'''
        return self._z1
    
    @z1.setter
    def z1(self, z1:Number):
        self._z1 = z1

    @property
    def x2(self) -> Number:
        '''The x-coordinate of the second endpoint.'''
        return self._x2
    
    @x2.setter
    def x2(self, x2:Number):
        self._x2 = x2

    @property
    def y2(self) -> Number:
        '''The y-coordinate of the second endpoint.'''
        return self._y2
    
    @y2.setter
    def y2(self, y2:Number):
        self._y2 = y2

    @property
    def z2(self) -> Number:
        '''The z-coordinate of the second endpoint.'''
        return self._z2
    
    @z2.setter
    def z2(self, z2:Number):
        self._z2 = z2

    @property
    def width(self) -> Number:
        '''The line segment's width.'''
        return self._width
    
    @property
    def resolution(self) -> int:
        '''The line's resolution.
        
        The line is drawn as a small cylinder using triangles.
        `resolution` represents the amount of triangles that will be
        used.
        '''
        return self._resolution

    @property
    def color(self) -> ColorType:
        '''The line segment's color in RGBA format.'''
        return self._color
    
    @color.setter
    def color(self, color:ColorType):
        self._color = color
        # self._shape.paint_uniform_color(color[:3])
        # if len(color) == 4:
        #     self._material.base_color = (1, 1, 1, color[3])

    def getPointFrom(self) -> Point3D:
        '''Returns the line segment's first endpoint.
        
        Returns:
            The line segment's first endpoint as a `Point3D` object.
        '''
        return Point3D((self.x1, self.y1, self.z1))
    
    def getPointTo(self) -> Point3D:
        '''Returns the line segment's second endpoint.
        
        Returns:
            The line segment's second endpoint as a `Point3D` object.
        '''
        return Point3D((self.x2, self.y2, self.z2))

    def length(self) -> float:
        '''Calculates the length of the line segment.
        
        Returns:
            The length of the line segment.
        '''
        return ((self._x2-self._x1)**2 + (self._y2-self._y1)**2 + (self._z2-self._z1)**2)**.5
    
    def intersectsLine(self, l:Line3D) -> NDArray|None:
        '''Calculates the intersection point with another line segment.
        
        Args:
            l: The other line segment.
        
        Returns:
            The intersection point if the segments intersect, `None`
                otherwise.
        '''
        p1 = np.array((self.x1, self.y1, self.z1))
        p2 = np.array((self.x2, self.y2, self.z2))
        q1 = np.array((l.x1, l.y1, l.z1))
        q2 = np.array((l.x2, l.y2, l.z2))
        
        v1 = p2 - p1
        v2 = q2 - q1

        n = np.cross(v1, v2)

        if np.allclose(n, [0,0,0]):
            return None
        
        b = q1-p1
        t = np.cross(b, v2) / np.dot(v1, v2)
        u = np.cross(b, v1) / np.dot(v1, v2)

        intersection_point = p1 + t*v1
        return intersection_point



        


class Arrow3D(Line3D):
    '''A class used to represent an arrow in 3D space.'''

    def __init__(self, start:Point3D|NDArray3|List3|Tuple3, end:Point3D|NDArray3|List3|Tuple3, width:Number=1, resolution:int=20, color:ColorType=(0, 0, 0), cone_to_cylinder_ratio:Number=0.1):
        '''Inits Arrow3D given the arrow's start and end.

        Args:
            start: The coordinates of the arrow's start.
            end: The coordinates of the arrow's end.
            width: The width of the displayed arrow.
            resolution: The resolution of the displayed arrow.
            color: The color of the displayed arrow (RGB or RGBA).
            cone_to_cylinder_ratio: the percentage of the arrow's
                length that is taken up by the arrow head.
        '''
        super().__init__(start, end, width, resolution, color)
        self._ratio = cone_to_cylinder_ratio

    def _addToScene(self, scene:Scene3D, name:None|str):
        name = str(id(self)) if name is None else name
        scene._shapeDict[name] = self
        shape = o3d.geometry.TriangleMesh.create_arrow(0.005 * self.width, 2 * 0.005 * self.width, (1 - self.cone_to_cylinder_ratio) * self.length(), self.cone_to_cylinder_ratio * self.length(), self.resolution)
        shape.translate((0, 0, -self.length()/2))
        shape.compute_vertex_normals()

        v = np.array(((self.x2-self.x1)/self.length(), (self.y2-self.y1)/self.length(), (self.z2-self.z1)/self.length()))
        z = np.array((0., 0., 1.))
        a = np.cross(z, v)
        div = (a**2).sum()**0.5
        
        if div != 0:
            a /= div
            aa = a * acos(np.dot(z, v))

            self._rot = o3d.geometry.get_rotation_matrix_from_axis_angle(aa)
            rotation = np.zeros((4, 4))
            rotation[:3, :3] = self._rot
            rotation[3, 3] = 1
        else:
            self._rot = None
            rotation = np.array(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))

        translation = np.array(((1, 0, 0, (self.x1+self.x2)/2), (0, 1, 0, (self.y1+self.y2)/2), (0, 0, 1, (self.z1+self.z2)/2), (0, 0, 0, 1)))

        material = rendering.MaterialRecord()
        material.shader = "defaultLitTransparency"
        color = self.color
        color = tuple((*color, 1)) if len(color) == 3 else color
        material.base_color = color
        scene.scene_widget.scene.add_geometry(name, shape, material)
        scene.scene_widget.scene.set_geometry_transform(name, translation @ rotation)

    @property
    def cone_to_cylinder_ratio(self) -> Number:
        '''The percentage of the arrow's length that is taken up by the arrow head.'''
        return self._ratio

class Sphere3D(Shape):
    '''A class used to represent a sphere in 3D space.'''

    def __init__(self, p:Point3D|NDArray3|List3|Tuple3, radius:Number=1, resolution:int=20, width:Number=1, color:ColorType=(0, 0, 0), filled:bool=False):
        '''Inits Sphere3D given the sphere's center and radius.

        Args:
            p: The coordinates of the center.
            radius: The sphere's radius.
            resolution: The resolution of the displayed sphere.
            width: The width of the displayed sphere (if not filled).
            color: The color of the displayed sphere (RGB or RGBA).
            filled: Whether to fill in the sphere or draw only its
                outline.
        '''

        if isinstance(p, Point3D):
            self._x = p.x
            self._y = p.y
            self._z = p.z
        elif isinstance(p, (np.ndarray, list, tuple)):
            self._x = p[0]
            self._y = p[1]
            self._z = p[2]
        else:
            raise TypeError("Incorrect type for p")
        
        self._radius = radius
        self._resolution = resolution
        self._width = width
        self._color = [*color, 1] if len(color) == 3 else [*color]
        self._filled = filled

    def _addToScene(self, scene:Scene3D, name:None|str):
        name = str(id(self)) if name is None else name
        scene._shapeDict[name] = self
        shape = o3d.geometry.TriangleMesh.create_sphere(1, self.resolution)
        shape.compute_vertex_normals()
        material = rendering.MaterialRecord()
        material.shader = "defaultLitTransparency"
        if not self.filled:
            shape = o3d.geometry.LineSet.create_from_triangle_mesh(shape)
            material.shader = "unlitLine"
            material.line_width = 2 * self.width
        color = self.color
        color = tuple((*color, 1)) if len(color) == 3 else color
        material.base_color = color
        scene.scene_widget.scene.add_geometry(name, shape, material)
        scene.scene_widget.scene.set_geometry_transform(name, ((self.radius, 0, 0, self.x), (0, self.radius, 0, self.y), (0, 0, self.radius, self.z), (0, 0, 0, 1)))

    def _update(self, name:str, scene:Scene3D):
        scene.scene_widget.scene.set_geometry_transform(name, ((self.radius, 0, 0, self.x), (0, self.radius, 0, self.y), (0, 0, self.radius, self.z), (0, 0, 0, 1)))
        color = self.color
        color = tuple((*color, 1)) if len(color) == 3 else color
        material = rendering.MaterialRecord()
        if self.filled:
            material.shader = "defaultLitTransparency"
        else:
            material.shader = "unlitLine"
            material.line_width = 2 * self.width
        material.base_color = color
        scene.scene_widget.scene.modify_geometry_material(name, material)

    @property
    def x(self) -> Number:
        '''The x-coordinate of the sphere's center point.'''
        return self._x
    
    @x.setter
    def x(self, x:Number):
        self._x = x

    @property
    def y(self) -> Number:
        '''The y-coordinate of the sphere's center point.'''
        return self._y
    
    @y.setter
    def y(self, y:Number):
        self._y = y

    @property
    def z(self) -> Number:
        '''The z-coordinate of the sphere's center point.'''
        return self._z
    
    @z.setter
    def z(self, z:Number):
        self._z = z

    @property
    def radius(self) -> Number:
        '''The sphere's radius.'''
        return self._radius
    
    @radius.setter
    def radius(self, r:Number):
        self._radius = r
    
    @property
    def resolution(self) -> int:
        '''The sphere's resolution.
        
        The sphere is drawn using triangles. `resolution` represents
        the amount of triangles that will be used.
        '''
        return self._resolution
    
    @property
    def width(self) -> Number:
        '''The sphere's width (if not filled).'''
        return self._width
    
    @width.setter
    def width(self, width:Number):
        self._width = width

    @property
    def color(self) -> ColorType:
        '''The sphere's color in RGBA format.'''
        return self._color
    
    @color.setter
    def color(self, color:ColorType):
        self._color = color

    @property
    def filled(self) -> bool:
        '''Whether to fill in the sphere or draw only its outline.'''
        return self._filled

    def getPointCenter(self) -> Point3D:
        '''Returns the sphere's center.
        
        Returns:
            The sphere's center point as a `Point3D` object.
        '''
        return Point3D((self.x, self.y, self.z))
    
    def contains(self, point:Point3D) -> bool:
        '''Determines whether a point is inside the sphere.

        Args:
            point: The point to check (if it's inside the sphere).

        Returns:
            `True` if the point is inside the sphere (incl. the
                outline), `False` otherwise.
        '''
        return (self.x - point.x) ** 2 + (self.y - point.y) ** 2 + (self.z - point.z) ** 2 <= self.radius ** 2

class Cuboid3D(Shape):
    '''A class used to represent a cuboid in 3D space.
    
    A class used to represent a cuboid in 3D space. The cuboid has to
    be axis-aligned. If you need a cuboid that supports rotation, use
    the more flexible (but less robust) `Cuboid3DGeneralized`.
    '''

    def __init__(self, p1:Point3D|NDArray3|List3|Tuple3, p2:Point3D|NDArray3|List3|Tuple3, width:Number=1, color:ColorType=(0, 0, 0), filled:bool=False):
        '''Inits Cuboid3D given 2 vertices of the cuboid.

        Args:
            p1: The coordinates of one vertex.
            p2: The coordinates of another vertex.
            width: The width of the displayed cuboid (if not filled).
            color: The color of the displayed cuboid (RGB or RGBA).
            filled: Whether to fill in the cuboid or draw only its
                outline.
        '''
        
        if isinstance(p1, Point3D):
            x1 = p1.x
            y1 = p1.y
            z1 = p1.z
        elif isinstance(p1, (np.ndarray, list, tuple)):
            x1 = p1[0]
            y1 = p1[1]
            z1 = p1[2]
        else:
            raise TypeError("Incorrect type for p1")
        
        if isinstance(p2, Point3D):
            x2 = p2.x
            y2 = p2.y
            z2 = p2.z
        elif isinstance(p2, (np.ndarray, list, tuple)):
            x2 = p2[0]
            y2 = p2[1]
            z2 = p2[2]
        else:
            raise TypeError("Incorrect type for p2")
        
        self._x_min = min(x1, x2)
        self._y_min = min(y1, y2)
        self._z_min = min(z1, z2)
        self._x_max = max(x1, x2)
        self._y_max = max(y1, y2)
        self._z_max = max(z1, z2)

        self._width = width
        self._color = [*color, 1] if len(color) == 3 else [*color]
        self._filled = filled

    def _addToScene(self, scene:Scene3D, name:None|str):
        name = str(id(self)) if name is None else name
        scene._shapeDict[name] = self
        material = rendering.MaterialRecord()
        material.shader = "defaultLitTransparency"
        if self.filled:
            shape = o3d.geometry.TriangleMesh.create_box(1, 1, 1)
            shape.translate((-0.5, -0.5, -0.5))
            shape.compute_vertex_normals()
        else:
            vertices = np.array(((-0.5, -0.5, -0.5),
                                 (0.5, -0.5, -0.5),
                                 (0.5, 0.5, -0.5),
                                 (-0.5, 0.5, -0.5),
                                 (-0.5, -0.5, 0.5),
                                 (0.5, -0.5, 0.5),
                                 (0.5, 0.5, 0.5),
                                 (-0.5, 0.5, 0.5)))
            lines = np.array(((0, 1), (1, 2), (2, 3), (3, 0),
                              (4, 5), (5, 6), (6, 7), (7, 4),
                              (0, 4), (1, 5), (2, 6), (3, 7)))
            shape = o3d.geometry.LineSet(o3d.utility.Vector3dVector(vertices), o3d.utility.Vector2iVector(lines))
            material.shader = "unlitLine"
            material.line_width = 2 * self.width
        color = self.color
        color = tuple((*color, 1)) if len(color) == 3 else color
        material.base_color = color
        scene.scene_widget.scene.add_geometry(name, shape, material)
        scene.scene_widget.scene.set_geometry_transform(name, ((self._x_max - self._x_min, 0, 0, (self.x_max + self.x_min)/2), (0, self._y_max - self._y_min, 0, (self.y_max + self.y_min)/2), (0, 0, self._z_max - self._z_min, (self.z_max + self.z_min)/2), (0, 0, 0, 1)))

    def _update(self, name:str, scene:Scene3D):
        scene.scene_widget.scene.set_geometry_transform(name, ((self._x_max - self._x_min, 0, 0, (self.x_max + self.x_min)/2), (0, self._y_max - self._y_min, 0, (self.y_max + self.y_min)/2), (0, 0, self._z_max - self._z_min, (self.z_max + self.z_min)/2), (0, 0, 0, 1)))
        color = self.color
        color = tuple((*color, 1)) if len(color) == 3 else color
        material = rendering.MaterialRecord()
        if self.filled:
            material.shader = "defaultLitTransparency"
        else:
            material.shader = "unlitLine"
            material.line_width = 2 * self.width
        material.base_color = color
        scene.scene_widget.scene.modify_geometry_material(name, material)

    @property
    def x_min(self) -> Number:
        '''The x-coordinate of the bottom-left-back vertex.'''
        return self._x_min
    
    @x_min.setter
    def x_min(self, x:Number):
        self._x_min = x
        if self._x_min > self._x_max:
            self._x_min, self._x_max = self._x_max, self._x_min

    @property
    def y_min(self) -> Number:
        '''The y-coordinate of the bottom-left-back vertex.'''
        return self._y_min
    
    @y_min.setter
    def y_min(self, y:Number):
        self._y_min = y
        if self._y_min > self._y_max:
            self._y_min, self._y_max = self._y_max, self._y_min

    @property
    def z_min(self) -> Number:
        '''The z-coordinate of the bottom-left-back vertex.'''
        return self._z_min
    
    @z_min.setter
    def z_min(self, z:Number):
        self._z_min = z
        if self._z_min > self._z_max:
            self._z_min, self._z_max = self._z_max, self._z_min
    
    @property
    def x_max(self) -> Number:
        '''The x-coordinate of the top-right-front vertex.'''
        return self._x_max
    
    @x_max.setter
    def x_max(self, x:Number):
        self._x_max = x
        if self._x_max < self._x_min:
            self._x_max, self._x_min = self._x_min, self._x_max

    @property
    def y_max(self) -> Number:
        '''The y-coordinate of the top-right-front vertex.'''
        return self._y_max
    
    @y_max.setter
    def y_max(self, y:Number):
        self._y_max = y
        if self._y_max < self._y_min:
            self._y_max, self._y_min = self._y_min, self._y_max

    @property
    def z_max(self) -> Number:
        '''The z-coordinate of the top-right-front vertex.'''
        return self._z_max
    
    @z_max.setter
    def z_max(self, z:Number):
        self._z_max = z
        if self._z_max < self._z_min:
            self._z_max, self._z_min = self._z_min, self._z_max

    @property
    def width(self) -> Number:
        '''The cuboid's width (if not filled).'''
        return self._width
    
    @width.setter
    def width(self, w:Number):
        self._width = w

    @property
    def color(self) -> ColorType:
        '''The cuboid's color in RGBA format.'''
        return self._color
    
    @color.setter
    def color(self, color:ColorType):
        self._color = color

    @property
    def filled(self) -> bool:
        '''Whether to fill in the cuboid or draw only its outline.'''
        return self._filled

    def translate(self, translation:Point3D|NDArray3|List3|Tuple3):
        '''Translates the cuboid by a vector.

        Translates the cuboid by `translation`. This is mostly useful
        when the min/max coordinate values would switch if you applied
        the translation individually to each vertex.

        Args:
            translation: The translation vector. Its coordinates will
                be added to the cuboid's coordinates.
        '''
        if isinstance(translation, Point3D):
            x = translation.x
            y = translation.y
            z = translation.z
        elif isinstance(translation, (np.ndarray, list, tuple)):
            x = translation[0]
            y = translation[1]
            z = translation[2]
        else:
            raise TypeError("Incorrect type for translation")
        
        self._x_min += x
        self._y_min += y
        self._z_min += z
        self._x_max += x
        self._y_max += y
        self._z_max += z

class Cuboid3DGeneralized(Shape):
    '''A class used to represent a cuboid in 3D space.
    
    A class used to represent a cuboid in 3D space. The cuboid may be
    translated and rotated. If you need a cuboid that supports more
    complex deformations, use a `Mesh3D` instead.
    '''

    def __init__(self, cuboid:Cuboid3D):
        '''Inits Cuboid3DGeneralized from a Cuboid3D object.

        Args:
            cuboid: The `Cuboid3D` object to copy.
        '''
        if isinstance(cuboid, Cuboid3D):
            self._vertices = np.array([[cuboid.x_min, cuboid.y_min, cuboid.z_min],
                                       [cuboid.x_max, cuboid.y_min, cuboid.z_min],
                                       [cuboid.x_max, cuboid.y_max, cuboid.z_min],
                                       [cuboid.x_min, cuboid.y_max, cuboid.z_min],
                                       [cuboid.x_min, cuboid.y_min, cuboid.z_max],
                                       [cuboid.x_max, cuboid.y_min, cuboid.z_max],
                                       [cuboid.x_max, cuboid.y_max, cuboid.z_max],
                                       [cuboid.x_min, cuboid.y_max, cuboid.z_max]])
            self._triangles = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
                                        [0, 4, 7], [0, 7, 3], [5, 1, 2], [5, 2, 6],
                                        [7, 6, 2], [7, 2, 3], [0, 1, 5], [0, 5, 4]])
            
        else:
            raise TypeError("Incorrect type for cuboid")

        self._width = cuboid.width
        self._color = cuboid.color
        self._filled = cuboid.filled

    def _addToScene(self, scene:Scene3D, name:None|str):
        name = str(id(self)) if name is None else name
        scene._shapeDict[name] = self
        material = rendering.MaterialRecord()
        material.shader = "defaultLitTransparency"
        if self.filled:
            shape = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(self._vertices), o3d.utility.Vector3iVector(self._triangles))
            shape.compute_vertex_normals()
        else:
            lines = np.array(((0, 1), (1, 2), (2, 3), (3, 0),
                              (4, 5), (5, 6), (6, 7), (7, 4),
                              (0, 4), (1, 5), (2, 6), (3, 7)))
            shape = o3d.geometry.LineSet(o3d.utility.Vector3dVector(self._vertices), o3d.utility.Vector2iVector(lines))
            material.shader = "unlitLine"
            material.line_width = 2 * self.width
        color = self.color
        color = tuple((*color, 1)) if len(color) == 3 else color
        material.base_color = color
        scene.scene_widget.scene.add_geometry(name, shape, material)

    def _update(self, name:str, scene:Scene3D):
        scene.removeShape(name)
        self._addToScene(scene, name)

    @property
    def width(self) -> Number:
        '''The cuboid's width (if not filled).'''
        return self._width
    
    @width.setter
    def width(self, width:Number):
        self._width = width

    @property
    def color(self) -> ColorType:
        '''The cuboid's color in RGBA format.'''
        return self._color
    
    @color.setter
    def color(self, color:ColorType):
        self._color = color

    @property
    def filled(self) -> bool:
        '''Whether to fill in the cuboid or draw only its outline.'''
        return self._filled

    def translate(self, translation:Point3D|NDArray3|List3|Tuple3):
        '''Translates the cuboid by a vector.

        Translates the cuboid by `translation`.

        Args:
            translation: The translation vector. Its coordinates will
                be added to the cuboid's coordinates.
        '''
        if isinstance(translation, Point3D):
            x = translation.x
            y = translation.y
            z = translation.z
        elif isinstance(translation, (np.ndarray, list, tuple)):
            x = translation[0]
            y = translation[1]
            z = translation[2]
        else:
            raise TypeError("Incorrect type for translation")
        
        self._vertices += np.array((x, y, z))

    def rotate(self, angle:Number, axis:NDArray3|List3|Tuple3):
        '''Rotates the cuboid.

        Rotates the cuboid using a rotation represented as axis-angle.

        Args:
            angle: The angle to rotate the cuboid.
            axis: The axis about which to rotate the cuboid.
        
        '''
        if isinstance(axis, (np.ndarray, list, tuple)):
            axis = np.array(axis)
            axis = axis / np.linalg.norm(axis)
            rot = o3d.geometry.get_rotation_matrix_from_axis_angle(angle * axis)
            self._vertices = (rot @ self._vertices.T).T

        else:
            raise TypeError("Incorrect type for axis")

class PointSet3D(ShapeSet):
    '''A class used to represent a set of points in 3D space.'''

    def __init__(self, points:None|NDArray|List|Tuple=None, size:Number=1, color:ColorType=(0, 0, 0)):
        '''Inits PointSet3D.

        Inits a PointSet3D containing `points`. If `points` is `None`,
        the pointset will be initialized empty.

        Args:
            points: The points of the pointset.
            size: The size of the displayed points.
            color: The color of the displayed points (RGB or RGBA).
        '''
        self._points:list[List3] = []
        self._size = size
        self._opacity = color[3] if len(color) == 4 else 1
        self._colors:list[ColorType] = []

        if points is not None:
            if isinstance(points, (np.ndarray, list, tuple)):
                self._points = [list(_) for _ in points]
                self._colors = [[*color, 1] if len(color) == 3 else [*color] for _ in points]
            else:
                raise TypeError(f"Unsupported type for points: {type(points)}")
            
    def __len__(self):
        return len(self._points)

    def __getitem__(self, idx):
        return self.getPointAt(idx)

    def _addToScene(self, scene:Scene3D, name:None|str):
        name = str(id(self)) if name is None else name
        scene._shapeDict[name] = self
        if len(self) == 0:
            shape = o3d.geometry.PointCloud()
        else:
            shape = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.points))
            shape.colors = o3d.utility.Vector3dVector(self.colors[:,:3])
        material = rendering.MaterialRecord()
        material.shader = "defaultUnlit"
        material.point_size = 5 * self.size
        material.base_color = (1, 1, 1, self._opacity)
        scene.scene_widget.scene.add_geometry(name, shape, material)

        self._shape = shape
        self._material = material

    def _update(self, name:str, scene:Scene3D):
        # or use https://www.open3d.org/docs/latest/python_api/open3d.visualization.rendering.Scene.html#open3d.visualization.rendering.Scene.update_geometry
        scene.removeShape(name)
        scene._shapeDict[name] = self
        self._shape.points = o3d.utility.Vector3dVector(self.points) if len(self._points) > 0 else o3d.utility.Vector3dVector()
        self._shape.colors = o3d.utility.Vector3dVector(self.colors[:,:3]) if len(self._colors) > 0 else o3d.utility.Vector3dVector()
        self._material.point_size = 5 * self.size
        self._material.base_color = (1, 1, 1, self._opacity)
        scene.scene_widget.scene.add_geometry(name, self._shape, self._material)
        
    @property
    def points(self) -> NDArray:
        '''The points of the pointset.'''
        return np.array(self._points)
    
    @points.setter
    def points(self, points:NDArray):
        if isinstance(points, (np.ndarray, list, tuple)):
            self._points = [list(_) for _ in points]

    @property
    def size(self) -> Number:
        '''The size of the displayed points.'''
        return self._size
    
    @size.setter
    def size(self, size:Number):
        self._size = size

    @property
    def colors(self) -> NDArray:
        '''The points' colors in RGBA format.'''
        return np.array(self._colors)
    
    @colors.setter
    def colors(self, colors:NDArray|List|Tuple):
        if isinstance(colors, (np.ndarray, list, tuple)):
            self._colors = [list(_) for _ in colors]

    def getPointAt(self, index:int) -> Point3D:
        '''Returns the point at the specified index.

        Args:
            index: The index at which the desired point is placed
                inside the pointset.
        
        Returns:
            The point at the specified index as a `Point3D` object. It
                retains its size and color.
        '''
        return Point3D(self._points[index], self.size, color=self._colors[index])
        
    def add(self, point:Point3D):
        '''Appends a point to the pointset.
        
        Args:
            point: The `Point3D` object to append.
        '''
        self._points.append([point.x, point.y, point.z])
        self._colors.append(point.color)

    def createRandom(self, bound:Cuboid3D, num_points:int, seed:None|int|str=None, color:ColorType=(0, 0, 0)):
        '''Appends random points to the pointset.

        Uniformly generates random points inside a region and appends
        them to the pointset.
        
        Args:
            bound: The area inside of which the random points will be
                generated.
            num_points: How many points to generate.
            seed: An optional seed for the RNG.
            color: The color of the generated points.
        '''
        if len(self.points) > 0:
            print('Point Set is not empty; random points will be appended to the existing ones.')

        if seed:
            if isinstance(seed, str):
                seed = seed.encode()
                seed = int.from_bytes(seed, "big") % (2<<32)
            np.random.seed(seed)

        if issubclass(type(bound), Cuboid3D):
            x1 = bound.x_min
            y1 = bound.y_min
            z1 = bound.z_min
            x2 = bound.x_max
            y2 = bound.y_max
            z2 = bound.z_max

            random_array = np.random.random_sample((num_points, 3))
            pts = random_array * np.array((x2-x1, y2-y1, z2-z1)) + np.array((x1, y1, z1))
            for p in pts:
                self._points.append(p)
                self._colors.append([*color, 1] if len(color) == 3 else [*color])

        else:
            raise TypeError
        
    def remove(self, index:int):
        '''Removes a point from the pointset.

        Removes a point from the pointset's specified index.
        
        Args:
            index: The index at which the to-be-removed point is placed
                inside the pointset.
        '''
        self._points.pop(index)
        self._colors.pop(index)
        
    def clear(self):
        '''Clears the pointset.

        Clears the pointset, completely removing all points and
        information about them (e.g., color).
        '''
        self._points.clear()
        self._colors.clear()
        
    def getAABB(self) -> Cuboid3D:
        '''Returns the AABB of the pointset.

        Returns the Axis Aligned Bounding Box of the points in the
        pointset.

        Returns:
            The AABB of the pointset.
        '''
        if len(self) > 1:
            points = self.points
            return Cuboid3D(points.min(axis=0), points.max(axis=0))
        else:
            raise RuntimeError("PointSet3D object must contain at least 2 points to define the AABB")
        
    def remove_duplicated_points(self):
        '''Removes points that exist multiple times in the pointset.'''
        try:
            self._shape.remove_duplicated_points()
        except:
            raise NotImplementedError("Currently only works after adding the pointset to a scene!")

class LineSet3D(ShapeSet):
    '''A class used to represent a set of lines in 3D space.'''

    def __init__(self, points:None|NDArray|List|Tuple=None, lines:None|NDArray|List|Tuple=None, width:Number=1, color:ColorType=(0, 0, 0)):
        '''Inits LineSet3D.

        Inits a LineSet3D containing `points` connected according to
        `lines`.
        
        If `lines` is `None`, the `points` will be connected in pairs
        i.e., (0, 1), (2, 3), etc.
        
        If `points` is `None`, the lineset will be initialized empty.

        Args:
            points: The points of the lineset.
            lines: The indices in `points` that are connected by a
                line.
            width: The width of the displayed lines.
            color: The color of the displayed lines (RGB or RGBA).
        '''
        self._points:list[List3] = []
        self._lines:list[List2] = []

        self._width = width
        self._opacity = color[3] if len(color) == 4 else 1
        self._colors:list[ColorType] = []

        if isinstance(points, PointSet3D):
            points = points.points

        if points is not None and lines is None:
            if isinstance(points, (np.ndarray, list, tuple)) and len(points) % 2 == 0:
                lines = [[i,i+1] for i in range(0, len(points), 2)]

        if points is not None and lines is not None:
            if isinstance(points, (np.ndarray, list, tuple)):
                self._points = [list(_) for _ in points]
            if isinstance(lines, (np.ndarray, list, tuple)):
                self._lines = [list(_) for _ in lines]

            self._colors = [[*color, 1] if len(color) == 3 else [*color] for _ in lines]
    
    def __len__(self):
        return len(self._lines)
    
    def __getitem__(self, idx):
        return self.getLineAt(idx)

    def _addToScene(self, scene:Scene3D, name:None|str):
        name = str(id(self)) if name is None else name
        scene._shapeDict[name] = self
        if len(self) == 0:
            shape = o3d.geometry.LineSet()
        else:
            shape = o3d.geometry.LineSet(o3d.utility.Vector3dVector(self.points), o3d.utility.Vector2iVector(self.lines))
            shape.colors = o3d.utility.Vector3dVector(self.colors[:,:3])
        material = rendering.MaterialRecord()
        material.shader = "unlitLine"
        material.line_width = 2 * self.width
        material.base_color = (1, 1, 1, self._opacity)
        scene.scene_widget.scene.add_geometry(name, shape, material)

        self._shape = shape
        self._material = material

    def _update(self, name:str, scene:Scene3D):
        # unfortunately, open3d does not support updating linesets yet; do it the ol' fashioned way
        scene.removeShape(name)
        scene._shapeDict[name] = self
        if len(self) == 0:
            self._shape.clear()
        else:
            self._shape.lines = o3d.utility.Vector2iVector(self.lines)
            self._shape.points = o3d.utility.Vector3dVector(self.points)
            self._shape.colors = o3d.utility.Vector3dVector(self.colors[:,:3])
        self._material.line_width = 2 * self.width
        self._material.base_color = (1, 1, 1, self._opacity)
        scene.scene_widget.scene.add_geometry(name, self._shape, self._material)

    @property
    def points(self) -> NDArray:
        '''The points of the lineset.'''
        return np.array(self._points)
    
    @points.setter
    def points(self, points:NDArray|List|Tuple):
        if isinstance(points, (np.ndarray, list, tuple)):
            self._points = [list(_) for _ in points]

    @property
    def lines(self) -> NDArray:
        '''The point indices indicating lines of the lineset.'''
        return np.array(self._lines)
    
    @lines.setter
    def lines(self, lines:NDArray|List|Tuple):
        if isinstance(lines, (np.ndarray, list, tuple)):
            self._lines = [list(_) for _ in lines]

    @property
    def width(self) -> Number:
        '''The width of the displayed lines.'''
        return self._width
    
    @width.setter
    def width(self, width:Number):
        self._width = width

    @property
    def colors(self) -> NDArray:
        '''The lines' colors in RGBA format.'''
        return np.array(self._colors)
    
    @colors.setter
    def colors(self, colors:NDArray|List|Tuple):
        if isinstance(colors, (np.ndarray, list, tuple)):
            self._colors = [list(_) for _ in colors]

    def getLineAt(self, index:int) -> Line3D:
        '''Returns the line at the specified index.

        Args:
            index: The index at which the desired line is placed
                inside the lineset.
        
        Returns:
            The line at the specified index as a `Line3D` object. It
                retains its width and color.
        '''
        return Line3D(self._points[self._lines[index][0]], self._points[self._lines[index][1]], self.width, color=self.colors[index])

    def add(self, line:Line3D):
        '''Appends a line to the lineset.
        
        Args:
            line: The `Line3D` object to append.
        '''
        idx1 = idx2 = None
        if len(self._points) == 0:
            self._points.append([line.x1, line.y1, line.z1])

        for i, p in enumerate(self._points):
            if p[0] == line.x1 and p[1] == line.y1 and p[2] == line.z1:
                idx1 = i
            if p[0] == line.x2 and p[1] == line.y2 and p[2] == line.z2:
                idx2 = i

        if idx1 == None:
            self._points.append([line.x1, line.y1, line.z1])
            idx1 = len(self._points) - 1
        if idx2 == None:
            self._points.append([line.x2, line.y2, line.z2])
            idx2 = len(self._points) - 1

        self._lines.append([idx1, idx2])
        self._colors.append([*line.color])

    def remove(self, index:int):
        '''Removes a line from the lineset.

        Removes a line from the lineset's specified index (does not
        affect LineSet3D.points in any way).
        
        Args:
            index: The index at which the to-be-removed line is placed
                inside the lineset.
        '''
        self._lines.pop(index)
        self._colors.pop(index)

    def clear(self):
        '''Clears the lineset.

        Clears the lineset, completely removing all points, lines and
        information about them (e.g., color).
        '''
        self._points.clear()
        self._lines.clear()
        self._colors.clear()

    @staticmethod
    def create_from_mesh(mesh:Mesh3D, width:Number=1, color:ColorType=(0, 0, 0)) -> LineSet3D:
        '''Creates a Lineset3D object from a Mesh3D object.

        Args:
            mesh: The mesh to be turned into a lineset.
            width: The width of the displayed lines.
            color: The color of the displayed lines (RGB or RGBA).

        Returns:
            The Lineset3D object extracted from the mesh.
        '''
        tm = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(mesh.vertices), o3d.utility.Vector3iVector(mesh.triangles))
        ls = o3d.geometry.LineSet.create_from_triangle_mesh(tm)
        return LineSet3D(np.asarray(ls.points), np.asarray(ls.lines), width, color)

class Mesh3D(ShapeSet):
    '''A class used to represent a triangle mesh in 3D space.'''

    def __init__(self, path:None|str=None, color:ColorType=(0, 0, 0)):
        '''Inits Mesh3D.

        Inits a Mesh3D from a specified path.

        Args:
            path: The path to a file describing a triangle mesh.
            color: The color of the displayed mesh (RGB or RGBA).
        '''
        self._color = [*color, 1] if len(color) == 3 else [*color]

        if path is not None:
            self._shape = o3d.io.read_triangle_mesh(path)
        else:
            self._shape = o3d.geometry.TriangleMesh()
        self._material = rendering.MaterialRecord()
        self._material.shader = "defaultLitTransparency"
        self._material.base_color = (*color[:3], color[3] if len(color) == 4 else 1)
        if not self._shape.has_vertex_normals():
            self._shape.compute_vertex_normals()
        if not self._shape.has_triangle_normals():
            self._shape.compute_triangle_normals()

    def _addToScene(self, scene:Scene3D, name:None|str):
        name = str(id(self)) if name is None else name
        scene._shapeDict[name] = self
        if not self._shape.has_triangle_normals():
            self._shape.compute_triangle_normals()
        if not self._shape.has_triangle_normals():
            self._shape.compute_triangle_normals()
        scene.scene_widget.scene.add_geometry(name, self._shape, self._material)

    def _update(self, name:str, scene:Scene3D):
        scene.removeShape(name)
        self._addToScene(scene, name)

    @property
    def vertices(self) -> NDArray:
        '''The vertices of the mesh.'''
        return np.copy(np.asarray(self._shape.vertices))
    
    @vertices.setter
    def vertices(self, vertices:NDArray|List|Tuple):
        self._shape.vertices = o3d.utility.Vector3dVector(vertices)

    @property
    def triangles(self) -> NDArray:
        '''The triangles (as indices to `points`) of the mesh.'''
        return np.copy(np.asarray(self._shape.triangles))
    
    @triangles.setter
    def triangles(self, triangles:NDArray|List|Tuple):
        self._shape.triangles = o3d.utility.Vector3iVector(triangles)

    @property
    def vertex_normals(self) -> NDArray:
        '''The normals of each vertex.'''
        return np.copy(np.asarray(self._shape.vertex_normals))
    
    @vertex_normals.setter
    def vertex_normals(self, normals:NDArray|List|Tuple):
        self._shape.vertex_normals = o3d.utility.Vector3dVector(normals)

    @property
    def triangle_normals(self) -> NDArray:
        '''The normals of each triangle.'''
        return np.copy(np.asarray(self._shape.triangle_normals))
    
    @triangle_normals.setter
    def triangle_normals(self, normals:NDArray|List|Tuple):
        self._shape.triangle_normals = o3d.utility.Vector3dVector(normals)

    @property
    def color(self) -> ColorType:
        '''The mesh's color in RGBA format.'''
        return self._color
    
    @color.setter
    def color(self, color:ColorType):
        self._color = color
        self._material.base_color = (*color[:3], color[3] if len(color) == 4 else 1)

    @property
    def vertex_colors(self) -> NDArray:
        '''A specific color for each vertex.'''
        if not self._shape.has_vertex_colors():
            self._shape.paint_uniform_color(self._color[:3])
        return np.copy(np.asarray(self._shape.vertex_colors))
    
    @vertex_colors.setter
    def vertex_colors(self, colors:NDArray|List|Tuple):
        self._shape.vertex_colors = o3d.utility.Vector3dVector(colors)

    def remove_duplicated_vertices(self):
        '''Removes duplicated vertices.'''
        self._shape.remove_duplicated_vertices()
        self._shape.compute_vertex_normals()

    def remove_unreferenced_vertices(self):
        '''Removes unreferenced vertices.'''
        self._shape.remove_unreferenced_vertices()
        self._shape.compute_vertex_normals()

    @staticmethod
    def create_bunny(color:ColorType=(0, 0, 0)) -> Mesh3D:
        '''Creates a mesh of the Stanford Bunny.
        
        Returns:
            The `Mesh3D` object of the Stanford Bunny.
        '''
        m = Mesh3D(o3d.data.BunnyMesh().path, color)
        m.remove_unreferenced_vertices()
        return m
    
    @staticmethod
    def create_armadillo(color:ColorType=(0, 0, 0)) -> Mesh3D:
        '''Creates a mesh of the Stanford Armadillo.
        
        Returns:
            The `Mesh3D` object of the Stanford Armadillo.
        '''
        m = Mesh3D(o3d.data.ArmadilloMesh().path, color)
        m.vertices = (((-1, 0, 0), (0, 1, 0), (0, 0, -1)) @ m.vertices.T).T
        m.vertex_normals = (((-1, 0, 0), (0, 1, 0), (0, 0, -1)) @ m.vertex_normals.T).T
        return m

class Triangle3D(Mesh3D):
    '''A class used to represent a triangle in 3D space.'''

    def __init__(self, p1:NDArray, p2:NDArray, p3:NDArray, color:ColorType=(0, 0, 0)):
        '''Inits Triangle3D.

        Inits a Triangle3D from three points.

        Args:
            p1: The first point of the triangle.
            p2: The second point of the triangle.
            p3: The third point of the triangle.
            color: The color of the displayed triangle (RGB or RGBA).
        '''
        self._color = [*color, 1] if len(color) == 3 else [*color]
        self._shape = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        self._material = rendering.MaterialRecord()
        self._material.shader = "defaultLitTransparency"
        self._material.base_color = (*color[:3], color[3] if len(color) == 4 else 1)

        self._shape.vertices = o3d.utility.Vector3dVector([p1, p2, p3])
        self._shape.triangles = o3d.utility.Vector3iVector([[0, 1, 2]])

    @property
    def p1(self) -> NDArray:
        '''The first point of the triangle.'''
        return np.copy(np.asarray(self._shape.vertices[0]))
    
    # @p1.setter
    # def p1(self, p1:Point3D):
    #     self._shape.vertices[0] = p1

    @property
    def p2(self) -> NDArray:
        '''The second point of the triangle.'''
        return np.copy(np.asarray(self._shape.vertices[1]))
    
    # @p2.setter
    # def p2(self, p2:Point3D):
    #     self._shape.vertices[1] = p2

    @property
    def p3(self) -> NDArray:
        '''The third point of the triangle.'''
        return np.copy(np.asarray(self._shape.vertices[2]))
    
    # @p3.setter
    # def p3(self, p3:Point3D):
    #     self._shape.vertices[2] = p3

    def __eq__(self, other:Triangle3D) :
        return np.array_equal(self.p1, other.p1) and np.array_equal(self.p2, other.p2) and np.array_equal(self.p3, other.p3)
    
    # def intersects(self, other:Triangle3D) :
    #     '''Checks if this triangle intersects with another.

    #     Args:
    #         other: The other triangle to check for intersection.
        
    #     Returns:
    #         Whether the two triangles intersect.
    #     '''
    #     return self.points_on_same_side(other.p1, other.p2) and self.points_on_same_side(other.p2, other.p3) and self.points_on_same_side(other.p3, other.p1)
    
    def containsPoint(self, p:NDArray):
        '''Checks if the triangle contains a point.

        Args:
            p: The point to check for containment.
        
        Returns:
            Whether the triangle contains the point.
        '''
        p0 = self.p1
        p1 = self.p2
        p2 = self.p3
        v0 = [p2[0] - p0[0], p2[1] - p0[1]]
        v1 = [p1[0] - p0[0], p1[1] - p0[1]]
        v2 = [p[0] - p0[0], p[1] - p0[1]]

        # Compute dot products
        dot00 = v0[0] * v0[0] + v0[1] * v0[1]
        dot01 = v0[0] * v1[0] + v0[1] * v1[1]
        dot02 = v0[0] * v2[0] + v0[1] * v2[1]
        dot11 = v1[0] * v1[0] + v1[1] * v1[1]
        dot12 = v1[0] * v2[0] + v1[1] * v2[1]

        # Compute barycentric coordinates
        if (dot00 * dot11 - dot01 * dot01) == 0:
            return False
        inv_denom = 1 / (dot00 * dot11 - dot01 * dot01)
        u = (dot11 * dot02 - dot01 * dot12) * inv_denom
        v = (dot00 * dot12 - dot01 * dot02) * inv_denom

        # Check if point is inside the triangle
        e=0.001
        return (u >= -e) and (v >= -e) and (u + v <= 1+e)


    
    def getLineIntersection(self, l:Line3D):
        '''Calculates the intersection point with another line segment.
        
        Args:
            l: The other line segment.
        
        Returns:
            The intersection point if the segments intersect, `None`
                otherwise.
        '''

        # Get the plane equation
        v1 = self.p2 - self.p1
        v2 = self.p3 - self.p1
        normal = np.cross(v1, v2)
        point_on_plane = self.p1

        d = -np.dot(normal, point_on_plane)

        plane_eq = np.concatenate((normal, [d]))

        # Find the plane-line intersection point
        line_point = np.array([l.getPointFrom().x, l.getPointFrom().y, l.getPointFrom().z])
        line_end_point = np.array([l.getPointTo().x, l.getPointTo().y, l.getPointTo().z])
        direction = line_end_point - line_point
        
        a,b,c,d = plane_eq
        x0, y0, z0 = line_point
        vx, vy, vz = direction
        if a*vx + b*vy + c*vz == 0:
            return None
        t = (-a*x0 - b*y0 - c*z0 - d) / (a*vx + b*vy + c*vz)
        e = 0.001
        if t < 0-e or t > 1 + e:
            return None
        else:
            intersection = np.array([x0 + vx*t, y0 + vy*t, z0 + vz*t])
            # Check if it is inside the triangle
            if self.containsPoint(intersection):
                return intersection
            

        


        

    def points_on_same_side(self, p1:NDArray, p2:NDArray): #-> bool:
        '''Checks if two points are on the same side of the triangle.

        Args:
            p1: The first point to check.
            p2: The second point to check.
        
        Returns:
            Whether the two points are on the same side of the triangle.
        '''
        n = np.cross(self.p2 - self.p1, self.p3 - self.p1)
        return np.dot(n, p1 - self.p1) * np.dot(n, p2 - self.p1) >= -0.0001



class ConvexPolygon3D(Mesh3D):
    '''A class used to represent a convex polygon in 3D space. All points must be in the same plane.'''

    def __init__(self, points:NDArray, normal:list|NDArray|None = None, color:ColorType=(0, 0, 0)):
        '''Inits ConvexPolygon3D.

        Inits a ConvexPolygon3D from a set of points.

        Args:
            points: The points of the convex polygon.
            color: The color of the displayed polygon (RGB or RGBA).
        '''
        self._color = [*color, 1] if len(color) == 3 else [*color]
        self._shape = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        self._material = rendering.MaterialRecord()
        self._material.shader = "defaultLitTransparency"
        self._material.base_color = (*color[:3], color[3] if len(color) == 4 else 1)


        # Add vertices
        if normal is None:
            normal = np.cross(points[1] - points[0], points[2] - points[0])
        self.normal=np.array(normal)
        center = np.mean(points, axis=0)
        self.center = center
        sorted_points = self.order_points(points)
        self._shape.vertices = o3d.utility.Vector3dVector(sorted_points)

        # Add triangles
        triangles=[]
        triangles.append([0, 1, len(sorted_points)-1])
        for i in range(1, len(sorted_points)-1):
            triangles.append([0, i, i+1])
        
        self._shape.triangles = o3d.utility.Vector3iVector(triangles)
    
    def collides(self, other:ConvexPolygon3D):
        '''Checks if this convex polygon collides with another.

        Args:
            other: The other convex polygon to check for collision.
        
        Returns:
            Whether the two convex polygons collide.
        '''

        def triangles_intersect(t1:NDArray, t2:NDArray):

            triangle = Triangle3D(t1[0], t1[1], t1[2])
            lines = [Line3D(t2[0], t2[1]), Line3D(t2[1], t2[2]), Line3D(t2[2], t2[0])]
            for line in lines:
                intersection=triangle.getLineIntersection(line)
                if intersection is not None:
                    return True
            
        
        triangles = self.vertices[self._shape.triangles]
        other_triangles = other.vertices[other._shape.triangles]

        for triangle in triangles:
            for other_triangle in other_triangles:
                if triangles_intersect(triangle, other_triangle):
                    return True
    

    def points_on_same_side(self, p1:NDArray, p2:NDArray): #-> bool:
        self.normal = np.cross(self.points[1] - self.points[0], self.points[2] - self.points[0])

        plane_eq = np.concatenate((self.normal, [np.dot(self.normal, self.center)]))

        d1 = np.dot(plane_eq[:3], p1) + plane_eq[3]
        d2 = np.dot(plane_eq[:3], p2) + plane_eq[3]
        if d1 * d2 < 0:
            return False
        return True

    
    def order_points(self, points:NDArray) -> NDArray:
        '''Orders the points of the convex polygon.

        Orders the points of the convex polygon in a clockwise manner.

        Args:
            points: The points to order.
        
        Returns:
            The points in clockwise order.
        '''

        def get_angle(point1, point2, point3):
                # Form vectors from the given points
                vector1 = point1 - point2
                vector2 = point3 - point2
                
                # Calculate the cross product
                cross_product = np.cross(vector1, vector2)
                
                # Calculate the dot product
                dot_product = np.dot(vector1, vector2)
                
                # Calculate the angle in radians
                angle_rad = np.arctan2(np.dot(cross_product, self.normal), dot_product)
                
                # Convert angle to degrees
                angle_deg = np.degrees(angle_rad) % 360  # Ensure the angle is within [0, 360) range
                
                return angle_deg
        center= np.mean(points, axis=0)
        points = list(points)
        first_point = points[0]
        points.remove(first_point)

        sorted_points = sorted(points, key=lambda x: get_angle(first_point, center, x))
        sorted_points.insert(0, first_point)
        sorted_points.insert(0, center)
        return np.array(sorted_points)
    
      

    @property
    def points(self) -> NDArray:
        '''The points of the convex polygon.'''
        return np.copy(np.asarray(self._shape.vertices))
    
    @points.setter
    def points(self, points:NDArray|List|Tuple):
        self._shape.vertices = o3d.utility.Vector3dVector(points)
        self.center = np.mean(points, axis=0)
        self.normal = np.cross(points[1] - points[0], points[2] - points[0])


class Polyhedron3D(Mesh3D):
    def __init__(self, faces:list[ConvexPolygon3D]|NDArray, name: str, color:ColorType=(0, 0, 0)):
        self._color = [*color, 1] if len(color) == 3 else [*color]
        self._polygons = []
        self._shape = o3d.geometry.TriangleMesh()
        self._material = rendering.MaterialRecord()
        self._material.shader = "defaultLitTransparency"
        self._material.base_color = (*color[:3], color[3] if len(color) == 4 else 1)

        _vertices=[]
        _triangles=[]

        self.name = name

        for face in faces:
            self._polygons.append(face)

            number_of_vertices=len(_vertices)
            _vertices.extend(face.vertices)

            face_triangles = face.triangles
            for triangle in face_triangles:
                _triangles.append(np.array([i+number_of_vertices for i in triangle]))

        self._shape.vertices = o3d.utility.Vector3dVector(_vertices)
        self._shape.triangles = o3d.utility.Vector3iVector(_triangles)
    
    def collides(self, other:Polyhedron3D):
        for polygon in self._polygons:
            for other_polygon in other._polygons:
                if polygon.collides(other_polygon):
                    return True
        return False



    def collides_points(self, other:Polyhedron3D, show=True, scene=None):
        import random
        center = np.mean(self.vertices, axis=0)
        other_points = other.vertices
        print(len(other_points))
        print("other polyhedron:", other.name)
        for i in range(10):
            for j in range(10):
                for k in range(10):
                    div=5
                    if self.contains_point(np.array([i/div, j/div, k/div]), center):
                        scene.addShape(Point3D([i/div, j/div, k/div], color=(1, 0, 0), size=1), name="point"+f"{i}|{j}|{k}")
                    else:
                        scene.addShape(Point3D([i/div, j/div, k/div], color=(0, 1, 0), size=1), name="point"+f"{i}|{j}|{k}")
        return
        for other_polygon in other._polygons:
            for i, point in enumerate(other_polygon.vertices):
                # if i==0: # Center of the polygon
                #     continue
                
                    
                if not self.contains_point(point, center):
                    continue

                if show and scene:
                    print("index", i, "point", point)
                    print("Collision detected")
                    scene.addShape(Point3D(point, color=(0, 0, 1), size=5))

                return True
        return False
    
    def contains_point(self, point:NDArray, center:NDArray):
        polygon_index = -1
        for i, polygon in enumerate(self._polygons):
            if not polygon.points_on_same_side(center, point):
                return False
        
        return True
    
    @property
    def vertices(self) -> NDArray:
        '''The vertices of the mesh.'''
        return np.copy(np.asarray(self._shape.vertices))
    
    @vertices.setter
    def vertices(self, vertices:NDArray|List|Tuple):
        self._shape.vertices = o3d.utility.Vector3dVector(vertices)

    @property
    def triangles(self) -> NDArray:
        '''The triangles (as indices to `points`) of the mesh.'''
        return np.copy(np.asarray(self._shape.triangles))
    
    @triangles.setter
    def triangles(self, triangles:NDArray|List|Tuple):
        self._shape.triangles = o3d.utility.Vector3iVector(triangles)

    def _addToScene(self, scene: Scene3D, name: None | str):
        for i, polygon in enumerate(self._polygons):
            polygon._addToScene(scene, name + f"_face_{i}")
            # print("Added polygon", name+f"_face_{i}")
        scene._shapeDict[name] = self
        print(name)
    


    def _update(self, name: str, scene: Scene3D):
        for i, polygon in enumerate(self._polygons):
            polygon._update(scene, name + f"_{i}")
        scene._shapeDict[name] = self


