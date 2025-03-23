'''
Standard types for use throughout the project
'''
from dataclasses import dataclass, field
from abc import ABC, abstractmethod
import inspect
from typing import get_type_hints
import copy
import numpy as np

@dataclass
class PythonMsg:
    '''
    base class for creating types and messages in python
    '''
    def __setattr__(self,key,value):
        '''
        Overloads default atribute-setting functionality
          to avoid creating new fields that don't already exist
        This exists to avoid hard-to-debug errors from accidentally
          adding new fields instead of modifying existing ones

        To avoid this, use:
        object.__setattr__(instance, key, value)
        ONLY when absolutely necessary.
        '''
        if not hasattr(self,key):
            raise TypeError (f'Cannot add new field "{key}" to frozen class {self}')
        else:
            object.__setattr__(self,key,value)

    def print(self, indent = 0):
        ''' a more pretty way to print data '''
        indent = max(indent, 0)

        if indent == 0:
            print(' ' * indent + type(self).__name__)
        else:
            print(f'({type(self).__name__})')

        indent += 2
        for key in vars(self):
            attr = getattr(self, key)
            if isinstance(attr, PythonMsg):
                print(' ' * indent + f'{key} : ', end='')
                attr.print(indent = indent)
            else:
                print(' ' * indent + f'{key} : {attr}')

    def copy(self):
        ''' creates a copy of this class instance'''
        return copy.deepcopy(self)

@dataclass
class VectorizablePythonMsg(PythonMsg, ABC):
    ''' structure that can be converted to/from a vector '''
    @abstractmethod
    def to_vec(self) -> np.ndarray:
        ''' convert structure to vector '''

    @abstractmethod
    def from_vec(self, vec: np.ndarray) -> None:
        ''' update structure from vector '''

@dataclass
class NestedPythonMsg(PythonMsg):
    '''
    structure that is nested
    adds helper to create classes, set default values to None to use
    '''

    def __post_init__(self):
        for ancestor_class in type(self).mro():
            for name, type_hint in get_type_hints(ancestor_class).items():
                if not inspect.isclass(type_hint):
                    continue
                elif not issubclass(type_hint, PythonMsg):
                    continue
                elif getattr(self, name) is None:
                    setattr(self, name, type_hint())

@dataclass
class Position(VectorizablePythonMsg):
    ''' 3D position in the global frame'''
    xi: float = field(default = 0.)
    xj: float = field(default = 0.)
    xk: float = field(default = 0.)

    def L2dist(self, x: 'Position'):
        ''' L2 (Euclidean) distance with another position '''
        return np.sqrt((self.xi - x.xi)**2 + (self.xj - x.xj)**2 + (self.xk - x.xk)**2)

    def to_vec(self):
        return np.array([self.xi, self.xj, self.xk])

    def from_vec(self, vec):
        self.xi, self.xj, self.xk = vec

@dataclass
class BodyPosition(VectorizablePythonMsg):
    '''
    3D position in the body frame
    vehicle COM is always at 0,0,0
    '''
    x1: float = field(default = 0.)
    x2: float = field(default = 0.)
    x3: float = field(default = 0.)

    def to_vec(self):
        return np.array([self.x1, self.x2, self.x3])

    def from_vec(self, vec):
        self.x1, self.x2, self.x3 = vec

@dataclass
class BodyLinearVelocity(VectorizablePythonMsg):
    ''' body frame linear velocity '''
    v1: float = field(default = 0.)
    v2: float = field(default = 0.)
    v3: float = field(default = 0.)

    def mag(self):
        ''' magnitutde (speed) '''
        return (self.v1**2 + self.v2**2 + self.v3**2)**0.5

    def signed_mag(self):
        ''' magntitude, but negative if moving backwards '''
        return self.mag() * np.sign(self.v1)

    def to_vec(self):
        return np.array([self.v1, self.v2, self.v3])

    def from_vec(self, vec):
        self.v1, self.v2, self.v3 = vec

@dataclass
class BodyAngularVelocity(VectorizablePythonMsg):
    ''' body frame angular velocity '''
    w1: float = field(default = 0.)
    w2: float = field(default = 0.)
    w3: float = field(default = 0.)

    def to_vec(self):
        return np.array([self.w1, self.w2, self.w3])

    def from_vec(self, vec):
        self.w1, self.w2, self.w3 = vec

@dataclass
class OrientationQuaternion(VectorizablePythonMsg):
    ''' global frame orientation '''
    qr: float = field(default = 1.)
    qi: float = field(default = 0.)
    qj: float = field(default = 0.)
    qk: float = field(default = 0.)

    def e1(self):
        '''
        longitudinal basis vector
        points in same direction the vehicle does
        '''
        return np.array([[1 - 2*self.qj**2   - 2*self.qk**2,
                          2*(self.qi*self.qj + self.qk*self.qr),
                          2*(self.qi*self.qk - self.qj*self.qr)]]).T

    def e2(self):
        '''
        lateral basis vector
        points to left side of vehicle from driver's perspective
        '''
        return np.array([[2*(self.qi*self.qj - self.qk*self.qr),
                          1 - 2*self.qi**2   - 2*self.qk**2,
                          2*(self.qj*self.qk + self.qi*self.qr)]]).T

    def e3(self):
        '''
        normal basis vector
        points towards top of vehicle
        '''
        return np.array([[2*(self.qi*self.qk + self.qj*self.qr),
                          2*(self.qj*self.qk - self.qi*self.qr),
                          1 - 2*self.qi**2    - 2*self.qj**2]]).T

    def R(self):
        # pylint: disable=line-too-long
        '''
        rotation matrix
        '''
        return np.array([[1 - 2*self.qj**2 - 2*self.qk**2,       2*(self.qi*self.qj - self.qk*self.qr), 2*(self.qi*self.qk + self.qj*self.qr)],
                         [2*(self.qi*self.qj + self.qk*self.qr), 1 - 2*self.qi**2 - 2*self.qk**2,       2*(self.qj*self.qk - self.qi*self.qr)],
                         [2*(self.qi*self.qk - self.qj*self.qr), 2*(self.qj*self.qk + self.qi*self.qr), 1 - 2*self.qi**2 - 2*self.qj**2      ]])

    def Rinv(self):
        # pylint: disable=line-too-long
        '''
        inverse rotation matrix
        '''
        return np.array([[1 - 2*self.qj**2 - 2*self.qk**2,       2*(self.qi*self.qj + self.qk*self.qr), 2*(self.qi*self.qk - self.qj*self.qr)],
                         [2*(self.qi*self.qj - self.qk*self.qr), 1 - 2*self.qi**2 - 2*self.qk**2,       2*(self.qj*self.qk + self.qi*self.qr)],
                         [2*(self.qi*self.qk + self.qj*self.qr), 2*(self.qj*self.qk - self.qi*self.qr), 1 - 2*self.qi**2 - 2*self.qj**2      ]])

    def norm(self):
        '''
        norm of the quaternion
        '''
        return np.sqrt(self.qr**2 + self.qi**2 + self.qj**2 + self.qk**2)

    def normalize(self):
        '''
        normalize a quaternion

        any orientation quaternion must always be normalized
        this function exists to help ensure that
        '''
        norm = self.norm()
        self.qr /= norm
        self.qi /= norm
        self.qj /= norm
        self.qk /= norm

    def to_vec(self):
        ''' convert to a vector '''
        return np.array([self.qi, self.qj, self.qk, self.qr])

    def from_vec(self, vec):
        ''' unpack from a vector '''
        self.qi, self.qj, self.qk, self.qr = vec

    def from_yaw(self, yaw):
        ''' quaternion from yaw (on a flat euclidean surface)'''
        self.qi = 0
        self.qj = 0
        self.qk = np.sin(yaw/2)
        self.qr = np.cos(yaw/2)

    def to_yaw(self):
        ''' quaternion to yaw (on a flat euclidean surface)'''
        return 2*np.arctan2(self.qk, self.qr)

    def qdot(self,w: BodyAngularVelocity) -> 'OrientationQuaternion':
        ''' derivative from body frame angular velocity '''
        qdot = OrientationQuaternion()
        qdot.qr = -0.5 * (self.qi * w.w1 + self.qj*w.w2 + self.qk*w.w3)
        qdot.qi =  0.5 * (self.qr * w.w1 + self.qj*w.w3 - self.qk*w.w2)
        qdot.qj =  0.5 * (self.qr * w.w2 + self.qk*w.w1 - self.qi*w.w3)
        qdot.qk =  0.5 * (self.qr * w.w3 + self.qi*w.w2 - self.qj*w.w1)
        return qdot

@dataclass
class ParametricPose(PythonMsg):
    ''' parametric pose '''
    s: float = field(default = 0.)
    y: float = field(default = 0.)
    n: float = field(default = 0.)
    ths: float = field(default = 0.)

@dataclass
class PoseVel(NestedPythonMsg):
    ''' euclidean pose and velocity '''
    x: Position = field(default = None)
    q: OrientationQuaternion = field(default = None)
    v: BodyLinearVelocity = field(default = None)
    w: BodyAngularVelocity = field(default = None)
