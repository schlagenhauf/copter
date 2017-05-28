import math

class Vector():
  def __init__(self, x=0, y=0, z=0):
    self.x = x
    self.y = y
    self.z = z

  @classmethod
  def fromList(cls, valueList):
    if len(valueList) is not 3:
      raise ValueError('Initialization of vector with more or less than 3 values')
    return cls(valueList[0], valueList[1], valueList[2])

  def __add__(self, rightOp):
    return Vector(self.x + rightOp.x, self.y + rightOp.y, self.z + rightOp.z)

  def __sub__(self, rightOp):
    return Vector(self.x - rightOp.x, self.y - rightOp.y, self.z - rightOp.z)

  def __mul__(self, rightOp):
    return Vector(self.x * rightOp, self.y * rightOp, self.z * rightOp)

  def __div__(self, rightOp):
    return self.__mul__(1/rightOp)

  def __iadd__(self, rightOp):
    self.x += rightOp.x
    self.y += rightOp.y
    self.z += rightOp.z
    return self

  def __isub__(self, rightOp):
    self.x -= rightOp.x
    self.y -= rightOp.y
    self.z -= rightOp.z
    return self

  def __imul__(self, rightOp):
    self.x *= rightOp
    self.y *= rightOp
    self.z *= rightOp
    return self

  def __str__(self):
    return '{X: ' + str(self.x) + ' Y: ' + str(self.y) + ' Z: ' + str(self.z) + '}'

  def norm(self):
    return math.sqrt(pow(self.x,2) + pow(self.y,2) + pow(self.z,2))

  def norm2(self):
    return pow(self.x,2) + pow(self.y,2) + pow(self.z,2)

  def normalize(self):
    norm = self.norm()
    self.x /= norm
    self.y /= norm
    self.z /= norm

  def cross(self, op):
    x = self.y*op.z - self.z*op.y
    y = self.z*op.x - self.x*op.z
    z = self.x*op.y - self.y*op.x
    return Vector(x,y,z)

  def dot(self, op):
    return self.x*op.x + self.y*op.y + self.z*op.z
