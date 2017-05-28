import math

class Quaternion:
  def __init__(self, w=1, x=0, y=0, z=0):
    self.w = w
    self.x = x
    self.y = y
    self.z = z


  @classmethod
  def fromEulerAngles(cls, alpha, beta, gamma):
    ca = math.cos(alpha/2)
    sa = math.sin(alpha/2)
    cb = math.cos(beta/2)
    sb = math.sin(beta/2)
    cg = math.cos(gamma/2)
    sg = math.sin(gamma/2)
    print "ca: " + str(ca) + " ,sa: " + str(sa) + " ,cb: " + str(cb) + " ,sb: " + str(sb) + " ,cg: " + str(cg) + " ,sg: " + str(sg)

    w = ca*cb*cg+sa*sb*sg
    x = sa*cb*cg-ca*sb*sg
    y = ca*sb*cg+sa*cb*sg
    z = ca*cb*sg-sa*sb*cg
    return cls(w,x,y,z)


  @classmethod
  def fromInstance(cls, inst):
    return cls(inst.w, inst.x, inst.y, inst.z)


  @classmethod
  def fromAngleAxis(cls, ang, axis):
    w = math.cos(ang/2)
    x = axis.x * math.sin(ang/2)
    y = axis.y * math.sin(ang/2)
    z = axis.z * math.sin(ang/2)
    qt = cls(w,x,y,z)
    return qt


  @classmethod
  def fromTwoVecs(cls, v1, v2):
    ang = math.sqrt(v1.norm2() * v2.norm2()) + v1.dot(v2)
    axis = v1.cross(v2)
    qt = cls(ang, axis.x, axis.y, axis.z)
    return qt


  def __add__(self, rOp):
    return Quaternion(self.w + rOp.w, self.x + rOp.x, self.y + rOp.y, self.z +
        rOp.z)


  def __sub__(self, rOp):
    return Quaternion(self.w - rOp.w, self.x - rOp.x, self.y - rOp.y, self.z -
        rOp.z)


  def __mul__(self, rOp):
    w = self.w * rOp.w - self.x * rOp.x - self.y * rOp.y - self.z * rOp.z
    x = self.w * rOp.x + self.x * rOp.w + self.y * rOp.z - self.z * rOp.y
    y = self.w * rOp.y - self.x * rOp.z + self.y * rOp.w + self.z * rOp.x
    z = self.w * rOp.z + self.x * rOp.y - self.y * rOp.x + self.z * rOp.w
    return Quaternion(w,x,y,z)


  def __str__(self):
    return '{W: ' + str(self.w) + ' X: ' + str(self.x) + ' Y: ' + str(self.y) + ' Z: ' + str(self.z) + '}'


  def scale(self, factor):
    self.w *= factor
    self.x *= factor
    self.y *= factor
    self.z *= factor


  def scaled(self, factor):
    qt = Quaternion.fromInstance(self)
    qt.scale(factor)
    return qt


  def conjugate(self):
    self.x = -self.x
    self.y = -self.y
    self.z = -self.z


  def normalize(self):
    n = self.norm()
    if n == 0:
        return
    self.w /= n
    self.x /= n
    self.y /= n
    self.z /= n


  def ang(self, op, normalized = True):
    if normalized:
      return math.acos(self.w*op.w + self.x*op.x + self.y*op.y + self.z*op.z)
    else:
      return math.acos((self.w*op.w + self.x*op.x + self.y*op.y + self.z*op.z) / (self.norm() * op.norm()))


  def norm(self):
    return math.sqrt(pow(self.x,2) + pow(self.y,2) + pow(self.z,2) + pow(self.w,2))


  def slerp(self, destQt, t):
    tmpQt = Quaternion.fromInstance(destQt)
    tmpQt.normalize()

    print 'tmp: ' + str(tmpQt)
    dotP = self.dot(tmpQt)

    if dotP < 0:
      tmpQt.conjugate()
      dotP = -dotP

    print 'self.w: ' + str(self.w) + ' new.w: ' + str(tmpQt.w)

    a = math.acos(self.w - tmpQt.w)
    qt = self.scaled(math.sin((1-t)*a)) + tmpQt.scaled(math.sin(t*a))
    qt.scale(1/math.sin(a))
    #qt.normalize()
    return qt


  def slerp2(self, destQt, t, eps = 0.01):
    if t <= 0:
      return self
    if t >= 1:
      return destQt

    tmpQt = Quaternion.fromInstance(destQt)
    tmpQt.normalize()
    self.normalize()
    angle = self.ang(tmpQt)

    if abs(angle) < eps:
      print 'LERP'
      return self.lerp(destQt, t)
    elif angle < 0:
      tmpQt.scale(-1)
      angle = -dotP
      print 'Negated!'

    print 'SLERP'

    return (self.scaled(math.sin((1-t)*angle)) + tmpQt.scaled(math.sin(t*angle))).scaled(1/math.sin(angle))


  def lerp(self, destQt, t):
    tmpQt = Quaternion.fromInstance(destQt)
    tmpQt.normalize()

    return self.scaled(1-t) + tmpQt.scaled(t)


  def toAngleAxis(self):
    #print self
    self.normalize()
    return (2*math.acos(self.w), 2*math.asin(self.x), 2*math.asin(self.y), 2*math.asin(self.z))


  def rotAngleInDeg(self):
    self.normalize()
    return 2*math.acos(self.w) * 180 / math.pi
