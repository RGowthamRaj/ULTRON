"""Educational 3D sphere rigid bodies and static infinite planes. SI units."""
from dataclasses import dataclass, field
from math import cos, exp, isfinite, sin, sqrt


@dataclass(frozen=True)
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, v):
        return Vec3(self.x + v.x, self.y + v.y, self.z + v.z)

    def __neg__(self):
        return self * -1

    def __sub__(self, v):
        return self + -v

    def __mul__(self, k):
        return Vec3(self.x * k, self.y * k, self.z * k)

    __rmul__ = __mul__

    def dot(self, v):
        return self.x * v.x + self.y * v.y + self.z * v.z

    def cross(self, v):
        return Vec3(self.y * v.z - self.z * v.y,
                    self.z * v.x - self.x * v.z,
                    self.x * v.y - self.y * v.x)

    def length(self):
        return sqrt(self.dot(self))

    def unit(self):
        n = self.length()
        if n < 1e-12:
            raise ValueError('Cannot normalize a zero vector')
        return self * (1 / n)


@dataclass
class Sphere:
    position: Vec3
    radius: float = 0.5
    mass: float = 1.0  # Zero mass means static.
    velocity: Vec3 = field(default_factory=Vec3)
    omega: Vec3 = field(default_factory=Vec3)
    restitution: float = 0.45
    friction: float = 0.6
    orientation: tuple = (1.0, 0.0, 0.0, 0.0)  # w, x, y, z
    force: Vec3 = field(default_factory=Vec3)
    torque: Vec3 = field(default_factory=Vec3)
    color: str = '#e8a55c'

    def __post_init__(self):
        if not isfinite(self.radius) or self.radius <= 0:
            raise ValueError('Radius must be finite and positive')
        if not isfinite(self.mass) or self.mass < 0:
            raise ValueError('Mass must be finite and nonnegative')
        if not 0 <= self.restitution <= 1 or not isfinite(self.friction) or self.friction < 0:
            raise ValueError('Invalid material coefficients')
        norm = sqrt(sum(v * v for v in self.orientation))
        if len(self.orientation) != 4 or not isfinite(norm) or norm < 1e-12:
            raise ValueError('Orientation must be a nonzero finite quaternion')
        self.orientation = tuple(v / norm for v in self.orientation)

    @property
    def inv_mass(self):
        return 1 / self.mass if self.mass else 0.0

    @property
    def inv_inertia(self):
        return 2.5 * self.inv_mass / self.radius ** 2

    def apply_force(self, force, point=None):
        self.force = self.force + force
        if point is not None:
            self.torque = self.torque + (point - self.position).cross(force)

    def impulse(self, impulse, arm=Vec3()):
        if self.mass:
            self.velocity = self.velocity + impulse * self.inv_mass
            self.omega = self.omega + arm.cross(impulse) * self.inv_inertia

    def rotate(self, v):
        w, x, y, z = self.orientation
        q = Vec3(x, y, z)
        t = q.cross(v) * 2
        return v + t * w + q.cross(t)

    def integrate_rotation(self, dt):
        speed = self.omega.length()
        if speed < 1e-12:
            return
        half = speed * dt / 2
        a = self.omega * (sin(half) / speed)
        w, x, y, z = self.orientation
        v = Vec3(x, y, z)
        xyz = v * cos(half) + a * w + a.cross(v)
        q = (cos(half) * w - a.dot(v), xyz.x, xyz.y, xyz.z)
        n = sqrt(sum(t * t for t in q))
        self.orientation = tuple(t / n for t in q)


@dataclass(frozen=True)
class Plane:
    """Allowed half-space: normal.dot(position) >= offset."""
    normal: Vec3 = Vec3(0, 1, 0)
    offset: float = 0.0
    restitution: float = 0.45
    friction: float = 0.6

    def __post_init__(self):
        if abs(self.normal.length() - 1) > 1e-6:
            raise ValueError('Plane normal must have unit length')
        if not isfinite(self.offset) or not 0 <= self.restitution <= 1:
            raise ValueError('Invalid plane offset or restitution')
        if not isfinite(self.friction) or self.friction < 0:
            raise ValueError('Invalid plane friction')


class Contact:
    def __init__(self, a, b, normal, depth, material):
        self.a, self.b, self.n, self.depth = a, b, normal, depth
        self.ra = normal * a.radius
        self.rb = -normal * b.radius if b else Vec3()
        self.friction = sqrt(a.friction * material.friction)
        closing = self.relative().dot(normal)
        self.target = -min(a.restitution, material.restitution) * closing if closing < -1 else 0
        self.normal_impulse = 0.0
        self.tangent_impulse = Vec3()

    def relative(self):
        va = self.a.velocity + self.a.omega.cross(self.ra) if self.a.mass else Vec3()
        vb = self.b.velocity + self.b.omega.cross(self.rb) if self.b and self.b.mass else Vec3()
        return vb - va

    def effective_mass(self, axis):
        k = self.a.inv_mass + (self.b.inv_mass if self.b else 0)
        k += self.a.inv_inertia * self.ra.cross(axis).dot(self.ra.cross(axis))
        if self.b:
            k += self.b.inv_inertia * self.rb.cross(axis).dot(self.rb.cross(axis))
        return k

    def apply(self, impulse):
        self.a.impulse(-impulse, self.ra)
        if self.b:
            self.b.impulse(impulse, self.rb)

    def solve(self):
        k = self.effective_mass(self.n)
        old = self.normal_impulse
        self.normal_impulse = max(0, old + (self.target - self.relative().dot(self.n)) / k)
        self.apply(self.n * (self.normal_impulse - old))
        rv = self.relative()
        tangent = rv - self.n * rv.dot(self.n)
        if tangent.length() > 1e-10:
            axis = tangent.unit()
            candidate = self.tangent_impulse - tangent * (1 / self.effective_mass(axis))
            limit = self.friction * self.normal_impulse
            if candidate.length() > limit:
                candidate = candidate.unit() * limit
            self.apply(candidate - self.tangent_impulse)
            self.tangent_impulse = candidate


class World:
    def __init__(self, gravity=Vec3(0, -9.81, 0), iterations=16, damping=0.02):
        if iterations < 1 or damping < 0:
            raise ValueError('Invalid solver settings')
        self.gravity, self.iterations, self.damping = gravity, iterations, damping
        self.bodies, self.planes, self.contacts = [], [], []

    def detect(self):
        contacts = []
        for i, a in enumerate(self.bodies):
            for p in self.planes:
                depth = a.radius - (p.normal.dot(a.position) - p.offset)
                if a.mass and depth > 0:
                    contacts.append(Contact(a, None, -p.normal, depth, p))
            for b in self.bodies[i + 1:]:
                if not (a.mass or b.mass):
                    continue
                delta = b.position - a.position
                distance = delta.length()
                depth = a.radius + b.radius - distance
                if depth > 0:
                    n = delta * (1 / distance) if distance > 1e-10 else Vec3(1, 0, 0)
                    contacts.append(Contact(a, b, n, depth, b))
        return contacts

    def step(self, dt=1 / 120):
        if not isfinite(dt) or not 0 < dt <= 1 / 60:
            raise ValueError('Use a fixed timestep in (0, 1/60] seconds')
        for b in self.bodies:
            if b.mass:
                b.velocity = (b.velocity + (self.gravity + b.force * b.inv_mass) * dt) * exp(-self.damping * dt)
                b.omega = (b.omega + b.torque * (b.inv_inertia * dt)) * exp(-self.damping * dt)
                b.position = b.position + b.velocity * dt
                b.integrate_rotation(dt)
            b.force, b.torque = Vec3(), Vec3()
        self.contacts = self.detect()
        for _ in range(self.iterations):
            for c in self.contacts:
                c.solve()
        # Recompute geometry on each positional pass; keep orientation/velocity independent.
        for _ in range(4):
            for c in self.detect():
                total = c.a.inv_mass + (c.b.inv_mass if c.b else 0)
                correction = c.n * (0.7 * max(c.depth - 0.001, 0) / total)
                c.a.position = c.a.position - correction * c.a.inv_mass
                if c.b:
                    c.b.position = c.b.position + correction * c.b.inv_mass
