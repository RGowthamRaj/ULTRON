import math
import unittest
from engine import Plane, Sphere, Vec3, World


class PhysicsTests(unittest.TestCase):
    def test_elastic_collision_conserves_momentum_and_energy(self):
        w = World(gravity=Vec3(), damping=0)
        a = Sphere(Vec3(-0.49, 0, 0), velocity=Vec3(2, 0, 0), restitution=1, friction=0)
        b = Sphere(Vec3(0.49, 0, 0), mass=2, velocity=Vec3(-1, 0, 0), restitution=1, friction=0)
        w.bodies = [a, b]
        w.step()
        self.assertAlmostEqual(a.velocity.x + 2 * b.velocity.x, 0)
        self.assertAlmostEqual(0.5 * a.velocity.dot(a.velocity) + b.velocity.dot(b.velocity), 3)
        self.assertAlmostEqual(a.velocity.x, -2)
        self.assertAlmostEqual(b.velocity.x, 1)

    def test_floor_support(self):
        w = World()
        b = Sphere(Vec3(0, 4, 0), restitution=0)
        w.bodies = [b]
        w.planes = [Plane(restitution=0)]
        for _ in range(1200):
            w.step()
        self.assertAlmostEqual(b.position.y, b.radius, delta=0.002)
        self.assertAlmostEqual(b.velocity.y, 0, delta=0.001)

    def test_friction_creates_spin_and_reduces_slip(self):
        w = World(damping=0)
        b = Sphere(Vec3(0, 0.49, 0), velocity=Vec3(4, 0, 0), restitution=0)
        w.bodies = [b]
        w.planes = [Plane(restitution=0)]
        for _ in range(100):
            w.step()
        self.assertLess(b.velocity.x, 4)
        self.assertLess(b.omega.z, 0)
        slip = b.velocity + b.omega.cross(Vec3(0, -b.radius, 0))
        self.assertAlmostEqual(slip.x, 0, delta=0.02)

    def test_quaternion_rotation_and_normalization(self):
        b = Sphere(Vec3(), omega=Vec3(0, math.pi, 0))
        b.integrate_rotation(0.5)
        axis = b.rotate(Vec3(1, 0, 0))
        self.assertAlmostEqual(axis.x, 0)
        self.assertAlmostEqual(axis.z, -1)
        self.assertAlmostEqual(sum(x*x for x in b.orientation), 1)

    def test_force_and_torque(self):
        w = World(gravity=Vec3(), damping=0)
        b = Sphere(Vec3(), mass=2)
        b.apply_force(Vec3(0, 120, 0), Vec3(1, 0, 0))
        w.bodies = [b]
        w.step()
        self.assertAlmostEqual(b.velocity.y, 0.5)
        self.assertAlmostEqual(b.omega.z, 5)
        self.assertEqual(b.force, Vec3())
        self.assertEqual(b.torque, Vec3())

    def test_static_body_and_plane_offset(self):
        w = World()
        static = Sphere(Vec3(3, 2, 0), mass=0, velocity=Vec3(10, 0, 0))
        dynamic = Sphere(Vec3(0, 1.45, 0), restitution=0)
        w.bodies = [static, dynamic]
        w.planes = [Plane(offset=1, restitution=0)]
        w.step()
        self.assertEqual(static.position, Vec3(3, 2, 0))
        self.assertGreater(dynamic.position.y, 1.498)

    def test_coincident_centers_remain_finite(self):
        w = World(gravity=Vec3())
        w.bodies = [Sphere(Vec3()), Sphere(Vec3())]
        w.step()
        self.assertGreater((w.bodies[1].position - w.bodies[0].position).length(), 0.99)

    def test_reject_invalid_inputs(self):
        with self.assertRaises(ValueError):
            Sphere(Vec3(), radius=0)
        with self.assertRaises(ValueError):
            Plane(Vec3(0, 2, 0))
        with self.assertRaises(ValueError):
            World().step(1)


if __name__ == '__main__':
    unittest.main()
