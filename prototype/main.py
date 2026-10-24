"""Run with python main.py. Tkinter is the only graphical dependency."""
import math
import random
import time
import tkinter as tk
from engine import Plane, Sphere, Vec3, World


class Demo:
    def __init__(self, root):
        self.root = root
        root.title('Rigid3D | Sphere Physics Sandbox')
        self.canvas = tk.Canvas(root, width=1100, height=760, bg='#101827', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.yaw, self.pitch, self.zoom = 0.65, 0.48, 1.0
        self.paused = False
        self.drag = None
        self.reset()
        root.bind('<space>', lambda e: self.pause())
        root.bind('<KeyPress-r>', lambda e: self.reset())
        root.bind('<KeyPress-n>', lambda e: self.spawn())
        root.bind('<KeyPress-Right>', lambda e: self.single_step())
        root.bind('<Escape>', lambda e: root.destroy())
        self.canvas.bind('<ButtonPress-1>', self.begin_drag)
        self.canvas.bind('<B1-Motion>', self.orbit)
        self.canvas.bind('<MouseWheel>', lambda e: self.scale(1.1 if e.delta > 0 else 1 / 1.1))
        self.canvas.bind('<Button-4>', lambda e: self.scale(1.1))
        self.canvas.bind('<Button-5>', lambda e: self.scale(1 / 1.1))
        self.last = time.perf_counter()
        self.accumulator = 0.0
        self.tick()

    def reset(self):
        self.rng = random.Random(19)
        self.world = World()
        self.world.planes = [Plane(), Plane(Vec3(1, 0, 0), -5), Plane(Vec3(-1, 0, 0), -5),
                             Plane(Vec3(0, 0, 1), -5), Plane(Vec3(0, 0, -1), -5)]
        for i in range(18):
            self.world.bodies.append(Sphere(Vec3((i % 3 - 1) * 1.2, 1 + (i // 9) * 1.6 + (i % 3) * 0.25,
                                                ((i // 3) % 3 - 1) * 1.2),
                                            velocity=Vec3(self.rng.uniform(-1, 1), 0, self.rng.uniform(-1, 1)),
                                            color=self.rng.choice(['#f4aa67', '#74d3cb', '#b5a0ec', '#f07e8c'])))
        self.accumulator = 0.0

    def spawn(self):
        if len(self.world.bodies) < 80:
            self.world.bodies.append(Sphere(Vec3(self.rng.uniform(-3, 3), 7, self.rng.uniform(-3, 3)),
                                            velocity=Vec3(1, 0, 0), color='#74d3cb'))

    def pause(self):
        self.paused = not self.paused
        self.accumulator = 0.0

    def single_step(self):
        if self.paused:
            self.world.step()

    def begin_drag(self, event):
        self.drag = (event.x, event.y)

    def orbit(self, event):
        if self.drag:
            x, y = self.drag
            self.yaw += (event.x - x) * 0.008
            self.pitch = max(0.15, min(1.25, self.pitch + (event.y - y) * 0.006))
            self.drag = (event.x, event.y)

    def scale(self, factor):
        self.zoom = max(0.4, min(2, self.zoom * factor))

    def camera(self, v):
        v = v - Vec3(0, 2, 0)
        x = math.cos(self.yaw) * v.x - math.sin(self.yaw) * v.z
        z = math.sin(self.yaw) * v.x + math.cos(self.yaw) * v.z
        return Vec3(x, math.cos(self.pitch) * v.y - math.sin(self.pitch) * z,
                    math.sin(self.pitch) * v.y + math.cos(self.pitch) * z)

    def project(self, v):
        v = self.camera(v)
        scale = 650 * self.zoom / max(2, 20 - v.z)
        return (self.canvas.winfo_width() / 2 + v.x * scale,
                self.canvas.winfo_height() * 0.53 - v.y * scale, scale)

    def line(self, a, b, color='#29394c', width=1):
        ax, ay, _ = self.project(a)
        bx, by, _ = self.project(b)
        self.canvas.create_line(ax, ay, bx, by, fill=color, width=width)

    def draw(self):
        c = self.canvas
        c.delete('all')
        for i in range(-5, 6):
            self.line(Vec3(i, 0, -5), Vec3(i, 0, 5))
            self.line(Vec3(-5, 0, i), Vec3(5, 0, i))
        for a, b in [(Vec3(-5, 0, -5), Vec3(5, 0, -5)), (Vec3(5, 0, -5), Vec3(5, 0, 5)),
                     (Vec3(5, 0, 5), Vec3(-5, 0, 5)), (Vec3(-5, 0, 5), Vec3(-5, 0, -5))]:
            self.line(a, b, '#64758a', 2)
        for b in sorted(self.world.bodies, key=lambda body: self.camera(body.position).z):
            x, y, scale = self.project(b.position)
            r = b.radius * scale
            c.create_oval(x-r, y-r, x+r, y+r, fill=b.color, outline='#e7edf5', width=1)
            # A projected diameter makes angular motion visible. It is a visual marker.
            axis = b.rotate(Vec3(1, 0, 0)) * (b.radius * 0.85)
            self.line(b.position - axis, b.position + axis, '#253146', 2)
        c.create_text(28, 27, text='RIGID3D / PHYSICS SANDBOX', fill='#edf3fc', font=('Arial', 19, 'bold'), anchor='nw')
        c.create_text(28, 60, text='3D spheres  /  gravity  /  friction  /  angular impulses', fill='#a9b8cc', font=('Arial', 11), anchor='nw')
        status = 'PAUSED' if self.paused else 'RUNNING'
        c.create_text(28, 89, text=f'{status}    {len(self.world.bodies)} bodies    {len(self.world.contacts)} contacts    120 Hz physics',
                      fill='#74d3cb', font=('Arial', 11), anchor='nw')
        c.create_text(28, c.winfo_height()-30, anchor='w', fill='#a9b8cc', font=('Arial', 10),
                      text='Space: pause  |  N: drop sphere  |  R: reset  |  Right arrow: paused step  |  Drag: orbit  |  Wheel: zoom  |  Esc: exit')

    def tick(self):
        now = time.perf_counter()
        elapsed = min(now - self.last, 0.1)
        self.last = now
        if not self.paused:
            self.accumulator += elapsed
            while self.accumulator >= 1 / 120:
                self.world.step()
                self.accumulator -= 1 / 120
        self.draw()
        self.root.after(16, self.tick)


if __name__ == '__main__':
    try:
        root = tk.Tk()
    except tk.TclError as error:
        raise SystemExit('The viewer needs a desktop display and Tkinter. Run python -m unittest -v for headless physics checks.') from error
    Demo(root)
    root.mainloop()
