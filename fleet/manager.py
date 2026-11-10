"""Mixed-fleet manager: one mission, three different bodies.

Robots are abstract here (position, speed, what they can do); fleet/run.py moves the real models.
The manager owns a shared occupancy grid, a task queue and the assignment rule, and replans when the
world changes (blocked aisle, dropped box, robot fault).
"""

import heapq
import math
from dataclasses import dataclass, field

CELL = 0.5  # m per grid cell


class GridMap:
    """Shared 2D map. Cells are free or blocked; robots plan on it with A*."""

    def __init__(self, x0, y0, x1, y1):
        self.x0, self.y0 = x0, y0
        self.nx, self.ny = int((x1 - x0) / CELL), int((y1 - y0) / CELL)
        self.blocked = set()

    def cell(self, p):
        return (int((p[0] - self.x0) / CELL), int((p[1] - self.y0) / CELL))

    def center(self, c):
        return (self.x0 + (c[0] + 0.5) * CELL, self.y0 + (c[1] + 0.5) * CELL)

    def block_rect(self, x0, y0, x1, y1, inflate=0.35):
        a, b = self.cell((x0 - inflate, y0 - inflate)), self.cell((x1 + inflate, y1 + inflate))
        for i in range(a[0], b[0] + 1):
            for j in range(a[1], b[1] + 1):
                self.blocked.add((i, j))

    def nearest_free(self, p):
        """Closest free cell center to p (a dropped box can land next to an obstacle)."""
        c = self.cell(p)
        for rad in range(0, 8):
            ring = [(c[0] + i, c[1] + j) for i in range(-rad, rad + 1) for j in range(-rad, rad + 1)
                    if max(abs(i), abs(j)) == rad and self.free((c[0] + i, c[1] + j))]
            if ring:
                return self.center(min(ring, key=lambda q: math.dist(self.center(q), p)))
        return tuple(p)

    def free(self, c):
        return 0 <= c[0] < self.nx and 0 <= c[1] < self.ny and c not in self.blocked

    def plan(self, start, goal):
        """A* on 8-connected cells. Returns a list of (x, y) waypoints or None."""
        s, g = self.cell(start), self.cell(goal)
        self.blocked.discard(s)
        if not self.free(g):
            return None
        h = lambda c: math.dist(c, g)  # noqa: E731
        openq, came, cost = [(h(s), s)], {s: None}, {s: 0.0}
        while openq:
            _, c = heapq.heappop(openq)
            if c == g:
                path = []
                while c is not None:
                    path.append(self.center(c))
                    c = came[c]
                path.reverse()
                path[-1] = tuple(goal)
                return path
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (c[0] + dx, c[1] + dy)
                    if (dx or dy) and self.free(n):
                        nc = cost[c] + math.hypot(dx, dy)
                        if nc < cost.get(n, 1e9):
                            cost[n], came[n] = nc, c
                            heapq.heappush(openq, (nc + h(n), n))
        return None


@dataclass
class Task:
    id: int
    kind: str                 # "unload" | "carry" | "patrol" | "recover"
    src: tuple | None = None
    dst: tuple | None = None
    box: int | None = None
    owner: str | None = None
    done: bool = False


@dataclass
class Robot:
    name: str
    body: str                 # "atlas" | "spot" | "stretch"
    pos: list
    speed: float
    skills: set
    path: list = field(default_factory=list)
    task: Task | None = None
    stage: str = "idle"       # what it is doing now, shown on the panel
    carrying: int | None = None
    fault: bool = False
    heading: float = 0.0
    goal: tuple | None = None


class FleetManager:
    def __init__(self, grid, robots, staging, line, patrol):
        self.grid, self.robots = grid, {r.name: r for r in robots}
        self.staging, self.line, self.patrol = staging, line, patrol
        self.tasks, self.log, self.next_id = [], [], 0
        self.delivered, self.staged = [], []
        self.patrol_i = 0

    # --- events from the world (sensors, robots) ---------------------------------------
    def say(self, t, msg):
        self.log.append((t, msg))

    def add_task(self, kind, **kw):
        self.next_id += 1
        task = Task(self.next_id, kind, **kw)
        self.tasks.append(task)
        return task

    def box_staged(self, t, box):
        self.staged.append(box)
        self.add_task("carry", src=self.staging, dst=self.line, box=box)
        self.say(t, f"box {box} on staging -> carry task")

    def obstacle_seen(self, t, rect, by):
        self.grid.block_rect(*rect)
        self.say(t, f"{by} sees aisle blocked -> map updated, replanning")
        for r in self.robots.values():
            if r.path and any(self.grid.cell(p) in self.grid.blocked for p in r.path):
                self.route(r, r.path[-1])

    def box_dropped(self, t, robot, box, where):
        r = self.robots[robot]
        r.carrying, r.task.done = None, True
        self.say(t, f"{robot} dropped box {box} -> recover task")
        self.add_task("recover", src=self.grid.nearest_free(where), dst=self.line, box=box)

    def fault(self, t, robot):
        r = self.robots[robot]
        if r.carrying is not None:  # it goes down with the box: the box needs recovering
            self.box_dropped(t, robot, r.carrying, list(r.pos))
            r.task = None
        r.fault, r.path, r.stage = True, [], "FAULT"
        if r.task and not r.task.done:
            self.say(t, f"{robot} fault -> task {r.task.id} back to the queue")
            r.task.owner, r.task = None, None
        else:
            self.say(t, f"{robot} fault")

    def recover(self, t, robot):
        self.robots[robot].fault = False
        self.robots[robot].stage = "idle"
        self.say(t, f"{robot} back online")

    # --- planning ------------------------------------------------------------------------
    def route(self, r, goal):
        r.goal = tuple(goal)
        path = self.grid.plan(r.pos, goal)
        r.path = path[1:] if path else []
        return path is not None

    def assign(self, t):
        for task in sorted(self.tasks, key=lambda k: k.kind != "recover"):  # a box on the floor is a hazard: first
            if task.done or task.owner or task.kind in ("unload", "patrol"):
                continue
            free = [r for r in self.robots.values()
                    if task.kind in r.skills and r.task is None and not r.fault]
            if not free:
                continue
            r = min(free, key=lambda r: math.dist(r.pos, task.src))
            task.owner, r.task, r.stage = r.name, task, f"to pick box {task.box}"
            self.route(r, task.src)
            self.say(t, f"task {task.id} ({task.kind} box {task.box}) -> {r.name}")

    def tick(self, t, dt):
        self.assign(t)
        for r in self.robots.values():
            if r.fault:
                continue
            if r.body == "spot" and r.task is None and not r.path:
                self.patrol_i = (self.patrol_i + 1) % len(self.patrol)
                r.stage = "patrolling"
                self.route(r, self.patrol[self.patrol_i])
            self.move(r, dt)
            if r.task and not r.path:
                if r.goal and math.dist(r.pos, r.goal) < 0.15:
                    self.arrived(t, r)
                elif r.goal and not self.route(r, r.goal) and r.stage != "no path, waiting":
                    r.stage = "no path, waiting"
                    self.say(t, f"{r.name}: no path to its goal, waiting")

    def move(self, r, dt):
        step = r.speed * dt
        while r.path and step > 0:
            tx, ty = r.path[0]
            dx, dy = tx - r.pos[0], ty - r.pos[1]
            d = math.hypot(dx, dy)
            if d > 1e-6:
                r.heading = math.atan2(dy, dx)
            if d <= step:
                r.pos[:] = [tx, ty]
                r.path.pop(0)
                step -= d
            else:
                r.pos[0] += dx / d * step
                r.pos[1] += dy / d * step
                step = 0

    def arrived(self, t, r):
        task = r.task
        if r.carrying is None:  # at the pickup
            r.carrying = task.box
            if task.box in self.staged:
                self.staged.remove(task.box)
            r.stage = f"carrying box {task.box}"
            self.route(r, task.dst)
            self.say(t, f"{r.name} picked box {task.box}")
        else:                   # at the line
            self.delivered.append(task.box)
            self.say(t, f"{r.name} delivered box {task.box} to the line")
            r.carrying, task.done, r.task, r.stage = None, True, None, "idle"


if __name__ == "__main__":
    # Self-check without any rendering: three boxes, one blocked aisle, one drop, one fault.
    g = GridMap(-8, -5, 8, 5)
    g.block_rect(-0.5, -1.2, 4.5, -1.0)  # shelving row south of the aisle
    g.block_rect(-0.5, 1.0, 4.5, 1.2)    # shelving row north of the aisle
    atlas = Robot("Atlas", "atlas", [-1.5, 3.0], 1.0, {"carry", "recover"})
    spot = Robot("Spot", "spot", [5.0, -3.0], 1.6, {"inspect"})
    fm = FleetManager(g, [atlas, spot], staging=(-2.0, 2.5), line=(6.0, 0.0),
                      patrol=[(5.0, -3.0), (5.0, 3.0), (0.0, 0.0), (-1.0, -3.0)])
    t, dt = 0.0, 0.1
    events = {5.0: lambda: fm.box_staged(t, 1), 20.0: lambda: fm.box_staged(t, 2),
              12.0: lambda: fm.obstacle_seen(t, (1.5, -0.8, 2.5, 0.8), "Spot"),
              40.0: lambda: fm.box_staged(t, 3), 45.0: lambda: fm.fault(t, "Atlas"),
              50.0: lambda: fm.recover(t, "Atlas")}
    while t < 120:
        for et in [e for e in events if e <= t]:
            events.pop(et)()
        fm.tick(t, dt)
        t = round(t + dt, 3)
    for when, msg in fm.log:
        print(f"{when:6.1f}s  {msg}")
    assert fm.delivered == [1, 2, 3], fm.delivered
    print("ok: all boxes delivered after a blocked aisle and a fault")
