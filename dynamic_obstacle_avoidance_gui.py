import tkinter as tk
from tkinter import ttk, messagebox
import heapq, random

DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(start, goal, walls, blocked, rows, cols):
    queue = [(heuristic(start, goal), 0, start)]
    parent, cost, visited = {}, {start: 0}, set()
    while queue:
        _, _, cur = heapq.heappop(queue)
        if cur in visited:
            continue
        visited.add(cur)
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            return path[::-1]
        for dr, dc in DIRECTIONS:
            nxt = (cur[0]+dr, cur[1]+dc)
            r, c = nxt
            if not (0 <= r < rows and 0 <= c < cols):
                continue
            if nxt in walls or nxt in blocked:
                continue
            new_cost = cost[cur] + 1
            if new_cost < cost.get(nxt, float("inf")):
                cost[nxt] = new_cost
                parent[nxt] = cur
                heapq.heappush(queue, (new_cost+heuristic(nxt, goal), new_cost, nxt))
    return None

def parse_positions(text):
    result = []
    if not text.strip():
        return result
    for item in text.split(";"):
        p = item.strip().split(",")
        if len(p) != 2:
            raise ValueError("Enter obstacles as row,column; row,column")
        result.append((int(p[0]), int(p[1])))
    return result

class RobotApp:
    def __init__(self, root):
        self.root = root
        root.title("Dynamic Obstacle Avoidance Robot")
        root.geometry("1120x740")
        root.minsize(900, 620)
        root.configure(bg="#e9edf2")
        self.running = False
        self.after_id = None
        self.make_ui()
        self.reset_simulation()

    def make_ui(self):
        style = ttk.Style()
        for theme in ("vista", "clam"):
            try:
                style.theme_use(theme)
                break
            except tk.TclError:
                pass
        style.configure("TButton", font=("Segoe UI", 10), padding=6)
        style.configure("TLabel", font=("Segoe UI", 10))
        style.configure("TLabelframe.Label", font=("Segoe UI", 10, "bold"))

        top = tk.Frame(self.root, bg="#243b5a", height=62)
        top.pack(fill="x")
        top.pack_propagate(False)
        tk.Label(top, text="DYNAMIC OBSTACLE AVOIDANCE ROBOT",
                 bg="#243b5a", fg="white",
                 font=("Segoe UI Semibold", 17)).pack(side="left", padx=20, pady=14)
        tk.Label(top, text="A*  |  PARTIAL OBSERVABILITY  |  DYNAMIC REPLANNING",
                 bg="#243b5a", fg="#c5dcef",
                 font=("Segoe UI", 9)).pack(side="right", padx=18)

        body = tk.Frame(self.root, bg="#e9edf2")
        body.pack(fill="both", expand=True, padx=14, pady=12)

        left = ttk.LabelFrame(body, text="Simulation Settings", padding=12)
        left.pack(side="left", fill="y", padx=(0, 12))
        self.entries = {}
        fields = [
            ("Rows", "8"), ("Columns", "8"),
            ("Start (row,col)", "0,0"), ("Goal (row,col)", "7,7"),
            ("Static obstacles", "2,2; 2,3; 4,4; 5,4"),
            ("Moving obstacles", "1,5; 5,1"),
            ("Sensor range", "2"), ("Maximum steps", "100")
        ]
        for label, default in fields:
            ttk.Label(left, text=label).pack(anchor="w", pady=(5, 2))
            e = ttk.Entry(left, width=28)
            e.insert(0, default)
            e.pack(fill="x", pady=(0, 3))
            self.entries[label] = e

        ttk.Separator(left).pack(fill="x", pady=9)
        ttk.Button(left, text="▶  Start Simulation", command=self.start).pack(fill="x", pady=3)
        ttk.Button(left, text="Step Forward", command=self.step).pack(fill="x", pady=3)
        ttk.Button(left, text="Stop", command=self.stop).pack(fill="x", pady=3)
        ttk.Button(left, text="Reset", command=self.reset_simulation).pack(fill="x", pady=3)
        ttk.Label(left, text="Obstacle format:\nrow,column; row,column\n\nExample:\n2,3; 4,5\n\nThe robot senses nearby\ncells and replans its path.",
                  foreground="#536579", justify="left").pack(anchor="w", pady=14)

        right = tk.Frame(body, bg="#e9edf2")
        right.pack(side="left", fill="both", expand=True)
        gridbox = ttk.LabelFrame(right, text="Robot Navigation Grid", padding=8)
        gridbox.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(gridbox, bg="white", highlightthickness=1,
                                highlightbackground="#b7c4d1")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda e: self.draw())

        info = ttk.LabelFrame(right, text="Simulation Information", padding=8)
        info.pack(fill="x", pady=(9, 0))
        self.status = tk.StringVar(value="Ready")
        self.details = tk.StringVar(value="")
        ttk.Label(info, textvariable=self.status,
                  font=("Segoe UI Semibold", 10), foreground="#244b70").pack(anchor="w")
        ttk.Label(info, textvariable=self.details).pack(anchor="w", pady=(3, 5))
        legend = tk.Frame(info, bg="#f0f3f6")
        legend.pack(fill="x")
        for sym, name, color in [
            ("R", "Robot", "#1686c9"), ("G", "Goal", "#26945c"),
            ("#", "Wall", "#657487"), ("M", "Moving obstacle", "#d65b65"),
            ("*", "Path", "#3b82d8"), ("?", "Unknown", "#a5afb9")
        ]:
            tk.Label(legend, text=f" {sym} ", bg=color, fg="white",
                     font=("Segoe UI", 9, "bold")).pack(side="left", padx=(3, 1))
            tk.Label(legend, text=name, bg="#f0f3f6",
                     font=("Segoe UI", 9)).pack(side="left", padx=(0, 7))

        logbox = ttk.LabelFrame(right, text="Activity", padding=6)
        logbox.pack(fill="x", pady=(9, 0))
        self.log = tk.Text(logbox, height=5, wrap="word",
                           font=("Consolas", 9), bg="#fbfcfd", fg="#243447")
        self.log.pack(fill="x")
        self.log.configure(state="disabled")

    def log_line(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def read_settings(self):
        rows = int(self.entries["Rows"].get())
        cols = int(self.entries["Columns"].get())
        start = tuple(map(int, self.entries["Start (row,col)"].get().split(",")))
        goal = tuple(map(int, self.entries["Goal (row,col)"].get().split(",")))
        walls = set(parse_positions(self.entries["Static obstacles"].get()))
        moving = parse_positions(self.entries["Moving obstacles"].get())
        sensor = int(self.entries["Sensor range"].get())
        maximum = int(self.entries["Maximum steps"].get())
        if rows < 2 or cols < 2 or len(start) != 2 or len(goal) != 2:
            raise ValueError("Grid must be at least 2x2. Enter start and goal as row,column.")
        if not (0 <= start[0] < rows and 0 <= start[1] < cols):
            raise ValueError("Start position is outside the grid.")
        if not (0 <= goal[0] < rows and 0 <= goal[1] < cols):
            raise ValueError("Goal position is outside the grid.")
        if start == goal:
            raise ValueError("Start and goal must be different.")
        if sensor < 1 or maximum < 1:
            raise ValueError("Sensor range and maximum steps must be at least 1.")
        if len(moving) != len(set(moving)):
            raise ValueError("Moving obstacle positions must be unique.")
        if walls & set(moving):
            raise ValueError("Static and moving obstacles cannot overlap.")
        for p in walls.union(moving):
            if not (0 <= p[0] < rows and 0 <= p[1] < cols):
                raise ValueError(f"Obstacle {p} is outside the grid.")
            if p == start or p == goal:
                raise ValueError("An obstacle cannot occupy the start or goal.")
        return rows, cols, start, goal, walls, moving, sensor, maximum

    def reset_simulation(self):
        self.stop()
        try:
            (self.rows, self.cols, self.start_pos, self.goal,
             self.walls, self.moving, self.sensor_range, self.max_steps) = self.read_settings()
        except Exception as e:
            messagebox.showerror("Invalid Settings", str(e))
            return
        self.robot = self.start_pos
        self.step_count = 0
        self.known, self.visible, self.path = set(), set(), []
        self.finished = False
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")
        self.sense_and_plan()
        self.status.set("Ready. Press Start Simulation or Step Forward.")
        self.draw()
        self.update_details()
        self.log_line("Simulation initialized.")

    def sense_and_plan(self):
        self.visible = {
            (r, c) for r in range(self.rows) for c in range(self.cols)
            if heuristic(self.robot, (r, c)) <= self.sensor_range
        }
        self.known.difference_update(self.visible)
        self.known.update(p for p in self.moving if p in self.visible)
        self.path = astar(self.robot, self.goal, self.walls, self.known,
                          self.rows, self.cols)

    def move_obstacles(self):
        for i, pos in enumerate(self.moving):
            occupied = set(self.moving)
            occupied.discard(pos)
            occupied.add(self.robot)
            options = []
            for dr, dc in DIRECTIONS:
                nxt = (pos[0]+dr, pos[1]+dc)
                if (0 <= nxt[0] < self.rows and 0 <= nxt[1] < self.cols
                        and nxt not in self.walls and nxt not in occupied):
                    options.append(nxt)
            if options:
                self.moving[i] = random.choice(options)

    def step(self):
        if self.finished:
            return
        if self.step_count >= self.max_steps:
            self.finished = True
            self.status.set("Maximum step limit reached.")
            self.log_line("STOP: Maximum step limit reached.")
            return
        if self.step_count > 0:
            self.move_obstacles()
        self.sense_and_plan()
        self.log_line(f"STEP {self.step_count} | Robot: {self.robot} | Detected: {sorted(self.known)}")
        if self.robot == self.goal:
            self.finished = True
            self.status.set("SUCCESS: Target reached!")
            self.log_line("SUCCESS: Target reached!")
        elif self.path is None:
            self.status.set("No known path. Robot waits and senses again.")
            self.log_line("No path on known map. Robot waits.")
        else:
            self.log_line(f"Planned path: {self.path}")
            self.log_line(f"Path cost: {len(self.path)-1}")
            next_cell = self.path[1]
            # This safety check uses the simulated environment's true positions.
            if next_cell in self.moving or next_cell in self.walls:
                self.status.set("Next cell blocked. Robot waits and replans.")
                self.log_line("Obstacle blocks next movement. Robot waits.")
            else:
                self.robot = next_cell
                self.status.set("Robot moved. Recalculating route...")
                self.log_line(f"Robot moved to: {self.robot}")
        self.step_count += 1
        if self.robot == self.goal:
            self.finished = True
            self.status.set("SUCCESS: Target reached!")
            self.log_line("SUCCESS: Target reached!")
        elif self.step_count >= self.max_steps and not self.finished:
            self.finished = True
            self.status.set("Maximum step limit reached.")
        self.draw()
        self.update_details()

    def start(self):
        self.reset_simulation()
        if self.finished:
            return
        self.running = True
        self.status.set("Simulation running...")
        self.run_loop()

    def run_loop(self):
        if not self.running:
            return
        self.step()
        if self.finished:
            self.running = False
        else:
            self.after_id = self.root.after(650, self.run_loop)

    def stop(self):
        self.running = False
        if self.after_id is not None:
            try:
                self.root.after_cancel(self.after_id)
            except tk.TclError:
                pass
            self.after_id = None
        if hasattr(self, "status"):
            self.status.set("Simulation stopped / paused.")

    def update_details(self):
        self.details.set(
            f"Robot: {self.robot}     Goal: {self.goal}     "
            f"Step: {self.step_count} / {self.max_steps}"
        )

    def draw(self):
        if not hasattr(self, "canvas"):
            return
        self.canvas.delete("all")
        cw, ch = max(self.canvas.winfo_width(), 400), max(self.canvas.winfo_height(), 350)
        cell = min((cw-50)/self.cols, (ch-50)/self.rows, 58)
        cell = max(20, cell)
        ox, oy = (cw-self.cols*cell)/2, (ch-self.rows*cell)/2
        for r in range(self.rows):
            for c in range(self.cols):
                p = (r, c)
                x, y = ox+c*cell, oy+r*cell
                fill, symbol, color = "#f1f3f5", "?", "#a5afb9"
                if p in self.walls:
                    fill, symbol, color = "#687789", "#", "white"
                elif p in self.known:
                    fill, symbol, color = "#f6d8dc", "M", "#a52e3b"
                elif self.path and p in self.path:
                    fill, symbol, color = "#dcecff", "*", "#2c6db4"
                elif p in self.visible:
                    fill, symbol, color = "#edf5fb", "", "#536273"
                if p == self.goal:
                    fill, symbol, color = "#d9f1e3", "G", "#187342"
                if p == self.robot:
                    fill, symbol, color = "#d5edfb", "R", "#086da8"
                self.canvas.create_rectangle(x, y, x+cell, y+cell,
                                             fill=fill, outline="#b7c4d1")
                if symbol:
                    self.canvas.create_text(x+cell/2, y+cell/2, text=symbol,
                                            fill=color,
                                            font=("Segoe UI Semibold", max(10, int(cell*.30))))
        self.canvas.create_text(ox, max(12, oy-12), anchor="w",
                                text="GRID MAP  •  One cell = one robot position",
                                fill="#536579", font=("Segoe UI", 9, "bold"))

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotApp(root)
    root.mainloop()
