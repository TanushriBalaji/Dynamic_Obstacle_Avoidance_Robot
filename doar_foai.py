
import heapq
import random

random.seed(7)

DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def get_int(prompt, minimum=0):
    while True:
        try:
            value = int(input(prompt))
            if value < minimum:
                print("Enter a value of at least", minimum)
                continue
            return value
        except ValueError:
            print("Please enter an integer.")


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(start, goal, walls, blocked, rows, cols):
    queue = [(heuristic(start, goal), 0, start)]
    parent = {}
    cost = {start: 0}
    visited = set()

    while queue:
        _, _, current = heapq.heappop(queue)

        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            path = [current]
            while current in parent:
                current = parent[current]
                path.append(current)
            return path[::-1]

        for dr, dc in DIRECTIONS:
            nxt = (current[0] + dr, current[1] + dc)
            r, c = nxt

            if not (0 <= r < rows and 0 <= c < cols):
                continue
            if nxt in walls or nxt in blocked:
                continue

            new_cost = cost[current] + 1

            if new_cost < cost.get(nxt, float("inf")):
                cost[nxt] = new_cost
                parent[nxt] = current
                f = new_cost + heuristic(nxt, goal)
                heapq.heappush(queue, (f, new_cost, nxt))

    return None


def display(rows, cols, robot, goal, walls, known, path, visible):
    path = set(path or [])

    print("\n    " + " ".join(f"{c:2}" for c in range(cols)))

    for r in range(rows):
        line = []

        for c in range(cols):
            cell = (r, c)

            if cell == robot:
                symbol = "R"
            elif cell == goal:
                symbol = "G"
            elif cell in walls:
                symbol = "#"
            elif cell in known:
                symbol = "M"
            elif cell in path:
                symbol = "*"
            elif cell in visible:
                symbol = "."
            else:
                symbol = "?"

            line.append(f"{symbol:2}")

        print(f"{r:2}  " + " ".join(line))


def main():
    print("=" * 48)
    print("DYNAMIC OBSTACLE AVOIDANCE ROBOT")
    print("A* + Partial Observability + Dynamic Replanning")
    print("=" * 48)

    rows = get_int("Enter number of rows (minimum 2): ", 2)
    cols = get_int("Enter number of columns (minimum 2): ", 2)

    start = (
        get_int("Enter robot start row: "),
        get_int("Enter robot start column: ")
    )
    goal = (
        get_int("Enter target row: "),
        get_int("Enter target column: ")
    )

    def valid(cell):
        return 0 <= cell[0] < rows and 0 <= cell[1] < cols

    if not valid(start) or not valid(goal):
        print("Error: start and goal must be inside the grid.")
        return

    if start == goal:
        print("Start and goal are identical. Robot is already there.")
        return

    walls = set()
    moving = []

    wall_count = get_int("Enter number of static obstacles: ")
    for i in range(wall_count):
        cell = (
            get_int(f"Static obstacle {i+1} row: "),
            get_int(f"Static obstacle {i+1} column: ")
        )
        if not valid(cell) or cell in (start, goal):
            print("Invalid obstacle position; try again.")
            return
        walls.add(cell)

    moving_count = get_int("Enter number of moving obstacles: ")
    for i in range(moving_count):
        cell = (
            get_int(f"Moving obstacle {i+1} row: "),
            get_int(f"Moving obstacle {i+1} column: ")
        )
        if not valid(cell) or cell in walls or cell in (start, goal):
            print("Invalid moving obstacle position; try again.")
            return
        if cell in moving:
            print("Moving obstacles must have different positions.")
            return
        moving.append(cell)

    sensor_range = get_int("Enter sensor range (minimum 1): ", 1)
    max_steps = get_int("Enter maximum robot steps: ", 1)

    robot = start
    known = set()

    print("\nLegend: R=Robot G=Goal #=Wall M=Known moving obstacle")
    print("*=Planned path .=Observed free cell ?=Unknown")

    for step in range(max_steps + 1):
        # Move obstacles before sensing, except at initial step.
        # Obstacles cannot move into the robot, walls, or each other.
        if step > 0:
            for i, pos in enumerate(moving):
                occupied = set(moving)
                occupied.discard(pos)
                occupied.add(robot)

                options = []
                for dr, dc in DIRECTIONS:
                    nxt = (pos[0] + dr, pos[1] + dc)
                    if (0 <= nxt[0] < rows and
                        0 <= nxt[1] < cols and
                        nxt not in walls and
                        nxt not in occupied):
                        options.append(nxt)

                if options:
                    moving[i] = random.choice(options)

        # Simulated local sensor
        visible = {
            (r, c)
            for r in range(rows)
            for c in range(cols)
            if heuristic(robot, (r, c)) <= sensor_range
        }

        # Update only cells inside the current sensor range
        known.difference_update(visible)
        known.update(pos for pos in moving if pos in visible)

        path = astar(robot, goal, walls, known, rows, cols)

        print(f"\nSTEP {step} | Robot: {robot} | Goal: {goal}")
        print("Detected obstacles:", sorted(known))

        if path is None:
            print("No path on the known map. Robot waits and senses again.")
            display(rows, cols, robot, goal, walls, known, [], visible)
            continue

        print("Planned path:", path)
        print("Path cost:", len(path) - 1)
        display(rows, cols, robot, goal, walls, known, path, visible)

        if robot == goal:
            print("\nSUCCESS: Target reached!")
            print("Robot movements:", step)
            return

        next_cell = path[1]

        # Safety check against the simulated true environment.
        # This is simulator ground truth, not a physical sensor.
        if next_cell in moving or next_cell in walls:
            print("Obstacle blocks next movement. Robot waits.")
            continue

        robot = next_cell
        print("Robot moves to:", robot)

    if robot == goal:
        print("\nSUCCESS: Target reached!")
    else:
        print("\nSTOP: Maximum step limit reached.")
        print("Final robot position:", robot)


if __name__ == "__main__":
    main()
