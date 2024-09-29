from _helpers import Node, Stack, Queue, PriorityQueue
from math import sqrt


class DFS_Algorithm:
    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim
        self.stack = Stack()
        self.stack.push(Node(pos=start_pos, parent=None))
        self.next_nodes = []

    def get_successors(self, x, y):
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def backtrack_solution(self, curr_node):
        return self._backtrack(curr_node)

    def _backtrack(self, curr_node):
        return [] if curr_node.parent is None else self._backtrack(curr_node.parent) + [curr_node.position()]

    def update(self, grid):
        curr_state = self.stack.pop()
        x, y = curr_state.position()
        done = False
        solution_path = []

        for step in self.get_successors(x, y):
            if self.is_valid_cell(step) and grid[step[0], step[1]] in [1,
                                                                       3] and step not in self.next_nodes:  # 1: empty cell has not explored yet, 3: goal cell
                self.next_nodes.append(step)
                self.stack.push(Node(pos=step, parent=curr_state))

                if step == self.goal_pos:
                    done = True
                    solution_path = self.backtrack_solution(curr_state)
                    break

        grid[x, y] = 4  # visited

        return solution_path, done, grid


class BFS_Algorithm:

    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim
        self.queue = Queue()
        self.queue.push(Node(pos=start_pos, parent=None))
        self.next_nodes = []

    def get_successors(self, x, y):
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def backtrack_solution(self, curr_node):
        return self._backtrack(curr_node)

    def _backtrack(self, curr_node):
        return [] if curr_node.parent is None else self._backtrack(curr_node.parent) + [curr_node.position()]

    def update(self, grid):
        curr_state = self.queue.pop()
        x, y = curr_state.position()
        done = False
        solution_path = []

        for step in self.get_successors(x, y):
            if self.is_valid_cell(step) and grid[step[0], step[1]] in [1,
                                                                       3] and step not in self.next_nodes:  # 1: empty cell has not explored yet, 3: goal cell

                self.next_nodes.append(step)
                self.queue.push(Node(pos=step, parent=curr_state))

                if step == self.goal_pos:
                    done = True
                    solution_path = self.backtrack_solution(curr_state)
                    break

        grid[x, y] = 4  # visited

        return solution_path, done, grid


class IDS_Algorithm:
    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim
        self.stack = Stack()
        self.stack.push(Node(pos=start_pos, parent=None))
        self.limit = 0
        self.next_nodes = []

    def get_successors(self, x, y):
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def backtrack_solution(self, curr_node):
        return self._backtrack(curr_node)

    def _backtrack(self, curr_node):
        return [] if curr_node.parent is None else self._backtrack(curr_node.parent) + [curr_node.position()]

    def reset_grid(self, grid):
        for i in range(self.grid_dim[0] + 1):
            for j in range(self.grid_dim[1] + 1):
                if grid[i][j] == 4:
                    grid[i][j] = 1

    def update(self, grid):
        curr_state = self.stack.pop()
        x, y = curr_state.position()
        done = False
        solution_path = []

        depth = len(self.backtrack_solution(curr_state))
        if depth <= self.limit:
            for step in self.get_successors(x, y):
                if self.is_valid_cell(step) and grid[step[0], step[1]] in [1,
                                                                           3] and step not in self.next_nodes:  # 1: empty cell has not explored yet, 3: goal cell
                    self.next_nodes.append(step)
                    self.stack.push(Node(pos=step, parent=curr_state))

                    if step == self.goal_pos:
                        done = True
                        solution_path = self.backtrack_solution(curr_state)
                        break

        grid[x, y] = 4  # visited

        if not done and self.stack.isEmpty():
            self.limit += 1
            self.reset_grid(grid)
            self.stack.push(Node(pos=self.start_pos, parent=None))
            self.next_nodes = []
        return solution_path, done, grid


class A_Star_Algorithm:
    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim
        self.heap = PriorityQueue()
        self.heap.push(Node(pos=start_pos, parent=None, cost=self.get_cost(None, start_pos)), priority=0)
        self.next_nodes = []

    def get_successors(self, x, y):
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def backtrack_solution(self, curr_node):
        return self._backtrack(curr_node)

    def _backtrack(self, curr_node):
        return [] if curr_node.parent is None else self._backtrack(curr_node.parent) + [curr_node.position()]

    def get_heuristic(self, pos):
        return abs(pos[0] - self.goal_pos[0]) + abs(pos[1] - self.goal_pos[1])

    def get_cost(self, parent, pos):
        if parent is not None:
            parent_heuristic = self.get_heuristic(parent.position())
            child_heuristic = self.get_heuristic(pos)
            cost = parent.cost - parent_heuristic + 1 + child_heuristic
            return cost
        else:
            return self.get_heuristic(pos)

    def update(self, grid):
        curr_state = self.heap.pop()
        x, y = curr_state.position()

        done = False
        solution_path = []

        for step in self.get_successors(x, y):
            if self.is_valid_cell(step) and grid[step[0], step[1]] in [1,
                                                                       3] and step not in self.next_nodes:  # 1: empty cell has not explored yet, 3: goal cell

                self.next_nodes.append(step)
                cost = self.get_cost(curr_state, step)
                self.heap.push(Node(pos=step, parent=curr_state, cost=cost), priority=cost)

                if step == self.goal_pos:
                    done = True
                    solution_path = self.backtrack_solution(curr_state)
                    break

        grid[x, y] = 4  # visited

        return solution_path, done, grid


class A_Star_Geometric_Algorithm:
    def __init__(self, start_pos, goal_pos, grid_dim):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.grid_dim = grid_dim
        self.heap = PriorityQueue()
        self.heap.push(Node(pos=start_pos, parent=None, cost=self.get_cost(None, start_pos)), priority=0)
        self.next_nodes = []

    def get_successors(self, x, y):
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1), (x + 1, y + 1), (x - 1, y - 1), (x - 1, y + 1), (x + 1, y - 1)]

    def is_valid_cell(self, pos):
        return 0 <= pos[0] <= self.grid_dim[0] and 0 <= pos[1] <= self.grid_dim[1]

    def backtrack_solution(self, curr_node):
        return self._backtrack(curr_node)

    def _backtrack(self, curr_node):
        return [] if curr_node.parent is None else self._backtrack(curr_node.parent) + [curr_node.position()]

    def get_heuristic(self, pos):
        return sqrt((pos[0] - self.goal_pos[0])**2 + (pos[1] - self.goal_pos[1])**2)
    def get_cost(self, parent, pos):
        if parent is not None:
            parent_heuristic = self.get_heuristic(parent.position())
            child_heuristic = self.get_heuristic(pos)
            cost = parent.cost - parent_heuristic + 1 + child_heuristic
            return cost
        else:
            return self.get_heuristic(pos)

    def update(self, grid):
        curr_state = self.heap.pop()
        x, y = curr_state.position()

        done = False
        solution_path = []

        for step in self.get_successors(x, y):
            if self.is_valid_cell(step) and grid[step[0], step[1]] in [1,
                                                                       3] and step not in self.next_nodes:  # 1: empty cell has not explored yet, 3: goal cell

                self.next_nodes.append(step)
                cost = self.get_cost(curr_state, step)
                self.heap.push(Node(pos=step, parent=curr_state, cost=cost), priority=cost)

                if step == self.goal_pos:
                    done = True
                    solution_path = self.backtrack_solution(curr_state)
                    break

        grid[x, y] = 4  # visited

        return solution_path, done, grid
