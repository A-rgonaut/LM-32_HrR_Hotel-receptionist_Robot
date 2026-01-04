from math import sqrt
# from utils import

class Agent():
    def __init__(self, start, goal, mappa):
        self.start = start
        self.goal  = goal
        self.mappa = mappa

    def next_states(self, state):
        return self.mappa.get(state, [])

class AgentBFS(Agent):
    def __init__(self, start, goal, mappa):
        super().__init__(start, goal, mappa)
        self.frontier = [[start]]

    def sort_frontier(self):
        pass

    def bfs(self):
        while self.frontier:
            path = self.frontier.pop(0)
            state = path[-1]
            if state == self.goal:
                return path
            for next_state in self.next_states(state):
                if next_state not in path:
                    self.frontier.append(path + [next_state])
            self.sort_frontier()

class AgentGreedy(AgentBFS):
    def __init__(self, start, goal, mappa):
        super().__init__(start, goal, mappa)

    def heuristic(self, path):
        return sqrt((path[-1].x - self.goal.x) ** 2 +
                    (path[-1].y - self.goal.y) ** 2)

    def sort_frontier(self):
        self.frontier.sort(key=lambda path: self.heuristic(path))

class AgentAStar(AgentGreedy):
    def __init__(self, start, goal, mappa):
        super().__init__(start, goal, mappa)

    def heuristic(self, path):
        return super().heuristic(path) + len(path)

class Pianifica():
    def pianifica(self, start, goal, mappa):
        path = AgentAStar(start, goal, mappa).bfs()
        for luogo in path:
            print(luogo)

"""
start = l1
goal = l3
mappa = {
    l1: [l2],
    l2: [l3]
}
"""
