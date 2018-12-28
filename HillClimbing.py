import heapq


""""to implement HillClimbingOriginal.py first we need a priority queue"""
class PriorityQueue:

    def __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

    def update(self, item, priority):
        for index, (p, c, i) in enumerate(self.heap):
            """If item is in priority queue with higher priority, update its priority"""
            if i == item:
                """If item is in priority queue with equal or less priority, do nothing"""
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
            """If item isn't in priority queue, push"""
        else:
            self.push(item, priority)

"""The main function: HillClimbingOriginal.py"""
def hillClimbing(problem):

    fringe = PriorityQueue()

    def pushToFringe(fringe, state, cost):
        fringe.push(state, cost)

    (current, c_cost, c_path) = (problem.getStartState(), 0, [])

    if current.isGoal():
        return c_path

    def getNeighbor(node, cost, path):
        bestH = 0
        for child_node, child_action, child_cost in problem.getSuccessors(node):
            if child_node.heuristic() > bestH:
                new_cost = cost + child_cost
                new_path = path + [child_action]
                bestNode = (child_node, new_cost, new_path)
                bestH = child_node.heuristic()
        return bestNode

    while True:
        (neighbor, n_cost, n_path) = getNeighbor(current, c_cost, c_path);
        if neighbor.heuristic() <= current.heuristic():
            return c_path
        (current, c_cost, c_path) = (neighbor, n_cost, n_path)
