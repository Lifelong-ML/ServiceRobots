import sys
import Queue

class map_node:
    def __init__(self, name):
        self.name = name
        self.distance = sys.maxint
        self.parent = None


class map_search:
    def __init__(self):
        self.adjacency_list = {}
        self.node_map = {}

    def add_node(self, name):
        self.node_map[name] = map_node(name)
        self.adjacency_list[name] = []


    def load_maps(self):
        self.add_node("levine_1st_floor")
        self.add_node("levine_2nd_floor")
        self.add_node("moore_1st_floor")
        self.add_node("moore_2nd_floor")
        self.add_node("skirkanich_1st_floor")
        self.add_node("skirkanich_2nd_floor")
        self.add_node("towne_1st_floor")
        self.add_node("towne_2nd_floor")
        self.adjacency_list["levine_1st_floor"] = ["levine_2nd_floor", "moore_1st_floor", "towne_1st_floor"]
        self.adjacency_list["moore_1st_floor"] = ["moore_2nd_floor", "skirkanich_1st_floor", "levine_1st_floor"]
        self.adjacency_list["skirkanich_1st_floor"] = ["skirkanich_2nd_floor", "towne_1st_floor", "moore_1st_floor"]
        self.adjacency_list["towne_1st_floor"] = ["towne_2nd_floor", "levine_1st_floor", "skirkanich_1st_floor"]
        self.adjacency_list["levine_2nd_floor"] = ["levine_1st_floor"]
        self.adjacency_list["moore_2nd_floor"] = ["moore_1st_floor"]
        self.adjacency_list["skirkanich_2nd_floor"] = ["skirkanich_1st_floor"]
        self.adjacency_list["towne_2nd_floor"] = ["towne_1st_floor"]


    def find_path(self, start, goal):
        print "start: " + start
        print "goal: " + goal
        q = Queue.Queue()
        s = self.node_map[start]
        s.distance = 0
        q.put(s)
        while q.not_empty:
            curr = q.get()
            if curr.name is goal:
                return self.retrieve_path(curr)
            for i, name in enumerate(self.adjacency_list[curr.name]):
                neighbor = self.node_map[name]
                if neighbor.distance == sys.maxint:
                    neighbor.distance = curr.distance + 1
                    neighbor.parent = curr
                    q.put(neighbor)

    def retrieve_path(self, end_node):
        path = []
        curr = end_node
        path.append(curr)
        while curr.parent is not None:
            curr = curr.parent
            path.append(curr)
        path.reverse()
        return path

    def path_to_String(self, path):
        s = path[0].name
        for i in range(1, len(path)):
            s += ", " + path[i].name
        return s


if __name__ == "__main__":
    map_search = map_search()
    map_search.load_maps()
    path = map_search.find_path("levine_2nd_floor", "skirkanich_2nd_floor")
    print map_search.path_to_String(path)