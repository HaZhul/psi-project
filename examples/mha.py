from psi_environment.data.car import Car
from psi_environment.data.action import Action
from psi_environment.data.map_state import MapState, TrafficLight
from psi_environment.environment import Environment
from psi_environment.api.environment_api import EnvironmentAPI
from psi_environment.data.point import PositionType

from collections import defaultdict, deque
from sortedcontainers import SortedList
import numpy as np
import time


class MyCar(Car):
    def __init__(self, road_key: tuple[int, int], road_pos: int, car_id: int):
        super().__init__(road_key, road_pos, car_id)
        self.algneed = True
        self.path = []
        self.actions = []
        self.points = 0
        self.traffic_var = 1.5
        self.lights_var = 2

    def get_action(self, map_state: MapState) -> Action:
        my_road_key = self.get_road_key()
        my_road_pos = self.get_road_pos()
        self.api = EnvironmentAPI(map_state)
        self.traffic_obj = TrafficLight(None)

        my_road = self.api.get_road(my_road_key)
        
        # ****
        if self.algneed:
            self.cost_matrix = self.api.get_adjacency_matrix()
            self.create_adjList()
            print(self.adjList)
            # print('Point: ', self.points)
            self.create_avaliable_set(self.points)
            # self.points += 1
            self.bfs(my_road_key)
            start = time.time()
            self.mha_star(my_road_key)
            print(f'Time: {time.time()-start}')
            print(f'\nPath:\n{self.path}, actions: {self.actions}')
        # ****

        if my_road.is_position_road_end(my_road_pos):
            i = self.path.index(my_road_key)
            try:
                current_action = self.actions[i]
            except:
                self.algneed = True
                return Action.BACK
            return Action(current_action)
        else:
            return Action.FORWARD
        


    def create_avaliable_set(self, point_number):
        """
        Returns a set of avaliable combinations of getting to the specific point.
        Roads can be one-way.
        """
        self.av_set = set()
        road_obj = self.api.get_points_for_specific_car(1)[point_number]
        if road_obj.type == PositionType.ROAD:
            point_data = self.api.get_points_for_specific_car(1)[point_number].road_positions
            for av_methods in point_data:
                road, _ = av_methods
                self.av_set.add(road)
        elif road_obj.type == PositionType.NODE:
            node = self.api.get_points_for_specific_car(1)[point_number].node
            self.av_set = [key for key in self.adjList.keys() if node == key[1]]
            
        self.av_set = list(self.av_set)


    def create_adjList(self):
        """
        A dict where keys are connections between 2 nodes and values are every possible neighbour of this connection.
        Values are: [Neighbour key (e.g. (0,1)); Action needed to reach it (action.py); Length to neighbour]
        """
        self.adjList = defaultdict(list)
        # print(self.cost_matrix[0:8, 0:8])
        for i in range(0, len(self.cost_matrix)):
            for j in range(0, len(self.cost_matrix)):
                if i != j:
                    for k in range(1,5):
                        next_road = self.api.get_next_road((i,j), k)
                        if next_road is not None:
                            next_road_key = next_road.get_key()
                            _, next_value = next_road_key
                            road_cost = self.cost_matrix[j,next_value]
                            self.adjList[(i,j)].append([next_road_key, k, road_cost])
        self.algneed = False

    
    def bfs(self, startNode):
        queue = deque([(startNode, [startNode], [])])
        point_list = self.av_set

        while queue:
            currentNode, path, action = queue.popleft()
            currentNode = currentNode

            if currentNode in point_list:
                # print(currentNode, ' is in ', point_list)
                # print('Start Node: ', startNode)
                # print(action)
                self.path = path
                self.actions = action
                break
                
            for neighbour in self.adjList[currentNode]:
                if neighbour[0] not in path:
                    queue.append((neighbour[0], path + [neighbour[0]], action + [neighbour[1]]))
    
    def manhattan_heuristic(self, point):
        manhattan_dist = []
        for possible_access in self.av_set:
            manhattan_dist.append(abs(point[0] - possible_access[0]) + abs(point[1] - possible_access[1]))
        return min(manhattan_dist)
    

    def traffic_heuristic(self, point):
        traffic_len = self.api.get_road_traffic(point)
        traffic_dist = []
        for possible_access in self.av_set:
            traffic_dist.append(abs(point[0] - possible_access[0]) + abs(point[1] - possible_access[1]) + traffic_len * self.traffic_var)
        return min(traffic_dist)

    def lights_heuristic(self, point):
        lights_dist = []
        traffic_list = self.traffic_obj.get_blocked_road_keys()
        if point in traffic_list:
            for possible_access in self.av_set:
                lights_dist.append(abs(point[0] - possible_access[0]) + abs(point[1] - possible_access[1]))
                return min(lights_dist) * self.lights_var
        else: 
            for possible_access in self.av_set:
                lights_dist.append(abs(point[0] - possible_access[0]) + abs(point[1] - possible_access[1]))
                return min(lights_dist)
            

    def mha_star(self, startNode):
            queue = SortedList()
            queue.add((0, 0, [startNode], [], startNode))

            while queue:
                # print(queue)
                totalCost, currentDist, path, action, currentNode = queue.pop(0)
                print(f'\nTotal cost: {totalCost}, path: {path}\n')
                if currentNode in self.av_set:
                    print('\n\n\nPath: ', path)
                    return path

                for neighbour in self.adjList[currentNode]:
                    if neighbour[0] not in path:
                        actual_cost = neighbour[2]
                        g_n = currentDist + actual_cost
                        h_n = self.manhattan_heuristic(neighbour[0])
                        f_cost = g_n + h_n
                        queue.add((f_cost, g_n, path + [neighbour[0]], action + [neighbour[1]], neighbour[0]))
        
            raise Exception('No path avaliable')


if __name__ == "__main__":
    env = Environment(
        agent_type=MyCar,
        ticks_per_second=3,
        n_bots=50,
        n_points=10,
        traffic_lights_length=10,
        random_seed=2137,
    )
    while env.is_running():
        current_cost, is_running = env.step()
        print(current_cost, is_running)
