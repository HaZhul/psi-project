from psi_environment.data.car import Car
from psi_environment.data.action import Action
from psi_environment.data.map_state import MapState
from psi_environment.environment import Environment
from psi_environment.api.environment_api import EnvironmentAPI

from collections import defaultdict, deque
import numpy as np


class MyCar(Car):
    def __init__(self, road_key: tuple[int, int], road_pos: int, car_id: int):
        super().__init__(road_key, road_pos, car_id)
        self.algneed = True
        self.path = []
        self.actions = []
        self.points = 1

    def get_action(self, map_state: MapState) -> Action:
        my_road_key = self.get_road_key()
        my_road_pos = self.get_road_pos()
        self.api = EnvironmentAPI(map_state)

        my_road = self.api.get_road(my_road_key)
        
        # ****
        if self.algneed:
            self.cost_matrix = self.api.get_adjacency_matrix()
            self.create_adjList()
            # print('Point: ', self.points)
            self.create_avaliable_set(self.points)
            # self.points += 1
            self.bfs(my_road_key)
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
        # print(self.api.get_points_for_specific_car(1))
        point_data = self.api.get_points_for_specific_car(1)[point_number].road_positions
        # print(point_data)
        for av_methods in point_data:
            road, _ = av_methods
            self.av_set.add(road)


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
        point_list = list(self.av_set)

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


if __name__ == "__main__":
    env = Environment(
        agent_type=MyCar,
        ticks_per_second=10,
        n_bots=50,
        n_points=10,
        traffic_lights_length=10,
        random_seed=2137,
    )
    while env.is_running():
        current_cost, is_running = env.step()
        print(current_cost, is_running)
