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

    def get_action(self, map_state: MapState) -> Action:
        my_road_key = self.get_road_key()
        my_road_pos = self.get_road_pos()


        self.api = EnvironmentAPI(map_state)
        my_road = self.api.get_road((1,2))
        
        if self.algneed:
            self.cost_matrix = self.api.get_adjacency_matrix()
            # print(self.cost_matrix)
            self.create_adjList()
            print(self.adjList)
    
        if my_road.is_position_road_end(my_road_pos):
            available_actions = my_road.get_available_actions()
            min_traffic = 9999

            for action in available_actions:
                traffic = self.api.get_next_road(my_road_key, action).get_traffic()
                if traffic < min_traffic:
                    min_traffic = traffic
                    action = action
                
                if min_traffic == 0:
                    break

            return action

        return Action.FORWARD
        
    def create_adjList(self):
        """
        A dict where keys are connection between 2 nodes and values are every possible neighbour of this connection.
        Values are: [Neighbour key (e.g. (0,1)); Action needed to reach it (action.py); Length of neighbour]
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

    
    def bfs(self, startNode, point):
        queue = deque([(startNode, [startNode], [0, 0])])
        paths = []

        while queue:
            # 'Wyciagniecie' aktualnego miasta oraz sciezki z lewej strony queue
            # Tak wiec rozwijamy najpierw wszystkie sciezki po kolei
            currentNode, path, current_point = queue.popleft()
            #print(f"BFS: {path}")
            
            # Warunek sprawdzajacy czy zostala osiagnieta odpowiednia dlugosc
            if currentNode == point:
                paths.append(path)
                # print(path)
                continue
            
            for neighbour in self.adjList[currentNode]:
                if neighbour not in path:
                    horizontal_dist = 1
                    vertical_dist = 1
                    queue.append((neighbour, path + [neighbour], [current_point[0] + horizontal_dist, current_point[1] + vertical_dist]))
        return paths


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
