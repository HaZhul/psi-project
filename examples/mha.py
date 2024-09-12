from psi_environment.data.car import Car
from psi_environment.data.action import Action
from psi_environment.data.map_state import MapState, TrafficLight
from psi_environment.environment import Environment
from psi_environment.api.environment_api import EnvironmentAPI
from psi_environment.data.point import PositionType

from collections import defaultdict
from sortedcontainers import SortedList


class MyCar(Car):
    def __init__(self, road_key: tuple[int, int], road_pos: int, car_id: int):
        super().__init__(road_key, road_pos, car_id)

        # self.algneed1 = True
        self.algneed2 = True
        self.position = 0
        self.key = 0
        self.path = [[],[],[]]
        self.actions = [[],[],[]]
        self.distList = [[],[],[]]
        self.dist = [[],[],[]]
        self.points = 0
        self.points_collected = 0
        self.chosen_heuristic_number = 0

        self.traffic_var = 10
        self.lights_var = 20
        self.stay_var = 1

    def get_action(self, map_state: MapState) -> Action:
        my_road_key = self.get_road_key()
        my_road_pos = self.get_road_pos()
        self.api = EnvironmentAPI(map_state)
        self.traffic_obj = TrafficLight(None)
        my_road = self.api.get_road(my_road_key)
        car_position_point = map_state.get_map_position_by_road_position(my_road_key, my_road_pos)

        # One-time
        if self.algneed2:
            self.cost_matrix = self.api.get_adjacency_matrix()
            self.create_adjList()
            self.points_collected = len(self.api.get_points_for_specific_car(1))

            # Going to the lowest numbered point
            # self.create_avaliable_set(self.points)

            # Going to the closest point
            self.get_closest_star(car_position_point)

            self.mha_star_manhattan(my_road_key)
            self.algneed2 = False

        # After collecting a point
        if len(self.api.get_points_for_specific_car(1)) != self.points_collected:
            self.points_collected = len(self.api.get_points_for_specific_car(1))
            # Going to the lowest numbered point
            # self.create_avaliable_set(self.points)

            # Going to the closest point
            self.get_closest_star(car_position_point)

            self.mha_star_manhattan(my_road_key)
            self.chosen_heuristic_number = 0
        

        # Every position change
        if self.position != my_road_pos:
            self.positions = my_road_pos
            self.mha_star_mh(my_road_key)
        else:
            self.dist[0] += 1 * self.stay_var

        # Every road change
        if self.key != my_road_key:
            self.key = my_road_key
            self.dist[self.chosen_heuristic_number] -= self.distList[self.chosen_heuristic_number].pop(0)
        

        
        self.check_other_path()
        

        if my_road.is_position_road_end(my_road_pos):
            try:
                i = self.path[self.chosen_heuristic_number].index(my_road_key)
                current_action = self.actions[self.chosen_heuristic_number][i]
            except:
                return Action.BACK
            return Action(current_action)
        else:
            return Action.FORWARD
        

    def check_other_path(self):
        for i in range(3):
            if i == self.chosen_heuristic_number:
                continue

            if self.dist[i] < 0.7 * self.dist[self.chosen_heuristic_number] and self.dist[i] != 0:
                self.chosen_heuristic_number = i

    def create_avaliable_set(self, point_number):
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

    def get_closest_star(self, car_point):
        stars_dict = defaultdict(list)
        for i in range(len(self.api.get_points_for_specific_car(1))):
            road_obj = self.api.get_points_for_specific_car(1)[i]
            if road_obj.type == PositionType.ROAD:
                point_data = self.api.get_points_for_specific_car(1)[i].map_position
            elif road_obj.type == PositionType.NODE:
                point_data = self.api.get_points_for_specific_car(1)[i].map_position
            stars_dict[i].append(self.l1_dist(car_point, point_data))
        
        min_dist = min(((key, min(value_list)) for key, value_list in stars_dict.items() if value_list), key=lambda x: x[1])[0]
        self.create_avaliable_set(min_dist)

    
    def l1_dist(self, point1, point2):
        return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])
        


    def create_adjList(self):
        self.adjList = defaultdict(list)
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

    
    def manhattan_heuristic(self, point):
        manhattan_dist = []
        for possible_access in self.av_set:
            manhattan_dist.append((abs(point[0] - possible_access[0]) + abs(point[1] - possible_access[1])))
        return min(manhattan_dist)
    

    def traffic_heuristic(self, point):
        traffic_len = self.api.get_road_traffic(point)
        road_length = self.api.get_road_length(point)
        traffic_dist = []
        for possible_access in self.av_set:
            traffic_dist.append((abs(point[0] - possible_access[0]) + abs(point[1] - possible_access[1])) 
                                + traffic_len/road_length * (3*self.traffic_var))
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
                lights_dist.append(abs(point[0] - possible_access[0]) + abs(point[1] - possible_access[1]) + self.lights_var/5)
            return min(lights_dist)
            

    def mha_star_manhattan(self, startNode):
            
            queue = SortedList()
            queue.add((0, 0, [startNode], [], startNode, [0]))

            while queue:
                totalCost, currentDist, path, action, currentNode, distList = queue.pop(0)
                if currentNode in self.av_set:
                    self.actions[0] = action
                    self.path[0] = path
                    self.dist[0] = currentDist
                    self.distList[0] = distList
                    break

                for neighbour in self.adjList[currentNode]:
                    if neighbour[0] not in path:
                        actual_cost = neighbour[2]
                        g_n = currentDist + actual_cost
                        h_n = self.manhattan_heuristic(neighbour[0])
                        f_cost = g_n + h_n
                        queue.add((f_cost, g_n, path + [neighbour[0]], action + [neighbour[1]], neighbour[0], distList + [actual_cost]))
        

    def mha_star_mh(self, startNode):
        for i in [1, 2]:
            queue = SortedList()
            queue.add((0, 0, [startNode], [], startNode, [0]))

            while queue:
                totalCost, currentDist, path, action, currentNode, distList = queue.pop(0)
                if currentNode in self.av_set:
                    self.actions[i]= action
                    self.path[i] = path
                    self.dist[i] = totalCost
                    self.distList[i] = distList
                    break

                for neighbour in self.adjList[currentNode]:
                    if neighbour[0] not in path:
                        actual_cost = neighbour[2]
                        g_n = currentDist + actual_cost
                        h_n = self.heuristic_func(i, neighbour[0])
                        f_cost = g_n + h_n
                        queue.add((f_cost, g_n, path + [neighbour[0]], action + [neighbour[1]], neighbour[0], distList + [actual_cost]))
        
    def heuristic_func(self, i, N):
        if i == 1:
            return self.traffic_heuristic(N)
        elif i == 2:
            return self.lights_heuristic(N)
        else:
            raise Exception('Out of range')

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
