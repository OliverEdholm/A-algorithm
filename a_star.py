'''
A* search algorithm on 2d grid

This is a program done for educational purposes that lets you have a 2d grid
with set out obstacles, start and a goal. Then the A* search algorithm will
find the best possible path to the path to the goal.

Oliver Edholm 2016-10-16 22:01
'''
# imports
from math import sqrt


# objects
class AStarResult(object):
    def __init__(self, solveable=True, best_path=None):
        self.solveable = solveable
        self.best_path = best_path
        self.solved = self.best_path is not None

    def __str__(self):
        return 'solveable={}, solved={}'.format(self.solveable, self.solved)


class Node(object):
    def __init__(self, coordinate, is_obstacle, is_endpoint,
                 movement_cost=None, heuristic=None, parent=None):
        # values
        self.coordinate = coordinate
        self.parent = parent
        self.movement_cost = movement_cost
        self.H = heuristic
        self.G = None

        # attributes
        self.is_obstacle = is_obstacle
        self.is_endpoint = is_endpoint

        self.G = self._get_g_cost()
        self.F = self._get_f_cost()

    def _get_g_cost(self, parent=None, movement_cost=None):
        if parent is None:
            parent = self.parent
        if movement_cost is None:
            movement_cost = self.movement_cost

        if movement_cost:
            if parent:
                if parent.G:
                    return movement_cost + parent.G
        elif self.is_endpoint:
            return movement_cost

    def _get_f_cost(self):
        if self.H and self.G:
            return self.G + self.H
        else:
            return None

    def _calculate_heuristic(self, node, end_node):
        def manhattan_distance(coordinate1, coordinate2):
            return sum((abs(dimension1 - dimension2)
                        for dimension1, dimension2 in zip(coordinate1,
                                                          coordinate2)))

        return manhattan_distance(node.coordinate, end_node.coordinate)

    def update(self, parent, distance_to_parent, end_node, in_open_list=False):
        new_movement_cost = round(sqrt(distance_to_parent), 1) * 10
        new_g = self._get_g_cost(parent=parent,
                                 movement_cost=new_movement_cost)

        if new_g < self.G or not in_open_list:
            self.movement_cost = new_movement_cost
            self.parent = parent

            self.H = self._calculate_heuristic(self, end_node)

            self.G = new_g
            self.F = self._get_f_cost()


class Grid(object):
    def __init__(self, grid_layout):
        self.grid_shape = (len(grid_layout[0]), len(grid_layout))

        self.endpoint_nodes = []
        self.nodes = []

        self._interpret_grid_layout(grid_layout)

    def _interpret_grid_layout(self, grid_layout):
        # interpreting *grid_layout*
        for y, row in enumerate(reversed(grid_layout)):  # y goes down up
            node_layer = []
            for x, tile in enumerate(row):
                coordinate = (x, y)
                node_args = [coordinate, False, False]  # args for normal node

                if tile == 1:  # obstacle
                    node_args[1] = True
                elif tile == 2:  # endpoint
                    node_args[2] = True
                elif tile != 0:  # unknown
                    raise ValueError('Tile {} is in unknown format.'.format(
                        tile))

                node = Node(*node_args)
                node_layer.append(node)

                if node_args[2]:
                    self.endpoint_nodes.append(node)
            self.nodes.append(node_layer)

        assert len(self.endpoint_nodes) == 2, 'You can only have two endpoints.'

    def get_node(self, coordinate):
        # returns None if both if statements aren't satisfied
        if coordinate[0] < self.grid_shape[0] and coordinate[0] >= 0:
            if coordinate[1] < self.grid_shape[1] and coordinate[1] >= 0:
                return self.nodes[coordinate[1]][coordinate[0]]


# classes
class AStarSolver:
    def __init__(self, grid, starting_endpoint=0, allow_diagonals=True):
        assert starting_endpoint <= 2, 'There are only two endpoints, 0 means \
                                         endpoint 1, 1 means endpoint 2.'
        self.grid = grid
        self.allow_diagonals = allow_diagonals

        self.start_node = self.grid.endpoint_nodes[starting_endpoint]
        self.end_node = self.grid.endpoint_nodes[int(
                                                 not bool(starting_endpoint))]

        self.open_list = [self.start_node]
        self.closed_list = []

    def _get_and_update_node_children(self, node):
        # get coordinates of *node*s children.
        node_children_coordinates = []

        node_x, node_y = node.coordinate

        # straights are coordinates reached straight from *node*
        straights = [(node_x - 1, node_y), (node_x + 1, node_y),
                     (node_x, node_y - 1), (node_x, node_y + 1)]
        # turns into tuples that store coordinates and the distance from *node*
        straights = [(coordinate, 1) for coordinate in straights]
        node_children_coordinates.extend(straights)

        if self.allow_diagonals:
            # diagonals are coordinates reached diagonally from *node*
            diagonals = [(node_x - 1, node_y + 1), (node_x - 1, node_y - 1),
                         (node_x + 1, node_y + 1), (node_x + 1, node_y - 1)]
            # turns into tuples that store coordinates and the distance from *node*
            diagonals = [(coordinate, 2) for coordinate in diagonals]
            node_children_coordinates.extend(diagonals)

        # get and update *node*s children
        node_children = []
        for coordinate, distance_to_node in node_children_coordinates:
            child_node = self.grid.get_node(coordinate)
            if child_node is None:
                continue
            elif child_node in self.closed_list:
                continue

            if not child_node.is_obstacle:
                child_node.update(node, distance_to_node, self.end_node,
                                  in_open_list=child_node in self.open_list)
                node_children.append(child_node)

        return node_children

    def _get_best_open_node(self):
        if not self.open_list:
            return None

        self.open_list = sorted(self.open_list, key=lambda node: node.F,
                                reverse=True)

        return self.open_list[0]

    def _get_best_path(self):
        path_coordinates = []

        current_node = self.end_node
        while True:
            if current_node.coordinate not in path_coordinates:
                path_coordinates.append(current_node.coordinate)

            if current_node == self.start_node:
                break

            current_node = current_node.parent

        return list(reversed(path_coordinates))

    def update(self):
        node = self._get_best_open_node()
        self.open_list = self.open_list[1:]
        self.closed_list.append(node)

        if node is None:
            return AStarResult(solveable=False)

        node_children = self._get_and_update_node_children(node)

        solved = False
        for child_node in node_children:
            if child_node == self.end_node:
                solved = True

            if child_node not in self.open_list:
                self.open_list.append(child_node)

        if solved:
            return AStarResult(best_path=self._get_best_path())

        return AStarResult()

    def solve(self):
        while True:
            result = self.update()
            if result.solved:
                return result
            elif not result.solveable:
                return result


# functions
def main():
    # 0 means walkable tile, 1 means obstacle, 2 means endpoint
    grid_layout = [[0, 0, 0, 0, 0, 0, 2, 0],
                   [0, 0, 0, 1, 1, 0, 0, 0],
                   [0, 0, 0, 0, 1, 1, 0, 0],
                   [0, 2, 0, 0, 0, 0, 0, 0]]
    grid = Grid(grid_layout)

    algorithm = AStarSolver(grid)
    print(algorithm.solve().best_path)


if __name__ == '__main__':  # if user is running this program
    main()
