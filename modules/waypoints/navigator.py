import pathlib
import networkx
from modules.waypoints.navigation_graph import NavigationGraph
import json
import cv2
from scipy import interpolate
import numpy as np

WINDOW_NAME = 'Navigation Graph'
DATA_FILE_PATH = pathlib.Path('data.json')
CURVE_COLOR = (255, 0, 100)


class Navigator:
    def __init__(self, file_name: str):
        with open(file_name, 'r') as file:
            data = json.load(file)
        self.nodes, edges, adjacency_matrix = data['nodes'], data['edges'], data['adjacency_matrix']
        self.graph = networkx.Graph()
        for n1, n2 in edges:
            self.graph.add_edge(n1, n2, weight=adjacency_matrix[n1][n2])

    def navigate(self, from_node: int, to_node: int, avoid_nodes=None):
        if avoid_nodes is None:
            networkx.shortest_path(self.graph, from_node, to_node, weight='weight')
        elif (from_node in avoid_nodes) or (to_node in avoid_nodes):
            return None
        graph = self.graph.copy()
        for node in avoid_nodes:
            graph.remove_node(node)
        try:
            return networkx.shortest_path(graph, from_node, to_node, weight='weight')
        except networkx.exception.NetworkXNoPath:
            return None

    def interpolate(self, path, count_points=100):
        centers = [self.nodes[i] for i in path]
        # noinspection PyTupleAssignmentBalance
        tck, u = interpolate.splprep(np.transpose(centers), s=0)
        x, y = interpolate.splev(np.linspace(0, 1, count_points), tck)
        return x.astype(int), y.astype(int)

    @staticmethod
    def draw_curve(image, x, y):
        for i in range(1, len(x)):
            cv2.line(image, (x[i-1], y[i-1]), (x[i], y[i]), CURVE_COLOR, 2)


if __name__ == "__main__":
    cv2.namedWindow(WINDOW_NAME)
    graph = NavigationGraph(DATA_FILE_PATH)
    navigator = Navigator(DATA_FILE_PATH)

    image = cv2.imread('field.png')
    graph.draw_edges(image)
    graph.draw_nodes(image)

    avoid = [5, 32, 6]
    path = navigator.navigate(0, 31, avoid_nodes=avoid)

    if path is not None:
        points = navigator.interpolate(path)
        navigator.draw_curve(image, *points)
        graph.draw_path(image, path, avoid_nodes=avoid)
        print(f'Path found: {path}')
    else:
        print('No path was found.')

    cv2.imshow(WINDOW_NAME, image)
    cv2.waitKey(0)
    cv2.destroyWindow(WINDOW_NAME)
