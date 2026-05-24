import numpy as np
from collections import deque

# Probability thresholds
PRIOR_PROB = 0.5  # Unknown space
OCC_PROB = 0.75  # Occupied space
FREE_PROB = 0.45  # Free space
TOLERANCE = 0.015  # Tolerance for detecting unknown space

def is_frontier(grid, x, y):
    """
    Checks if a cell is a frontier point.
    A frontier point is free space (< FREE_PROB) adjacent to unknown space (~PRIOR_PROB).
    """
    prob = grid[x, y]
    neighbors = [
    (x-1, y),    # Up
    (x+1, y),    # Down
    (x, y-1),    # Left
    (x, y+1),    # Right
]
    if prob >= FREE_PROB:  # Not free space
        return False

    # Check neighbors for adjacency to unknown space

    for nx, ny in neighbors:
        if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]:
            neighbor_prob = grid[nx, ny]
            if abs(neighbor_prob - PRIOR_PROB) < TOLERANCE:  # Adjacent to unknown space
                return True
    return False

#reference: https://arxiv.org/pdf/1806.03581

def merge_frontier(frontier_list):
    """
    Merge frontier cells into clusters based on 8-neighbor connectivity.
    Ensures no clusters have fewer than 5 points.
    """

    def get_neighbors(cell):
        """Returns the 8 neighbors of a cell."""
        x, y = cell
        return [
            (x-1, y), (x+1, y), (x, y-1), (x, y+1),  # Cardinal directions
            (x-1, y-1), (x-1, y+1), (x+1, y-1), (x+1, y+1)  # Diagonals
        ]

    # Sort frontier list for consistent processing
    frontier_list = sorted(frontier_list)

    visited = set()  # To track visited points
    clusters = []  # List to store clusters

    for cell in frontier_list:
        if cell in visited:
            continue

        # Start a new cluster with BFS
        cluster = []
        queue = deque([cell])

        while queue:
            current = queue.popleft()

            if current in visited:
                continue

            visited.add(current)
            cluster.append(current)

            # Check neighbors
            for neighbor in get_neighbors(current):
                if neighbor in frontier_list and neighbor not in visited:
                    queue.append(neighbor)

        # Add cluster if it meets the size requirement
        if len(cluster) > 5:
            clusters.append(cluster)
    #print(clusters)
    print(len(clusters))
    return clusters


        
import math

def get_centroid(clusters):
    """
    Calculate and return the centroid of each cluster.
    The centroid is the arithmetic mean of the points in the cluster.
    """
    centroids = []
    for cluster in clusters:
        if cluster:  # Ensure the cluster is not empty
            avg_x = sum(point[0] for point in cluster) / len(cluster)
            avg_y = sum(point[1] for point in cluster) / len(cluster)
            centroids.append((int(avg_x), int(avg_y)))
        else:
            # Handle empty clusters gracefully (optional)
            centroids.append(None)
    return centroids



def wavefront_frontier_detection(grid, start_x, start_y):
    """
    Wavefront Frontier Detector algorithm for probability-based occupancy grids.
    Identifies frontiers in a given occupancy grid starting from (start_x, start_y).
    """
    map_open_list = set()  # Points to be explored
    map_close_list = set()  # Points already explored
    frontiers = []  # List of individual frontier cells

    # Outer BFS queue (explores known free space)
    outer_queue = deque([(start_x, start_y)])
    map_open_list.add((start_x, start_y))

    while outer_queue:
        x, y = outer_queue.popleft()
        map_close_list.add((x, y))

        # Check if this cell is a frontier point
        if is_frontier(grid, x, y):
            frontiers.append((x, y))

        # Explore neighbors in the outer BFS
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        for nx, ny in neighbors:
            if (0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]
                and (nx, ny) not in map_open_list
                and (nx, ny) not in map_close_list):
                if grid[nx, ny] < FREE_PROB:  # Free space
                    outer_queue.append((nx, ny))
                    map_open_list.add((nx, ny))
    clusters =  merge_frontier(frontiers)
    return clusters

#reference: https://arxiv.org/pdf/1806.03581