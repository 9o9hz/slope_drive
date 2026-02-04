import numpy as np
import cv2

def find_special_points(skeleton):
    """
    Finds endpoints (1 neighbor) and branch points (>2 neighbors) in a skeleton.
    """
    binary_skeleton = (skeleton > 0).astype(np.uint8)
    
    # Kernel for convolution to count neighbors
    kernel = np.array([[1, 1, 1],
                       [1, 10, 1],
                       [1, 1, 1]], dtype=np.uint8)
    
    neighbor_map = cv2.filter2D(binary_skeleton, -1, kernel, borderType=cv2.BORDER_CONSTANT)

    # Endpoints have 1 neighbor -> value 11 (10 center + 1 neighbor)
    endpoints = np.argwhere(neighbor_map == 11)
    
    # Branch points have >2 neighbors -> value > 12 (10 center + >2 neighbors)
    branch_points = np.argwhere(neighbor_map > 12)
    
    return endpoints, branch_points

def trace_simple_branch(skeleton, start_point, branch_points_set):
    """
    Traces a simple, non-branching path from a start_point until it hits a branch point or an end.
    """
    path = []
    current_pixel = tuple(start_point)
    prev_pixel = None

    while True:
        path.append(current_pixel)
        r, c = current_pixel

        # Stop if we hit a branch point (that isn't our starting point)
        if current_pixel in branch_points_set and current_pixel != tuple(start_point):
            return path, current_pixel

        # Find the next pixel in the path by checking 8-connectivity
        next_pixel = None
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                
                nr, nc = r + dr, c + dc
                neighbor = (nr, nc)

                if (0 <= nr < skeleton.shape[0] and 0 <= nc < skeleton.shape[1] and
                        skeleton[nr, nc] > 0 and neighbor != prev_pixel):
                    next_pixel = neighbor
                    break
            if next_pixel:
                break
        
        if next_pixel is None:
            return path, None
        
        prev_pixel = current_pixel
        current_pixel = next_pixel

def prune_branches(skeleton_image, length_threshold):
    """
    Prunes short branches from a skeleton image.
    """
    pruned_skeleton = skeleton_image.copy()
    endpoints, branch_points = find_special_points(pruned_skeleton)
    branch_points_set = set(map(tuple, branch_points))

    for point in endpoints:
        start_point = tuple(point)
        
        if pruned_skeleton[start_point[0], start_point[1]] == 0:
            continue

        branch_pixels, junction = trace_simple_branch(pruned_skeleton, start_point, branch_points_set)
        
        if junction is not None and len(branch_pixels) < length_threshold:
            for r, c in branch_pixels:
                if (r, c) != junction:
                    pruned_skeleton[r, c] = 0
    
    return pruned_skeleton