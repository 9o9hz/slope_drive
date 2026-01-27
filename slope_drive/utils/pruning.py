import numpy as np
import cv2

def find_special_points(skeleton):
    """
    Finds endpoints (1 neighbor) and branch points (>2 neighbors) in a skeleton.
    This works on a skeletonized image where pixels are 0 or >0.
    
    Args:
        skeleton: A binary skeleton image (numpy array).

    Returns:
        A tuple containing:
        - endpoints: A list of (row, col) coordinates for pixels with 1 neighbor.
        - branch_points: A list of (row, col) coordinates for pixels with >2 neighbors.
    """
    # Ensure skeleton is binary 0 or 1 for this operation
    binary_skeleton = (skeleton > 0).astype(np.uint8)
    
    # Use a 3x3 kernel. The center element is weighted differently (e.g., 10)
    # so we can count neighbors of a pixel by looking at the convolution result.
    # A skeleton pixel's output value will be 10 + (number of neighbors).
    kernel = np.array([[1, 1, 1],
                       [1, 10, 1],
                       [1, 1, 1]], dtype=np.uint8)
    
    neighbor_map = cv2.filter2D(binary_skeleton, -1, kernel, borderType=cv2.BORDER_CONSTANT)

    # Endpoints have 1 neighbor, so map value is 10 + 1 = 11.
    endpoints = np.argwhere(neighbor_map == 11)
    
    # Branch points have more than 2 neighbors, so map value is > 10 + 2 = 12.
    branch_points = np.argwhere(neighbor_map > 12)
    
    return endpoints, branch_points

def trace_simple_branch(skeleton, start_point, branch_points_set):
    """
    Traces a simple, non-branching path from a start_point until it hits a branch point or an end.
    
    Args:
        skeleton: The binary skeleton image.
        start_point: The (row, col) tuple to start tracing from (usually an endpoint).
        branch_points_set: A set of (row, col) tuples for all branch points for fast lookup.

    Returns:
        A tuple containing:
        - path: A list of pixels [(r,c), ...] in the traced branch.
        - junction: The (r,c) of the branch point hit, or None if it's an isolated line.
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

                # Check bounds, if it's a skeleton pixel, and not the pixel we just came from
                if (0 <= nr < skeleton.shape[0] and 0 <= nc < skeleton.shape[1] and
                        skeleton[nr, nc] > 0 and neighbor != prev_pixel):
                    next_pixel = neighbor
                    break
            if next_pixel:
                break
        
        if next_pixel is None:
            # Reached the end of a line without hitting a branch point
            return path, None
        
        prev_pixel = current_pixel
        current_pixel = next_pixel

def prune_branches(skeleton_image, length_threshold):
    """
    Prunes short branches from a skeleton image.

    Args:
        skeleton_image: A binary skeleton image (numpy array, values 0 or 255).
        length_threshold: The minimum length (in pixels) for a branch to be kept.

    Returns:
        A new skeleton image with short branches removed.
    """
    # Create a copy to work on, as we will be modifying it
    pruned_skeleton = skeleton_image.copy()
    
    # Find all endpoints and branch points
    endpoints, branch_points = find_special_points(pruned_skeleton)
    
    # Use a set of tuples for branch points for O(1) average time complexity lookups
    branch_points_set = set(map(tuple, branch_points))

    # Iterate over each found endpoint
    for point in endpoints:
        start_point = tuple(point)
        
        # Before tracing, check if the endpoint still exists on the skeleton.
        # It might have been removed as part of another branch.
        if pruned_skeleton[start_point[0], start_point[1]] == 0:
            continue

        # Trace the branch starting from the endpoint
        branch_pixels, junction = trace_simple_branch(pruned_skeleton, start_point, branch_points_set)
        
        # If the traced branch is shorter than the threshold and it leads to a junction, prune it.
        if junction is not None and len(branch_pixels) < length_threshold:
            # Remove the pixels belonging to this short branch
            for r, c in branch_pixels:
                # IMPORTANT: Do not remove the junction point itself, only the branch leading to it.
                if (r, c) != junction:
                    pruned_skeleton[r, c] = 0
    
    return pruned_skeleton
