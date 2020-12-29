import math
import numpy as np


def _get_min_max_vertices(polygon):
    (xMax, yMax) = np.max(polygon, axis=0)
    (xMin, yMin) = np.min(polygon, axis=0)
    return xMin, yMin, xMax, yMax


def _get_bounding_box_vertices(polygon):
    (xMin, yMin, xMax, yMax) = _get_min_max_vertices(polygon)
    return np.array(((xMin, yMin), (xMax, yMin), (xMax, yMax), (xMin, yMax)))


def _get_grid_range(x_min, y_min, x_max, y_max, cell_size):
    i_max = math.ceil((x_max - x_min) / cell_size)
    j_max = math.ceil((y_max - y_min) / cell_size)
    return int(i_max), int(j_max)


def _get_grid_cells(i_max, j_max, x_min, y_min, cell_size):
    total_grids = i_max * j_max
    grids = np.zeros((total_grids, 2))
    ctr = 0
    for j in range(j_max):
        for i in range(i_max):
            grids[ctr, 0] = x_min + i * cell_size
            grids[ctr, 1] = y_min + j * cell_size
            ctr += 1
    return grids


def _get_grid_cell_index(i_max, j_max, k):
    j = math.floor(k / i_max)
    i = k - j * i_max
    return int(i), int(j)


def _get_line_segments(polygon):
    n = np.shape(polygon)[0]
    lines = np.zeros((n, 4))
    for s in range(n):
        if s != (n - 1):
            lines[s, 0] = polygon[s, 0]
            lines[s, 1] = polygon[s, 1]
            lines[s, 2] = polygon[s + 1, 0]
            lines[s, 3] = polygon[s + 1, 1]
        else:
            lines[s, 0] = polygon[s, 0]
            lines[s, 1] = polygon[s, 1]
            lines[s, 2] = polygon[0, 0]
            lines[s, 3] = polygon[0, 1]

    return lines


def _get_grid_rays(cells_center, x_max):
    k = np.shape(cells_center)[0]
    grid_rays = np.zeros((k, 4))
    grid_rays[:, 0:2] = cells_center
    grid_rays[:, 2] = x_max + 1
    grid_rays[:, 3] = cells_center[:, 1]
    return grid_rays


def is_intersecting(line_segment, other_line_segments):
    n = np.shape(other_line_segments)[0]
    line_segments = np.full((n, 4), line_segment)

    f = line_segments[:, 2:4] - line_segments[:, 0:2]
    g = other_line_segments[:, 2:4] - other_line_segments[:, 0:2]
    qmp = other_line_segments[:, 0:2] - line_segments[:, 0:2]

    fxg = np.cross(f, g)
    qpxf = np.cross(qmp, f)

    mantissa = 1e-10

    # collinear but disjoint
    is_collinear = np.logical_and(
        np.abs(fxg) <= mantissa,
        np.abs(qpxf) <= mantissa
    )

    f = f[np.logical_not(is_collinear)]
    g = g[np.logical_not(is_collinear)]
    qmp = qmp[np.logical_not(is_collinear)]
    fxg = fxg[np.logical_not(is_collinear)]
    qpxf = qpxf[np.logical_not(is_collinear)]

    # parallel and non intersecting
    is_parallel = np.logical_and(
        np.abs(fxg) <= mantissa,
        np.abs(qpxf) > mantissa
    )

    f = f[np.logical_not(is_parallel)]
    g = g[np.logical_not(is_parallel)]
    qmp = qmp[np.logical_not(is_parallel)]
    fxg = fxg[np.logical_not(is_parallel)]
    qpxf = qpxf[np.logical_not(is_parallel)]

    t = np.cross(qmp, g) / fxg
    u = np.cross(qmp, f) / fxg

    return np.logical_and(
        np.abs(fxg) >= mantissa,
        np.logical_and(
            np.logical_and(0 <= t, t <= 1),
            np.logical_and(0 <= u, u <= 1)
        )
    )


def _check_lie_inside(ray, polygon_line_segments):
    num_intersections = np.sum(is_intersecting(ray, polygon_line_segments))

    if num_intersections == 0 or num_intersections % 2 == 0:
        return False
    else:
        return True


def _get_inside_grid_index(rays, polygon_line_segments):
    k = np.shape(rays)[0]
    flag = np.zeros((k), dtype=bool)
    for i in range(k):
        flag[i] = _check_lie_inside(rays[i], polygon_line_segments)
    return flag


class GridCells(object):
    def __init__(self):
        pass


def get_grid_cells(polygon, c):
    # namespace to hold grid related methods and variables
    grid_cells = GridCells()

    min_max = _get_min_max_vertices(polygon)
    grid_cells.min_max = _get_grid_range(min_max[0], min_max[1], min_max[2], min_max[3], c)

    grid_cells.bounding_box = _get_bounding_box_vertices(polygon)

    grid_cells.cells = _get_grid_cells(grid_cells.min_max[0], grid_cells.min_max[1],
                                       min_max[0], min_max[1], c)

    grid_cells.cells_center = grid_cells.cells + c / 2

    grid_cells.get_index = _get_grid_cell_index

    grid_cells.valid = _get_inside_grid_index(
        _get_grid_rays(grid_cells.cells_center, _get_min_max_vertices(polygon)[2]),
        _get_line_segments(polygon))

    return grid_cells
