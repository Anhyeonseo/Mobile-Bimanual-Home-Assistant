"""Versioned map-point goals and a conservative offline grid feasibility check.

A 3D display is separate from the floor navigation map. No click can choose a
new coordinate frame, floor height, collision margin or map revision on behalf
of the robot. This planner is a static test baseline, not Nav2 or 3D SLAM.
"""
from __future__ import annotations
from copy import deepcopy
from dataclasses import dataclass
import heapq
import math
import re

from .fetch import InvalidTask


def identifier(value, field):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,96}", value):
        raise InvalidTask(f"invalid {field}")
    return value


def number(value, field):
    if type(value) not in (int, float):
        raise InvalidTask(f"{field} must be finite numeric")
    try:
        result = float(value)
    except OverflowError as error:
        raise InvalidTask(f"{field} is too large") from error
    if not math.isfinite(result) or abs(result) > 1e6:
        raise InvalidTask(f"invalid {field}")
    return result


@dataclass(frozen=True)
class NavigateRequest:
    request_id: str
    map_id: str
    map_revision: str
    floor_id: str
    frame_id: str
    x: float
    y: float
    z: float
    yaw_rad: float

    @classmethod
    def from_dict(cls, data):
        fields = {'schema_version','operation','request_id','map_id','map_revision','floor_id','frame_id','point','yaw_rad'}
        if not isinstance(data, dict) or set(data) != fields:
            raise InvalidTask("invalid navigate_to fields")
        if type(data['schema_version']) is not int or data['schema_version'] != 1 or data['operation'] != 'navigate_to':
            raise InvalidTask("unsupported navigation request")
        point = data['point']
        if not isinstance(point, dict) or set(point) != {'x','y','z'}:
            raise InvalidTask("point must contain x/y/z in map metres")
        yaw = number(data['yaw_rad'], 'yaw_rad')
        if not -math.pi <= yaw <= math.pi:
            raise InvalidTask("yaw_rad outside [-pi, pi]")
        return cls(*(identifier(data[k], k) for k in ('request_id','map_id','map_revision','floor_id','frame_id')),
                   *(number(point[k], k) for k in ('x','y','z')), yaw)


class NavigationMap:
    def __init__(self, data: dict):
        fields = {'schema_version','map_id','revision','floor_id','frame_id','floor_z_m','resolution_m',
                  'origin','rows','robot_radius_m','clearance_m','surface_tolerance_m'}
        if not isinstance(data, dict) or set(data) != fields or type(data['schema_version']) is not int or data['schema_version'] != 1:
            raise InvalidTask("invalid navigation map")
        self.map_id, self.revision, self.floor_id, self.frame_id = (
            identifier(data[k], k) for k in ('map_id','revision','floor_id','frame_id'))
        self.floor_z = number(data['floor_z_m'], 'floor_z_m')
        self.resolution = number(data['resolution_m'], 'resolution_m')
        radius, clearance, self.tolerance = (number(data[k], k) for k in
                                           ('robot_radius_m','clearance_m','surface_tolerance_m'))
        if not 0 < self.resolution <= 10 or not 0 < radius <= 10 or not 0 <= clearance <= 10 or not 0 < self.tolerance <= 0.1:
            raise InvalidTask("invalid map scale or footprint")
        origin = data['origin']
        if not isinstance(origin, dict) or set(origin) != {'x','y'}:
            raise InvalidTask("invalid map origin")
        self.ox, self.oy = (number(origin[k], k) for k in ('x','y'))
        rows = data['rows']
        if not isinstance(rows, list) or not 1 <= len(rows) <= 256 or not isinstance(rows[0], str) or not 1 <= len(rows[0]) <= 256:
            raise InvalidTask("invalid map dimensions")
        if any(not isinstance(row, str) or len(row) != len(rows[0]) or set(row) - set('.#?') for row in rows):
            raise InvalidTask("rows must be rectangular . free / # occupied / ? unknown")
        self.rows = tuple(rows)
        self.width, self.height = len(rows[0]), len(rows)
        padding = math.ceil((radius + clearance) / self.resolution)
        if padding >= min(self.width, self.height):
            raise InvalidTask("footprint exceeds navigation map")
        # Prefix sums bound inflation work even for large footprint settings.
        summed = [[0] * (self.width + 1) for _ in range(self.height + 1)]
        for y, row in enumerate(rows):
            for x, cell in enumerate(row):
                summed[y+1][x+1] = (cell != '.') + summed[y][x+1] + summed[y+1][x] - summed[y][x]
        blocked = set()
        for y in range(self.height):
            for x in range(self.width):
                if x < padding or y < padding or x >= self.width-padding or y >= self.height-padding:
                    blocked.add((x,y))
                    continue
                left, right, bottom, top = x-padding, x+padding+1, y-padding, y+padding+1
                if summed[top][right] - summed[bottom][right] - summed[top][left] + summed[bottom][left]:
                    blocked.add((x,y))
        self.blocked = frozenset(blocked)
        self._document = deepcopy(data)

    def descriptor(self) -> dict:
        return {**deepcopy(self._document), 'mode': 'simulation', 'navigation_dimensions': 2,
                'scene_mesh_available': False, 'rows_order': 'increasing_map_y',
                'inflation': 'conservative_square', 'physical_map_verified': False}

    def _cell(self, x, y):
        x, y = number(x, 'x'), number(y, 'y')
        cell = (math.floor((x-self.ox)/self.resolution), math.floor((y-self.oy)/self.resolution))
        if not 0 <= cell[0] < self.width or not 0 <= cell[1] < self.height:
            raise InvalidTask("point_outside_map")
        if cell in self.blocked:
            raise InvalidTask("point_blocked_or_insufficient_clearance")
        return cell

    def plan(self, request: NavigateRequest, current_xy: tuple[float,float]) -> dict:
        if (request.map_id, request.map_revision, request.floor_id, request.frame_id) != (
                self.map_id, self.revision, self.floor_id, self.frame_id):
            raise InvalidTask("map_revision_floor_or_frame_mismatch")
        if abs(request.z - self.floor_z) > self.tolerance:
            raise InvalidTask("select_navigable_floor_surface")
        start, goal = self._cell(*current_xy), self._cell(request.x, request.y)
        queue, cost, parent = [(0, start)], {start: 0}, {}
        while queue:
            _, cell = heapq.heappop(queue)
            if cell == goal:
                break
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nxt = (cell[0]+dx,cell[1]+dy)
                if not 0 <= nxt[0] < self.width or not 0 <= nxt[1] < self.height or nxt in self.blocked:
                    continue
                new = cost[cell]+1
                if new < cost.get(nxt, math.inf):
                    cost[nxt], parent[nxt] = new, cell
                    heapq.heappush(queue, (new+abs(nxt[0]-goal[0])+abs(nxt[1]-goal[1]), nxt))
        if goal not in cost:
            raise InvalidTask("no_path")
        cells, cell = [goal], goal
        while cell != start:
            cell = parent[cell]
            cells.append(cell)
        cells.reverse()
        path = [{'x': self.ox+(x+0.5)*self.resolution, 'y': self.oy+(y+0.5)*self.resolution} for x,y in cells]
        return {'map_id': self.map_id, 'map_revision': self.revision, 'floor_id': self.floor_id,
                'frame_id': self.frame_id, 'path_xy': path,
                'goal': {'x': request.x, 'y': request.y, 'z': self.floor_z, 'yaw_rad': request.yaw_rad},
                'planner': 'offline_grid_astar', 'physical_path_verified': False}
