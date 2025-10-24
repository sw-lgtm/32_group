from typing import List, Tuple, Optional


class RegionDetector:
    def __init__(self,
                 padding: int = 10,             # Padding around the region of interest (≥ object width/pixels)
                 min_width: int = 4,            # Ignore narrow boxes
                 min_height: int = 4,           # Ignore short boxes
                 min_size: int = 200,           # Minimum area filter
                 shrink_allowed: bool = False,  # Whether shrinking is allowed (default: not allowed for stability)
                 max_shrink_step: int = 3       # Maximum shrinking per update (if allowed)
                 ):
        self.region: Optional[Tuple[int, int, int, int]] = None  # (x_min, y_min, x_max, y_max)
        self.padding = int(padding)
        self.min_w = int(min_width)
        self.min_h = int(min_height)
        self.min_size = int(min_size)
        self.shrink_allowed = bool(shrink_allowed)
        self.max_shrink = int(max_shrink_step)

    def _calculate_region_hash(self, grid: List[List[int]],
                               area: Optional[Tuple[int, int, int, int]] = None) -> int:
        """
        Fast hash computation for a region of the grid (sampled approach)
        """
        height, width = len(grid), len(grid[0])

        if area:
            x_min, y_min, x_max, y_max = area
        else:
            # Sampling strategy: check every Nth row/column
            step = max(1, min(height, width) // 15)
            x_min, y_min, x_max, y_max = 0, 0, width - 1, height - 1

        # Sampled hash computation
        sample_values = []
        for y in range(y_min, y_max + 1, max(1, (y_max - y_min) // 8)):
            for x in range(x_min, x_max + 1, max(1, (x_max - x_min) // 8)):
                if 0 <= y < height and 0 <= x < width:
                    sample_values.append(grid[y][x])

        return hash(tuple(sample_values))

    def _initial_scan(self, grid: List[List[int]],
                      must_contain: Optional[Tuple[int, int]] = None) -> Optional[Tuple[int, int, int, int]]:
        """
        First pass scan: optimize by scanning rows and columns independently
        Time complexity: O(H + W) instead of O(H×W)
        """
        height, width = len(grid), len(grid[0])

        # Step 1: Find the valid row range (scanning columns only)
        y_min, y_max = self._find_valid_rows(grid)
        if y_min is None:
            return self.region

        # Step 2: Within the valid row range, find column boundaries
        x_min, x_max = self._find_valid_columns(grid, y_min, y_max)
        if x_min is None:
            return self.region

        # Calculate final bounds
        return self._compute_final_bounds(
            x_min, y_min, x_max, y_max, width, height, must_contain
        )

    def _find_valid_rows(self, grid: List[List[int]]) -> Tuple[Optional[int], Optional[int]]:
        """
        Quickly find the row range containing valid regions
        Time complexity: O(H×sampling points)
        """
        height, width = len(grid), len(grid[0])

        # Sampling strategy: check specific points in each row
        sample_columns = [0, width // 4, width // 2, 3 * width // 4, width - 1]

        y_min = None
        y_max = None

        # From top to bottom, find the first valid row
        for y in range(height):
            is_valid = any(grid[y][x] <= 30 for x in sample_columns if 0 <= x < width)
            if is_valid:
                y_min = y
                break

        if y_min is None:
            return None, None

        # From bottom to top, find the last valid row
        for y in range(height - 1, y_min - 1, -1):
            is_valid = any(grid[y][x] <= 30 for x in sample_columns if 0 <= x < width)
            if is_valid:
                y_max = y
                break

        return y_min, y_max

    def _find_valid_columns(self, grid: List[List[int]],
                            y_min: int, y_max: int) -> Tuple[Optional[int], Optional[int]]:
        """
        Find the column boundaries within valid row range
        """
        width = len(grid[0])

        # Sampling rows
        sample_rows = []
        step = max(1, (y_max - y_min) // 4)
        for y in range(y_min, y_max + 1, step):
            sample_rows.append(y)
        if y_max not in sample_rows:
            sample_rows.append(y_max)

        x_min = width
        x_max = -1

        for y in sample_rows:
            row = grid[y]
            # Scan from left to right
            for x in range(width):
                if row[x] <= 30:
                    x_min = min(x_min, x)
                    break

            # Scan from right to left
            for x in range(width - 1, -1, -1):
                if row[x] <= 30:
                    x_max = max(x_max, x)
                    break

        if x_max < x_min:
            return None, None

        return x_min, x_max

    def _update_from_changed_area(self, grid: List[List[int]],
                                  changed_area: Tuple[int, int, int, int],
                                  must_contain: Optional[Tuple[int, int]]) -> Optional[Tuple[int, int, int, int]]:
        """
        Incremental update: only scan the changed region
        Suitable for real-time SLAM-like map updates
        """
        height, width = len(grid), len(grid[0])
        cx_min, cy_min, cx_max, cy_max = changed_area

        # Expand search area slightly to avoid missing edges
        margin = 3
        cx_min = max(0, cx_min - margin)
        cy_min = max(0, cy_min - margin)
        cx_max = min(width - 1, cx_max + margin)
        cy_max = min(height - 1, cy_max + margin)

        # Scan only within the changed region
        x_min, y_min = width, height
        x_max, y_max = -1, -1
        known_count = 0

        for y in range(cy_min, cy_max + 1):
            row = grid[y]
            for x in range(cx_min, cx_max + 1):
                if row[x] <= 30:
                    known_count += 1
                    x_min = min(x_min, x)
                    x_max = max(x_max, x)
                    y_min = min(y_min, y)
                    y_max = max(y_max, y)

        if known_count == 0:
            return self.region

        # Merge with the existing region
        if self.region is not None:
            prev_x_min, prev_y_min, prev_x_max, prev_y_max = self.region
            x_min = min(x_min, prev_x_min)
            y_min = min(y_min, prev_y_min)
            x_max = max(x_max, prev_x_max)
            y_max = max(y_max, prev_y_max)

        return self._compute_final_bounds(
            x_min, y_min, x_max, y_max, width, height, must_contain
        )

    def _compute_final_bounds(self, x_min, y_min, x_max, y_max, width, height, must_contain):
        # Add bounds computation logic here if needed
        return x_min, y_min, x_max, y_max

    def is_inside(self, x: float, y: float) -> bool:
        """
        Check if the given point (x, y) is inside the region
        """
        if not self.region:
            return True
        x_min, y_min, x_max, y_max = self.region
        return x_min <= x <= x_max and y_min <= y <= y_max

    def get_region(self) -> Optional[Tuple[int, int, int, int]]:
        """
        Return the current region of interest
        """
        return self.region
