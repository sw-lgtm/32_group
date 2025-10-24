# frontier_explorer.py
from dataclasses import dataclass
from typing import List, Tuple, Optional, Set
import math


# ============================================
# Configuration parameters for frontier exploration
# ============================================
@dataclass
class ExplorationConfig:
    # Cell classification thresholds
    free_cell_threshold: int = 210
    unknown_cell_threshold: int = 80
    obstacle_cell_threshold: int = 60

    # Safety parameters
    safety_clearance: int = 7
    min_exploration_distance: int = 22
    max_exploration_distance: int = 200
    robot_exclusion_radius: float = 4.0

    # Sampling parameters
    coarse_scan_interval: int = 2
    spatial_bucket_size: int = 14
    max_candidates_to_evaluate: int = 48

    # Reachability analysis
    local_clearance_radius: int = 120
    local_free_ratio_threshold: float = 0.45
    local_evaluation_window: int = 5

    # Goal orientation
    goal_influence_weight: float = 0.5


class FrontierDetectionEngine:
    """
    Advanced frontier detection and evaluation system that identifies
    boundary regions between explored and unexplored areas.
    """

    def __init__(self, configuration: Optional[ExplorationConfig] = None) -> None:
        self.config = configuration or ExplorationConfig()
        self.previous_candidates: List[Tuple[int, int]] = []

    # ============================================
    # Main Public Interface
    # ============================================

    def detect_exploration_frontiers(
            self,
            occupancy_map: List[List[int]],
            search_region: Optional[Tuple[int, int, int, int]] = None,
            updated_regions: Optional[List[Tuple[int, int, int, int]]] = None
    ) -> List[Tuple[int, int]]:
        """
        Identify frontier cells that represent boundaries between known and unknown areas.

        Args:
            occupancy_map: 2D grid representing environment occupancy
            search_region: Optional bounding box (x_min, y_min, x_max, y_max) to limit search
            updated_regions: List of recently changed regions for incremental processing

        Returns:
            List of (x, y) coordinates representing frontier candidate positions
        """
        map_height = len(occupancy_map)
        if map_height == 0:
            return []
        map_width = len(occupancy_map[0])

        search_bounds = self._compute_search_bounds(search_region, map_width, map_height)

        if updated_regions:
            return self._incremental_frontier_scan(occupancy_map, search_bounds, updated_regions)
        else:
            return self._full_map_frontier_scan(occupancy_map, search_bounds)

    def evaluate_navigation_candidates(
            self,
            robot_pose: Tuple[float, float],
            frontier_candidates: List[Tuple[int, int]],
            occupancy_map: List[List[int]],
            goal_position: Optional[Tuple[int, int]] = None,
            exploration_region: Optional[Tuple[int, int, int, int]] = None
    ) -> List[Tuple[int, int]]:
        """
        Evaluate and rank frontier candidates for navigation suitability.

        Args:
            robot_pose: Current robot position (x, y)
            frontier_candidates: List of candidate frontier positions
            occupancy_map: Environment occupancy information
            goal_position: Optional goal position for directional bias
            exploration_region: Valid region for exploration

        Returns:
            Ranked list of suitable navigation targets
        """
        if not frontier_candidates:
            return []

        robot_x, robot_y = int(robot_pose[0]), int(robot_pose[1])

        # Multi-stage evaluation pipeline
        candidates = frontier_candidates

        # Stage 1: Distance-based filtering
        candidates = self._filter_by_navigation_distance(candidates, (robot_x, robot_y))
        if not candidates:
            return []

        # Stage 2: Spatial deduplication
        candidates = self._deduplicate_spatially(candidates)
        candidates = candidates[:self.config.max_candidates_to_evaluate]

        # Stage 3: Reachability assessment
        candidates = self._assess_navigation_reachability(candidates, (robot_x, robot_y),
                                                          occupancy_map, exploration_region)
        if not candidates:
            return []

        # Stage 4: Goal-aware ranking
        if goal_position:
            candidates = self._rank_by_exploration_utility(candidates, (robot_x, robot_y), goal_position)

        self.previous_candidates = candidates
        return candidates

    # ============================================
    # Frontier Detection Core Logic
    # ============================================

    def _full_map_frontier_scan(
            self,
            occupancy_map: List[List[int]],
            search_bounds: Tuple[int, int, int, int]
    ) -> List[Tuple[int, int]]:
        """Perform comprehensive frontier scan across the entire search region."""
        x_min, y_min, x_max, y_max = search_bounds
        map_height = len(occupancy_map)
        map_width = len(occupancy_map[0])

        frontier_cells = []
        scan_step = max(1, self.config.coarse_scan_interval)

        for y in range(max(y_min, 1), min(y_max, map_height - 1), scan_step):
            for x in range(max(x_min, 1), min(x_max, map_width - 1), scan_step):
                if self._is_valid_frontier_cell(occupancy_map, x, y):
                    frontier_cells.append((x, y))

        return frontier_cells

    def _incremental_frontier_scan(
            self,
            occupancy_map: List[List[int]],
            search_bounds: Tuple[int, int, int, int],
            updated_regions: List[Tuple[int, int, int, int]]
    ) -> List[Tuple[int, int]]:
        """Efficiently scan only regions that have recently changed."""
        x_min, y_min, x_max, y_max = search_bounds
        frontier_cells = []
        expansion_margin = self.config.safety_clearance + 2

        for region_xmin, region_ymin, region_xmax, region_ymax in updated_regions:
            # Expand region to account for frontier propagation
            expanded_region = (
                max(x_min, region_xmin - expansion_margin),
                max(y_min, region_ymin - expansion_margin),
                min(x_max, region_xmax + expansion_margin),
                min(y_max, region_ymax + expansion_margin)
            )

            region_frontiers = self._full_map_frontier_scan(occupancy_map, expanded_region)
            frontier_cells.extend(region_frontiers)

        return self._remove_duplicate_positions(frontier_cells)

    def _is_valid_frontier_cell(
            self,
            occupancy_map: List[List[int]],
            x: int,
            y: int
    ) -> bool:
        """
        Determine if a cell qualifies as a valid frontier candidate.

        Criteria:
        1. Cell is in free space (above free threshold)
        2. Has unknown neighbors (indicating exploration boundary)
        3. Has sufficient local clearance from obstacles
        """
        cell_value = occupancy_map[y][x]

        # Must be in navigable space
        if cell_value < self.config.free_cell_threshold:
            return False

        # Must border unknown territory
        if not self._has_unknown_neighbors(occupancy_map, x, y):
            return False

        # Must have safe local environment
        if not self._has_obstacle_clearance(occupancy_map, x, y):
            return False

        return True

    def _has_unknown_neighbors(
            self,
            occupancy_map: List[List[int]],
            x: int,
            y: int
    ) -> bool:
        """Check if cell has adjacent unknown areas."""
        map_height = len(occupancy_map)
        map_width = len(occupancy_map[0])

        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue

                nx, ny = x + dx, y + dy
                if 0 <= nx < map_width and 0 <= ny < map_height:
                    neighbor_value = occupancy_map[ny][nx]
                    if (self.config.unknown_cell_threshold < neighbor_value <
                            self.config.free_cell_threshold):
                        return True
        return False

    def _has_obstacle_clearance(
            self,
            occupancy_map: List[List[int]],
            x: int,
            y: int
    ) -> bool:
        """Verify sufficient clearance from obstacles in local region."""
        map_height = len(occupancy_map)
        map_width = len(occupancy_map[0])
        clearance = self.config.safety_clearance
        clearance_squared = clearance * clearance

        for dy in range(-clearance, clearance + 1):
            for dx in range(-clearance, clearance + 1):
                if dx * dx + dy * dy > clearance_squared:
                    continue

                nx, ny = x + dx, y + dy
                if 0 <= nx < map_width and 0 <= ny < map_height:
                    if occupancy_map[ny][nx] <= self.config.obstacle_cell_threshold:
                        return False
        return True

    # ============================================
    # Candidate Evaluation Pipeline
    # ============================================

    def _filter_by_navigation_distance(
            self,
            candidates: List[Tuple[int, int]],
            robot_position: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        """Filter candidates based on practical navigation distances."""
        robot_x, robot_y = robot_position
        min_dist_sq = self.config.robot_exclusion_radius ** 2
        max_dist_sq = self.config.max_exploration_distance ** 2
        absolute_min_sq = self.config.min_exploration_distance ** 2

        valid_candidates = []

        for cand_x, cand_y in candidates:
            distance_squared = (cand_x - robot_x) ** 2 + (cand_y - robot_y) ** 2

            if (min_dist_sq < distance_squared <= max_dist_sq and
                    distance_squared >= absolute_min_sq):
                valid_candidates.append((cand_x, cand_y))

        return valid_candidates

    def _deduplicate_spatially(self, candidates: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Remove spatially redundant candidates using grid hashing."""
        if not candidates:
            return []

        bucket_size = self.config.spatial_bucket_size
        spatial_buckets = {}

        for x, y in candidates:
            bucket_key = (x // bucket_size, y // bucket_size)
            if bucket_key not in spatial_buckets:
                spatial_buckets[bucket_key] = (x, y)

        return list(spatial_buckets.values())

    def _assess_navigation_reachability(
            self,
            candidates: List[Tuple[int, int]],
            robot_position: Tuple[int, int],
            occupancy_map: List[List[int]],
            valid_region: Optional[Tuple[int, int, int, int]]
    ) -> List[Tuple[int, int]]:
        """Evaluate which candidates are practically reachable."""
        map_height = len(occupancy_map)
        map_width = len(occupancy_map[0])
        region_bounds = self._compute_search_bounds(valid_region, map_width, map_height)

        robot_x, robot_y = robot_position
        max_range_sq = self.config.local_clearance_radius ** 2

        reachable_candidates = []

        for cand_x, cand_y in candidates:
            # Check region bounds
            if not self._is_within_bounds((cand_x, cand_y), region_bounds):
                continue

            # Check proximity to robot
            if (cand_x - robot_x) ** 2 + (cand_y - robot_y) ** 2 > max_range_sq:
                continue

            # Check local navigability
            if self._is_locally_navigable(occupancy_map, cand_x, cand_y):
                reachable_candidates.append((cand_x, cand_y))

        return reachable_candidates if reachable_candidates else candidates[:2]

    def _is_locally_navigable(
            self,
            occupancy_map: List[List[int]],
            x: int,
            y: int
    ) -> bool:
        """Assess local area for sufficient free space ratio."""
        map_height = len(occupancy_map)
        map_width = len(occupancy_map[0])
        window = self.config.local_evaluation_window

        free_cells = 0
        total_cells = 0

        for dy in range(-window, window + 1):
            for dx in range(-window, window + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < map_width and 0 <= ny < map_height:
                    total_cells += 1
                    if occupancy_map[ny][nx] >= self.config.free_cell_threshold:
                        free_cells += 1

        return (total_cells > 0 and
                free_cells / total_cells >= self.config.local_free_ratio_threshold)

    def _rank_by_exploration_utility(
            self,
            candidates: List[Tuple[int, int]],
            robot_position: Tuple[int, int],
            goal_position: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        """Rank candidates by combination of goal direction and proximity."""
        robot_x, robot_y = robot_position
        goal_x, goal_y = goal_position

        # Compute goal direction vector
        goal_vector_x, goal_vector_y = goal_x - robot_x, goal_y - robot_y
        goal_distance = max(1.0, math.hypot(goal_vector_x, goal_vector_y))

        scored_candidates = []

        for cand_x, cand_y in candidates:
            # Candidate direction vector
            cand_vector_x, cand_vector_y = cand_x - robot_x, cand_y - robot_y
            cand_distance = max(1.0, math.hypot(cand_vector_x, cand_vector_y))

            # Alignment with goal direction (cosine similarity)
            dot_product = cand_vector_x * goal_vector_x + cand_vector_y * goal_vector_y
            alignment_score = dot_product / (cand_distance * goal_distance)  # [-1, 1]
            normalized_alignment = (alignment_score + 1.0) * 0.5  # [0, 1]

            # Proximity to goal
            goal_proximity = math.hypot(cand_x - goal_x, cand_y - goal_y)
            proximity_score = 1.0 / (1.0 + goal_proximity / 120.0)  # [0, 1]

            # Combined score
            exploration_score = (
                    self.config.goal_influence_weight * proximity_score +
                    (1.0 - self.config.goal_influence_weight) * normalized_alignment
            )

            scored_candidates.append(((cand_x, cand_y), exploration_score))

        # Sort by descending score
        scored_candidates.sort(key=lambda item: -item[1])
        return [candidate for candidate, _ in scored_candidates]

    # ============================================
    # Utility Methods
    # ============================================

    def _compute_search_bounds(
            self,
            specified_region: Optional[Tuple[int, int, int, int]],
            map_width: int,
            map_height: int
    ) -> Tuple[int, int, int, int]:
        """Compute valid search boundaries from input or default to map bounds."""
        if not specified_region:
            return (0, 0, map_width - 1, map_height - 1)

        x_min, y_min, x_max, y_max = specified_region
        x_min = max(0, min(x_min, map_width - 1))
        y_min = max(0, min(y_min, map_height - 1))
        x_max = max(0, min(x_max, map_width - 1))
        y_max = max(0, min(y_max, map_height - 1))

        # Ensure proper ordering
        if x_max < x_min:
            x_min, x_max = x_max, x_min
        if y_max < y_min:
            y_min, y_max = y_max, y_min

        return (x_min, y_min, x_max, y_max)

    def _is_within_bounds(
            self,
            position: Tuple[int, int],
            bounds: Tuple[int, int, int, int]
    ) -> bool:
        """Check if position is within specified bounds."""
        x, y = position
        x_min, y_min, x_max, y_max = bounds
        return x_min <= x <= x_max and y_min <= y <= y_max

    def _remove_duplicate_positions(self, positions: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Remove duplicate positions while preserving order."""
        seen_positions: Set[Tuple[int, int]] = set()
        unique_positions = []

        for position in positions:
            if position not in seen_positions:
                seen_positions.add(position)
                unique_positions.append(position)

        return unique_positions


# ============================================
# Demonstration
# ============================================
if __name__ == "__main__":
    # Create test environment
    MAP_HEIGHT, MAP_WIDTH = 60, 60
    test_occupancy_map = [[128 for _ in range(MAP_WIDTH)] for _ in range(MAP_HEIGHT)]

    # Mark some areas as free and occupied
    test_occupancy_map[30][30] = 255  # Free space
    test_occupancy_map[12][12] = 0  # Obstacle
    test_occupancy_map[10][40] = 90  # Unknown area

    # Initialize frontier detection system
    frontier_detector = FrontierDetectionEngine()

    # Detect frontiers
    frontier_locations = frontier_detector.detect_exploration_frontiers(
        test_occupancy_map,
        search_region=(0, 0, MAP_WIDTH - 1, MAP_HEIGHT - 1)
    )

    # Evaluate navigation targets
    navigation_targets = frontier_detector.evaluate_navigation_candidates(
        robot_pose=(25, 25),
        frontier_candidates=frontier_locations,
        occupancy_map=test_occupancy_map,
        goal_position=(50, 50)
    )

    print(f"Detected {len(frontier_locations)} frontier candidates")
    print(f"Selected {len(navigation_targets)} navigation targets")