# draw_map.py (fixed)
from typing import List, Tuple, Optional
import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.lines as mlines

class MapDrawer:
    ROBOT_HEAD_LEN_M  = 0.18
    ROBOT_HEAD_W_M    = 0.12
    ROBOT_SHAFT_LEN_M = 0.10

    def __init__(self, map_size_pixels: int, map_size_meters: float,
                 title: str = "ListGridMap Viewer",
                 show_trajectory: bool = True,
                 origin_lower_left: bool = True):
        self.N = map_size_pixels
        self.map_size_meters = float(map_size_meters)
        self.m_per_pix = self.map_size_meters / float(self.N)
        self.title = title
        self.show_traj = show_trajectory
        self.origin = "lower" if origin_lower_left else "upper"

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        try: self.fig.canvas.set_window_title("SLAM")
        except Exception: pass
        self.ax.set_title(self.title)
        self.ax.set_xlabel("X (m)"); self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect("equal")
        self.extent_m = (0.0, self.N*self.m_per_pix, 0.0, self.N*self.m_per_pix)
        self.ax.set_xlim(self.extent_m[0], self.extent_m[1])
        self.ax.set_ylim(self.extent_m[2], self.extent_m[3])

        self.img_artist = None
        self.car_artist = None
        self.text_artist = None          # <<< 新增：复用的文字对象
        self.traj_line: Optional[mlines.Line2D] = None  # <<< 新增：复用的轨迹对象
        self.traj_xm: List[float] = []
        self.traj_ym: List[float] = []

        plt.ion(); plt.show(block=False)

    def _pix_to_m(self, x_pix: float, y_pix: float) -> Tuple[float, float]:
        return x_pix * self.m_per_pix, y_pix * self.m_per_pix

    def _draw_car(self, x_m: float, y_m: float, theta_deg: float):
        # 1) 箭头（为简单起见仍然移除重画；也可改成复用：更新 FancyArrow 的 verts）
        if self.car_artist is not None:
            try: self.car_artist.remove()
            except Exception: pass
            self.car_artist = None

        theta_rad = math.radians(theta_deg)
        dx = self.ROBOT_SHAFT_LEN_M * math.cos(theta_rad)
        dy = self.ROBOT_SHAFT_LEN_M * math.sin(theta_rad)

        self.car_artist = self.ax.arrow(
            x_m, y_m, dx, dy,
            head_width=self.ROBOT_HEAD_W_M,
            head_length=self.ROBOT_HEAD_LEN_M,
            fc="r", ec="r", length_includes_head=True, zorder=5
        )

        # 2) 文字：复用句柄而不是每帧新增
        label = f"({x_m:.2f} m, {y_m:.2f} m), θ={theta_deg:.1f}°"
        if self.text_artist is None:
            self.text_artist = self.ax.text(
                x_m, y_m, label, fontsize=9, color="r",
                va="bottom", zorder=6, clip_on=True
            )
        else:
            self.text_artist.set_position((x_m, y_m))
            self.text_artist.set_text(label)

    def _update_traj(self, x_m: float, y_m: float):
        if not self.show_traj:
            return
        self.traj_xm.append(x_m); self.traj_ym.append(y_m)
        if self.traj_line is None:
            self.traj_line = mlines.Line2D(self.traj_xm, self.traj_ym,
                                           linewidth=1.0, color="b", zorder=3)
            self.ax.add_line(self.traj_line)
        else:
            self.traj_line.set_data(self.traj_xm, self.traj_ym)

    def display(self, grid: List[List[int]],
                car_pose_pix_deg: Optional[Tuple[float, float, float]] = None) -> bool:
        # 地图
        if self.img_artist is None:
            self.img_artist = self.ax.imshow(
                grid, cmap=cm.gray, vmin=0, vmax=255,
                origin=self.origin, extent=self.extent_m, zorder=1
            )
        else:
            self.img_artist.set_data(grid)

        # 小车与轨迹
        if car_pose_pix_deg is not None:
            x_pix, y_pix, theta_deg = car_pose_pix_deg
            x_m, y_m = self._pix_to_m(x_pix, y_pix)
            self._draw_car(x_m, y_m, theta_deg)
            self._update_traj(x_m, y_m)

        try:
            self.fig.canvas.draw_idle()
            plt.pause(0.01)
            return True
        except Exception:
            return False


# 下面是一个最小示例（可删）
if __name__ == "__main__":
    N, M = 800, 32.0
    grid = [[200 for _ in range(N)] for _ in range(N)]
    for y in range(150, 650):
        for x in range(395, 405):
            grid[y][x] = 20

    drawer = MapDrawer(N, M, title="Demo", show_trajectory=True, origin_lower_left=True)
    x_pix, y_pix, theta = 100.0, 100.0, 45.0
    for _ in range(200):
        ok = drawer.display(grid, (x_pix, y_pix, theta))
        if not ok: break
        x_pix += 2.0; y_pix += 2.0
