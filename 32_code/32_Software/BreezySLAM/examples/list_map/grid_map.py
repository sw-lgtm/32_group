from typing import List
import numpy as np
from .draw_map import MapDrawer

class ListGridMap:

    def __init__(self,MAP_SIZE_PIXELS, MAP_SIZE_METERS):
        self.w = MAP_SIZE_PIXELS
        self.h = MAP_SIZE_PIXELS
        self.map_size_pixels = MAP_SIZE_PIXELS

        # grid[y][x]
        self.grid: List[List[int]] = [[128 for _ in range(self.w)] for _ in range(self.h)] #未知值默认128
        self.map_scale_meters_per_pixel = MAP_SIZE_METERS / float(MAP_SIZE_PIXELS)
        self.CurrCarPose = None

        self.drawer = MapDrawer(MAP_SIZE_PIXELS,MAP_SIZE_METERS,title="My SLAM Map", show_trajectory=True, origin_lower_left=True)


    def UpdateMap(self,mapbytes,origin_lower_left = True):
        mapnp = np.reshape(np.frombuffer(mapbytes, dtype=np.uint8), (self.map_size_pixels, self.map_size_pixels))

        #if origin_lower_left:

            #mapnp = np.flipud(mapnp) 会生成拷贝，频繁大图更新时有开销。如果后面显示端能用
            #mapnp = np.flipud(mapnp) #保证下标就是像素坐标

        self.grid = mapnp.tolist()

    def m2pix(self,x_m,y_m):
        s = self.map_scale_meters_per_pixel
        return x_m/s,y_m/s

    #更新车在地图中的位置
    def SetCarPose(self,x_m,y_m,theta_deg):
        x_pix,y_pix = self.m2pix(x_m,y_m)
        self.CurrCarPose = (x_pix,y_pix,theta_deg)

    def GetCarPose(self):
        if self.CurrCarPose is None:
            raise RuntimeError("车辆位姿尚未设置")
        return self.CurrCarPose[0],self.CurrCarPose[1],self.CurrCarPose[2]

    def draw(self):
        self.drawer.display(self.grid,self.CurrCarPose)
        print("drawing the map...")


