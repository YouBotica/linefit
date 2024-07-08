Popular Dataset config
---

> [!IMPORTANT]  
> We assume that you set the (0,0,0) at the sensor center position where sensor located. 
> Since I knew some data process will set to base link which will cause the sensor height parameter doesn't work.

Here is what I mean, all visualization here are provided by [DeFlow vis scripts](https://github.com/KTH-RPL/DeFlow/blob/main/tests/scene_flow.py):
![](../docs/sensor_pos.png)

- [x] KITTI: 1x 64 lidar
- [x] Nuscenes: 1x 32 lidar
- [x] Argoverse 2: 2x 32 lidar
- [ ] [Semi-indoor](https://github.com/KTH-RPL/DynamicMap_Benchmark?tab=readme-ov-file#dataset--scripts): 1x 16 lidar
- [ ] Waymo: custom lidar.
- [ ] [Zod](https://zod.zenseact.com/): 128 lidar + 2x 16 lidar
... More on the way