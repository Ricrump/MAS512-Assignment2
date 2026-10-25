import open3d as o3d
import numpy as np
from ouster.sdk import open_source
from ouster.sdk.client import ChanField, XYZLut
import os 

pcap_path = "swinging_load.pcap"
metadata_path = "swinging_load.json"

source = open_source(pcap_path, meta_path=metadata_path, index=True)

# lookup-table
xyzlut = XYZLut(source.metadata)

if len(source) == 0:
    print("No Lidar-Scans found in PCAP-File")
else:
    for idx in range(len(source)):
        print(f"iteration: {idx}")
        scan = source[idx]
        range_data = scan.field(ChanField.RANGE)
        reflectivity_data = scan.field(ChanField.REFLECTIVITY).flatten()
        
        # calculate meters
        xyz = xyzlut(range_data)
        pts = xyz.reshape(-1, 3)
        
        valid_mask = np.any(pts != 0, axis=1)
        pts_filtered = pts[valid_mask]
        refl_filtered = reflectivity_data[valid_mask]
        
        if len(pts_filtered) > 0:
            # build Open3D pointcloud
            base_pcd = o3d.geometry.PointCloud()
            base_pcd.points = o3d.utility.Vector3dVector(pts_filtered)

            # crop pointcloud with bounding box
            min_bound = np.array([0.319, 1.092, -0.088])
            max_bound = np.array([1.162, 2.632, 0.755])
            aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
            pcd = base_pcd.crop(aabb)

            # normalize reflectivity + show as grayscale
            max_refl = np.max(refl_filtered) if np.max(refl_filtered) > 0 else 255.0
            refl_normalized = refl_filtered / max_refl
            colors = np.vstack((refl_normalized, refl_normalized, refl_normalized)).T
            pcd.colors = o3d.utility.Vector3dVector(colors)
            
            # save
            output_dir = "croppedPlyAnimation"
            filename = os.path.join(output_dir, f"croppedFrame_{idx:04d}.ply")
            o3d.io.write_point_cloud(filename, pcd, write_ascii=False)
            print("Erste 3 echte Koordinaten:\n", pts_filtered[:3])