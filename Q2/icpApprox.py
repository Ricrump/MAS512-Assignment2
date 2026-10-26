import os
import json
import numpy as np
import open3d as o3d

trajectory_data = []
current_transform = np.eye(4)
prev_pcd = None

print("Berechne ICP-Trajektorie...")
for frame in range(99):
    filepath = os.path.join("./Q2/croppedPlyAnimation", f"croppedFrame_{(frame+1):04d}.ply")

    if not os.path.exists(filepath):
        continue

    # load pointcloud
    pcd = o3d.io.read_point_cloud(filepath)
    
    if prev_pcd is None:
        prev_pcd = pcd
        trajectory_data.append({
            "center": current_transform[:3, 3].tolist(),
            "matrix": current_transform[:3, :3].tolist()
        })
        continue

    threshold = 0.02  
    trans_init = np.eye(4) 

    # estimate normal of point with neighbouring points
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
    prev_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))

    # run ICP
    reg_p2p = o3d.pipelines.registration.registration_icp(
        pcd, prev_pcd, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
    )

    current_transform = current_transform @ np.linalg.inv(reg_p2p.transformation)
    
    # Daten für JSON aufbereiten (numpy arrays zu Listen konvertieren)
    trajectory_data.append({
        "center": current_transform[:3, 3].tolist(),
        "matrix": current_transform[:3, :3].tolist()
    })
    
    prev_pcd = pcd

# save data
json_out_path = "./Q2/croppedPlyAnimation/icpEvaluatedData.json"
with open(json_out_path, "w") as f:
    json.dump(trajectory_data, f, indent=4)

print(f"ICP-Berechnung abgeschlossen! Daten gespeichert unter: {json_out_path}")

