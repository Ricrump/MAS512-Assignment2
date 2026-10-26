import os
import json
import numpy as np
import open3d as o3d

def load_trajectory(json_path, center_to_origin=False):
    if not os.path.exists(json_path):
        print(f"Couldnt find: {json_path}, run icpApprox or cuboidRANSAC before")
        return []
        
    with open(json_path, "r") as f:
        raw_data = json.load(f)
        
    if not raw_data:
        return []
        
    first_center = np.array(raw_data[0]["center"]) if center_to_origin else np.zeros(3)
    
    frames = []
    for item in raw_data:
        shifted_center = np.array(item["center"]) - first_center
        frames.append({
            "center": shifted_center.tolist(),
            "matrix": item["matrix"]
        })
    return frames

def add_trajectory_to_vis(vis, frames, line_color, step=5, coord_size=0.05):
    if not frames:
        return

    centers = [f["center"] for f in frames]
    lines = [[i, i + 1] for i in range(len(centers) - 1)]
    colors = [line_color for _ in range(len(lines))]

    # 1. Trajektorien-Linie erstellen
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(centers),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)
    vis.add_geometry(line_set)

    # 2. Koordinaten-Frames (Posen) in definierten Schritten hinzufügen
    for i in range(0, len(frames), step):
        frame_data = frames[i]
        pose_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=coord_size)
        
        transform = np.eye(4)
        transform[:3, :3] = frame_data["matrix"]
        transform[:3, 3] = frame_data["center"]
        
        pose_frame.transform(transform)
        vis.add_geometry(pose_frame)

# load Data
icp_frames = load_trajectory("./Q2/croppedPlyAnimation/icpEvaluatedData.json", center_to_origin=False)
ransac_frames = load_trajectory("./Q2/croppedPlyAnimation/evaluatedData.json", center_to_origin=True)

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Trajectories: RANSAC (Orange), ICP (Blue)", width=1024, height=768)

vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5))

add_trajectory_to_vis(vis, icp_frames, line_color=[0.0, 0.5, 1.0], step=3, coord_size=0.05)
add_trajectory_to_vis(vis, ransac_frames, line_color=[1.0, 0.5, 0.0], step=3, coord_size=0.05)

view_ctl = vis.get_view_control()
view_ctl.set_zoom(0.4)
vis.run()
vis.destroy_window()