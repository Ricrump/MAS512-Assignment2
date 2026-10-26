import json
import os
import time
import numpy as np
import open3d as o3d


def get_plane_params(triplet):
    return (
        np.cross(triplet[0] - triplet[1], triplet[0] - triplet[2]),
        triplet[0],
    )

def plane_ransac(
    points, iterations=500, dist_threshold=0.005, inlier_threshold=80
):
    n_points = len(points)
    for _ in range(iterations):
        normal, anchor = get_plane_params(
            points[np.random.choice(n_points, 3, replace=False)]
        )
        distances = (
            np.abs((points[:, :3] @ normal) - (normal @ anchor))
            / np.linalg.norm(normal)
        )
        inlier_mask = distances < dist_threshold
        if np.sum(inlier_mask) > inlier_threshold:
            return normal / np.linalg.norm(normal), anchor, inlier_mask
    print("Insufficient inliers found. Run again.")
    exit()


json_path = "./Q2/croppedPlyAnimation/evaluatedData.json"

if not os.path.exists(json_path):
    animation_data = []
    prev_normal1, prev_normal2 = None, None

    # calculate RANSAC for every Frame
    for frame in range(99):
        filepath = os.path.join(
            "./Q2/croppedPlyAnimation", f"croppedFrame_{(frame+1):04d}.ply"
        )
        print(filepath)
        points = np.asarray(o3d.io.read_point_cloud(filepath).points)

        # calculate first side (plane) of cuboid
        normal1, _, mask1 = plane_ransac(points, 1500, 0.008, 130)

        # calculate second side (plane) of cuboid; has to be perpendicular to first side
        for _ in range(20000):
            normal2, _, mask2 = plane_ransac(points, 1500, 0.008, 70)
            if abs(normal1 @ normal2) <= 0.01:
                break
        else:
            print("No perpendicular plane found. Run again.")
            exit()

        # stabilize axes with previous frames (prevent fliping)
        if prev_normal1 is not None and prev_normal2 is not None:
            if (
                abs(np.dot(normal1, prev_normal2)) + abs(np.dot(normal2, prev_normal1))
                > abs(np.dot(normal1, prev_normal1)) + abs(np.dot(normal2, prev_normal2))
            ):
                normal1, normal2, mask1, mask2 = normal2, normal1, mask2, mask1

        if prev_normal1 is not None and np.dot(normal1, prev_normal1) < 0:
            normal1 = -normal1
        if prev_normal2 is not None and np.dot(normal2, prev_normal2) < 0:
            normal2 = -normal2

        prev_normal1, prev_normal2 = normal1, normal2

        # calculate third normal (side)
        normal3 = np.cross(normal1, normal2)
        normal3 /= np.linalg.norm(normal3)

        # align coordinate frame (by singular value decomposition without stretched and skewed part)
        U, _, Vt = np.linalg.svd(np.column_stack([normal1, normal2, normal3]))
        R_clean = U @ Vt
        trans_pts = points[mask1 | mask2] @ R_clean

        # evaluate box dimensions (by only taking points with z-Value between 5 and 95 percentile in account)
        lower = np.array(
            [
                np.min(trans_pts[:, 0]),
                np.min(trans_pts[:, 1]),
                np.percentile(trans_pts[:, 2], 5),
            ]
        )
        higher = np.array(
            [
                np.max(trans_pts[:, 0]),
                np.max(trans_pts[:, 1]),
                np.percentile(trans_pts[:, 2], 95),
            ]
        )
        center = ((lower + higher) / 2.0) @ R_clean.T

        animation_data.append(
            {"center": center.tolist(), "matrix": R_clean.tolist()}
        )

    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    with open(json_path, "w") as f:
        json.dump(animation_data, f)

# VISUALIZATION/Animation
with open(json_path, "r") as f:
    loaded_frames = json.load(f)

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="RANSAC-Animation", width=1024, height=768)

coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
vis.add_geometry(coordinate_frame)

pcd = o3d.io.read_point_cloud(
    os.path.join("./Q2/croppedPlyAnimation", "croppedFrame_0001.ply")
)
vis.add_geometry(pcd)
view_ctl = vis.get_view_control()

current_T = np.eye(4)
for frame_idx, frame_data in enumerate(loaded_frames):
    # form transformation-matrix
    target_T = np.eye(4)
    target_T[:3, :3] = frame_data["matrix"]
    target_T[:3, 3] = frame_data["center"]

    delta_T = target_T @ np.linalg.inv(current_T)
    coordinate_frame.transform(delta_T)
    current_T = target_T

    temp_pcd = o3d.io.read_point_cloud(
        os.path.join("./Q2/croppedPlyAnimation", f"croppedFrame_{(frame_idx+1):04d}.ply")
    )
    pcd.points = temp_pcd.points
    if temp_pcd.has_colors():
        pcd.colors = temp_pcd.colors

    vis.update_geometry(pcd)
    vis.update_geometry(coordinate_frame)
    vis.poll_events()
    vis.update_renderer()

    view_ctl.set_lookat([0.7405, 1.862, 0.3335])
    view_ctl.set_zoom(0.3)
    time.sleep(0.1)

vis.run()
vis.destroy_window()


