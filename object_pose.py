# # object_pose.py
# import numpy as np
# import pybullet as p

# def depth_to_pointcloud(depth_buffer, view_matrix, projection_matrix, width, height):
#     """
#     Convert PyBullet depth buffer + view/proj matrices into Nx3 world points.
#     - depth_buffer: (H,W) array values in [0,1]
#     - view_matrix & projection_matrix: 16-element lists (column-major)
#     Returns: pts_world (H,W,3), valid_mask (H,W)
#     """
#     view = np.array(view_matrix, dtype=np.float32).reshape((4, 4), order='F')
#     proj = np.array(projection_matrix, dtype=np.float32).reshape((4, 4), order='F')
#     cam = proj @ view
#     inv_cam = np.linalg.inv(cam)

#     H, W = height, width
#     i_coords = np.arange(H)
#     j_coords = np.arange(W)
#     js, is_ = np.meshgrid(j_coords, i_coords)

#     z_ndc = depth_buffer * 2.0 - 1.0
#     x_ndc = (js / (W - 1)) * 2.0 - 1.0
#     y_ndc = 1.0 - (is_ / (H - 1)) * 2.0

#     clip = np.stack([x_ndc, y_ndc, z_ndc, np.ones_like(z_ndc)], axis=-1)
#     clip_flat = clip.reshape(-1, 4).T

#     world_flat = inv_cam @ clip_flat
#     world_flat = world_flat.T
#     w = world_flat[:, 3:4]
#     valid = (np.abs(w[:, 0]) > 1e-6)

#     world_coords = np.zeros((world_flat.shape[0], 3), dtype=np.float32)
#     world_coords[valid] = (world_flat[valid, 0:3] / w[valid])

#     pts_world = world_coords.reshape((H, W, 3))
#     return pts_world, valid.reshape(H, W)


# def compute_masked_centroid_and_pca(pts_world, valid_mask, pixel_mask, k=500):
#     """
#     Given 3D points and a mask, compute centroid & orientation (PCA).
#     Returns: centroid (3,), axes (3x3)
#     """
#     mask = (pixel_mask & valid_mask)
#     if np.count_nonzero(mask) < 10:
#         return None, None

#     pts = pts_world[mask]
#     if pts.shape[0] > k:
#         idx = np.random.choice(pts.shape[0], k, replace=False)
#         pts = pts[idx]

#     centroid = pts.mean(axis=0)
#     X = pts - centroid
#     U, S, Vt = np.linalg.svd(X, full_matrices=False)
#     axes = Vt.T
#     return centroid, axes


# def get_object_pose(loaded_objects, seg, pts_world, valid_mask):
#     """
#     For each object in loaded_objects:
#     - Find centroid & orientation
#     - Draw debug lines in PyBullet
#     """
#     debug_ids = []
#     for obj_id in loaded_objects:
#         pixel_mask = (seg == obj_id)
#         if not np.any(pixel_mask):
#             continue

#         centroid, axes = compute_masked_centroid_and_pca(pts_world, valid_mask, pixel_mask)
#         if centroid is None:
#             continue

#         print(f"Object {obj_id} centroid: {centroid}, main_axis: {axes[:,0]}")

#         s = 0.02
#         x_axis = axes[:, 0] * s
#         y_axis = axes[:, 1] * s
#         z_axis = axes[:, 2] * s
#         id1 = p.addUserDebugLine(centroid - x_axis, centroid + x_axis, [1, 0, 0], 2, 0.05)
#         id2 = p.addUserDebugLine(centroid - y_axis, centroid + y_axis, [0, 1, 0], 2, 0.05)
#         id3 = p.addUserDebugLine(centroid - z_axis, centroid + z_axis, [0, 0, 1], 2, 0.05)
#         id_text = p.addUserDebugText(f"id:{obj_id}", centroid + np.array([0,0,0.04]), textSize=1.2)
#         debug_ids.extend([id1, id2, id3, id_text])

#     return debug_ids
# object_pose.py
import numpy as np
import pybullet as p
import scipy.spatial.transform

def depth_to_pointcloud(depth_buffer, view_matrix, projection_matrix, width, height):
    """
    Convert PyBullet depth buffer + view/proj matrices into Nx3 world points.
    - depth_buffer: (H,W) array values in [0,1]
    - view_matrix & projection_matrix: 16-element lists (column-major)
    Returns: pts_world (H,W,3), valid_mask (H,W)
    """
    view = np.array(view_matrix, dtype=np.float32).reshape((4, 4), order='F')
    proj = np.array(projection_matrix, dtype=np.float32).reshape((4, 4), order='F')
    cam = proj @ view
    inv_cam = np.linalg.inv(cam)

    H, W = height, width
    i_coords = np.arange(H)
    j_coords = np.arange(W)
    js, is_ = np.meshgrid(j_coords, i_coords)

    z_ndc = depth_buffer * 2.0 - 1.0
    x_ndc = (js / (W - 1)) * 2.0 - 1.0
    y_ndc = 1.0 - (is_ / (H - 1)) * 2.0

    clip = np.stack([x_ndc, y_ndc, z_ndc, np.ones_like(z_ndc)], axis=-1)
    clip_flat = clip.reshape(-1, 4).T

    world_flat = inv_cam @ clip_flat
    world_flat = world_flat.T
    w = world_flat[:, 3:4]
    valid = (np.abs(w[:, 0]) > 1e-6)

    world_coords = np.zeros((world_flat.shape[0], 3), dtype=np.float32)
    world_coords[valid] = (world_flat[valid, 0:3] / w[valid])

    pts_world = world_coords.reshape((H, W, 3))
    return pts_world, valid.reshape(H, W)


def compute_masked_centroid_and_pca(pts_world, valid_mask, pixel_mask, k=500):
    """
    Given 3D points and a mask, compute centroid & orientation (PCA).
    Returns: centroid (3,), axes (3x3), singular values
    """
    mask = (pixel_mask & valid_mask)
    if np.count_nonzero(mask) < 10:
        return None, None, None

    pts = pts_world[mask]
    if pts.shape[0] > k:
        idx = np.random.choice(pts.shape[0], k, replace=False)
        pts = pts[idx]

    centroid = pts.mean(axis=0)
    X = pts - centroid
    U, S, Vt = np.linalg.svd(X, full_matrices=False)
    axes = Vt.T
    return centroid, axes, S


def get_object_pose_and_grasp(loaded_objects, seg, pts_world, valid_mask):
    """
    For each object in loaded_objects:
    - Find centroid & orientation
    - Compute grasp pose (position + quaternion)
    - Draw debug lines
    Returns: dict {obj_id: {"centroid": ..., "grasp_pos": ..., "grasp_quat": ...}}
    """
    results = {}
    debug_ids = []

    for obj_id in loaded_objects:
        pixel_mask = (seg == obj_id)
        if not np.any(pixel_mask):
            continue

        centroid, axes, S = compute_masked_centroid_and_pca(pts_world, valid_mask, pixel_mask)
        if centroid is None:
            continue

        # --- Shape-based grasping strategy ---
        if S[0] < 0.03:  # small object
            grasp_pos = centroid + np.array([0, 0.01, 0.06])
            R = np.eye(3)  # gripper top-down
            strategy = "small object → centroid grasp"

        elif S[0] > 2 * S[1]:  # long object (pen/stick)
            grasp_pos = centroid + np.array([0, 0.01, 0.06])
            R = np.column_stack([axes[:, 0], axes[:, 1], axes[:, 2]])  # align with main axis
            strategy = "long object → grasp along main axis"

        else:  # flat object
            grasp_pos = centroid + np.array([0, 0.0, 0.2])
            R = np.eye(3)  # top-down grasp
            strategy = "flat object → top-down grasp"

        # Quaternion for PyBullet
        # quat = p.getQuaternionFromRotationMatrix(R.flatten().tolist())
        quat = scipy.spatial.transform.Rotation.from_matrix(R).as_quat() 

        print(f"[{strategy}] Object {obj_id}: centroid={centroid}, grasp_pos={grasp_pos}, quat={quat}")

        # --- Debug lines ---
        s = 0.02
        x_axis = axes[:, 0] * s
        y_axis = axes[:, 1] * s
        z_axis = axes[:, 2] * s
        id1 = p.addUserDebugLine(centroid - x_axis, centroid + x_axis, [1, 0, 0], 2, 0.05)
        id2 = p.addUserDebugLine(centroid - y_axis, centroid + y_axis, [0, 1, 0], 2, 0.05)
        id3 = p.addUserDebugLine(centroid - z_axis, centroid + z_axis, [0, 0, 1], 2, 0.05)
        id_text = p.addUserDebugText(f"id:{obj_id}", centroid + np.array([0, 0, 0.04]), textSize=1.2)
        debug_ids.extend([id1, id2, id3, id_text])

        results[obj_id] = {
            "centroid": centroid,
            "grasp_pos": grasp_pos,
            "grasp_quat": quat,
            "strategy": strategy
        }

    return results, debug_ids
