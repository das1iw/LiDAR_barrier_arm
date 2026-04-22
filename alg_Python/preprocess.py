import numpy as np
from scipy.spatial import cKDTree

# -------------------- PREPROCESSING --------------------

def _to_xyz(points):
    if points is None:
        return np.empty((0, 3), dtype=np.float32)
    pts = np.asarray(points)

    if pts.dtype.names is not None:
        if all(k in pts.dtype.names for k in ('x', 'y', 'z')):
            out = np.empty((len(pts), 3), dtype=np.float32)
            out[:, 0] = pts['x']
            out[:, 1] = pts['y']
            out[:, 2] = pts['z']
            return out
        return np.empty((0, 3), dtype=np.float32)

    if pts.ndim == 1:
        try:
            pts = np.array([(p[0], p[1], p[2]) for p in pts], dtype=np.float32)
        except Exception:
            return np.empty((0, 3), dtype=np.float32)

    if pts.ndim == 2 and pts.shape[1] >= 3:
        sliced = pts[:, :3]
        # Avoid copy if already correct dtype and contiguous
        if sliced.dtype == np.float32 and sliced.flags['C_CONTIGUOUS']:
            return sliced
        return sliced.astype(np.float32)

    return np.empty((0, 3), dtype=np.float32)


def remove_ground(points, z_min=-1.0):
    """
    Remove ground by simple height threshold (z > z_min).
    """
    xyz = _to_xyz(points)
    if xyz.shape[0] == 0:
        return xyz

    mask = xyz[:, 2] > z_min
    return xyz[mask]



def remove_outliers(points, k=8, z_thresh=2.5):
    xyz = _to_xyz(points)
    n = xyz.shape[0]
    if n < k + 1:
        return xyz

    tree = cKDTree(xyz)
    # query returns (distances, indices) — only distances needed
    dists, _ = tree.query(xyz, k=k + 1, workers=-1)  # k+1 includes self
    mean_knn = dists[:, 1:].mean(axis=1)              # skip self (index 0)

    mu = mean_knn.mean()
    sigma = mean_knn.std() + 1e-6
    mask = (mean_knn - mu) / sigma < z_thresh
    return xyz[mask]

def voxel_downsample(points, voxel_size=0.05):
    xyz = _to_xyz(points)
    if xyz.shape[0] == 0 or voxel_size <= 0:
        return xyz
    coords = np.floor(xyz / voxel_size).astype(np.int32)
    _, inv, counts = np.unique(coords, axis=0, return_inverse=True, return_counts=True)

    # Stay in float32 — halves accumulator memory
    sums = np.zeros((counts.shape[0], 3), dtype=np.float32)
    np.add.at(sums, inv, xyz)
    sums /= counts[:, None]   # in-place divide, no extra allocation
    return sums


def crop_roi(points,
             x_min=0.0, x_max=10.0,
             y_min=-5.0, y_max=5.0,
             z_min=-1.0, z_max=5.0,
             voxel_size=0.05,
             do_outlier_removal=True):
    """
    Full preprocessing:
      1) normalize input
      2) remove ground
      3) crop ROI
      4) remove outliers (optional)
      5) voxel downsample

    Returns Nx3 float32 array.
    """
    xyz = _to_xyz(points)
    if xyz.shape[0] == 0:
        return xyz

    # 1) ROI crop & remove ground
    mask = (
        (xyz[:, 0] > x_min) & (xyz[:, 0] < x_max) &
        (xyz[:, 1] > y_min) & (xyz[:, 1] < y_max) &
        (xyz[:, 2] > z_min) & (xyz[:, 2] < z_max)
    )
    xyz = xyz[mask]

    if xyz.shape[0] == 0:
        return xyz

    # 2) outlier removal
    if do_outlier_removal:
        xyz = remove_outliers(xyz)

    # 3) voxel downsample
    xyz = voxel_downsample(xyz, voxel_size=voxel_size)

    return xyz
