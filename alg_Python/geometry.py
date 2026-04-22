import numpy as np


def fit_line_pca(points):
    """
    Robust PCA line fitting.

    Returns:
        direction (unit vector),
        centroid,
        eigenvalues (sorted descending),
        linearity (lambda0 / lambda1)
    """
    pts = np.asarray(points)

    # Handle irregular input
    if pts.ndim == 1:
        pts = np.array([(p[0], p[1], p[2]) for p in pts])

    if pts.shape[0] < 5:
        return np.zeros(3), np.zeros(3), np.zeros(3), 0.0

    xyz = pts[:, :3]

    # Compute centroid and center data
    centroid = xyz.mean(axis=0)
    centered = xyz - centroid

    # Covariance
    n = centered.shape[0]
    cov = (centered.T @ centered) / (n-1)



    # Eigen decomposition (stable for symmetric matrices)
    eigvals, eigvecs = np.linalg.eigh(cov)

    # Sort descending
    order = np.argsort(eigvals)[::-1]
    eigvals = eigvals[order]
    eigvecs = eigvecs[:, order]

    # Stabilize eigenvalues (prevents collapse during motion/noise)
    eigvals[1] = max(eigvals[1], 1e-3)

    # Principal direction
    direction = eigvecs[:, 0]
    norm = np.linalg.norm(direction)

    if norm < 1e-6:
        return np.zeros(3), centroid, eigvals, 0.0

    direction = direction / norm

    # Enforce consistent direction (reduces angle flipping)
    if direction[0] < 0:
        direction = -direction

    # Linearity metric
    linearity = eigvals[0] / (eigvals[1] + 1e-6)

    return direction, centroid, eigvals, linearity


def compute_angle(direction, reference_axis=None):
    """
    Compute angle (degrees) between barrier direction and reference axis.

    Returns angle in [0, 90].
    """
    if reference_axis is None:
        reference_axis = np.array([0.0, 1.0, 0.0])

    direction = np.asarray(direction)
    reference_axis = np.asarray(reference_axis)

    # Normalize
    dir_norm = np.linalg.norm(direction)
    ref_norm = np.linalg.norm(reference_axis)

    if dir_norm < 1e-6 or ref_norm < 1e-6:
        return 0.0

    direction = direction / dir_norm
    reference_axis = reference_axis / ref_norm

    # Use absolute dot → orientation invariant
    cos_theta = np.clip(abs(direction @ reference_axis), -1.0, 1.0)

    angle = np.degrees(np.arccos(cos_theta))

    return angle
