# clustering.py
from sklearn.cluster import DBSCAN

def cluster(points):
    model = DBSCAN(eps=0.16, min_samples=20)
    labels = model.fit_predict(points[:, :3])

    clusters = []
    for l in set(labels):
        if l == -1:
            continue
        clusters.append(points[labels == l])

    return clusters
    
    