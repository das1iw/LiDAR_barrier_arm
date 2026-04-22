#include "barrier_detection/clustering.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <boost/make_shared.hpp>

#include <deque>
#include <algorithm>

// PCL 1.10 (Ubuntu 20.04): KdTreeFLANN::setInputCloud() requires
// boost::shared_ptr — passing std::shared_ptr fails to compile.

using CloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

// ---------------------------------------------------------------------------
// DBSCAN — matches sklearn.cluster.DBSCAN(eps=0.16, min_samples=20) exactly.
// ---------------------------------------------------------------------------

std::vector<boost::shared_ptr<CloudXYZ>>
cluster_dbscan(const boost::shared_ptr<CloudXYZ> & cloud,
               float eps,
               int   min_samples)
{
    if (!cloud || cloud->empty()) {
        return {};
    }

    const int n = static_cast<int>(cloud->size());

    constexpr int UNVISITED = -2;
    constexpr int NOISE     = -1;

    std::vector<int>  labels(n, UNVISITED);
    std::vector<bool> visited(n, false);

    // ── Build KD-tree once ─────────────────────────────────────────────────
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);              // requires boost::shared_ptr

    int cluster_id = 0;

    // ── Core DBSCAN loop ───────────────────────────────────────────────────
    for (int i = 0; i < n; ++i) {
        if (visited[i]) { continue; }
        visited[i] = true;

        std::vector<int>   neighbours;
        std::vector<float> sq_dists;
        tree.radiusSearch(cloud->points[i], eps, neighbours, sq_dists);

        if (static_cast<int>(neighbours.size()) < min_samples) {
            labels[i] = NOISE;
            continue;
        }

        // Start a new cluster
        labels[i] = cluster_id;

        // BFS expansion
        std::deque<int> queue(neighbours.begin(), neighbours.end());

        while (!queue.empty()) {
            const int q = queue.front();
            queue.pop_front();

            if (!visited[q]) {
                visited[q] = true;

                std::vector<int>   q_nb;
                std::vector<float> q_sq;
                tree.radiusSearch(cloud->points[q], eps, q_nb, q_sq);

                if (static_cast<int>(q_nb.size()) >= min_samples) {
                    for (int nb : q_nb) {
                        if (!visited[nb]) { queue.push_back(nb); }
                    }
                }
            }

            if (labels[q] == UNVISITED || labels[q] == NOISE) {
                labels[q] = cluster_id;
            }
        }

        ++cluster_id;
    }

    // ── Collect into per-cluster clouds ───────────────────────────────────
    std::vector<boost::shared_ptr<CloudXYZ>> clusters(cluster_id);
    for (auto & c : clusters) {
        c = boost::make_shared<CloudXYZ>();
    }

    for (int i = 0; i < n; ++i) {
        if (labels[i] >= 0) {
            clusters[labels[i]]->points.push_back(cloud->points[i]);
        }
    }

    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
                       [](const boost::shared_ptr<CloudXYZ> & c) {
                           return !c || c->empty();
                       }),
        clusters.end());

    return clusters;
}
