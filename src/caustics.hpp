// caustics.hpp - photon mapping with k-d tree for pencil
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "material.hpp"
#include "primitives.hpp"
#include "ray.hpp"
#include "rng.hpp"
#include "vec3.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace pencil {

struct Photon {
 Vec3 position;
 Vec3 direction;
 Color power;
 int bounces = 0;
};

// k-d tree node for spatial photon lookup
struct KDNode {
 Photon photon;
 int axis = 0;
 int left = -1;
 int right = -1;
};

class PhotonMap {
public:
 std::vector<Photon> photons;
 std::vector<KDNode> nodes;
 int root = -1;
 bool built = false;

 void add(const Photon& p) { photons.push_back(p); }

 void build() {
 if (photons.empty()) { built = true; return; }
 nodes.resize(photons.size());
 root = build_recursive(0, (int)photons.size(), 0);
 built = true;
 }

 // radius search using the k-d tree. O(log N) average.
 Color lookup(const Vec3& p, float radius) const {
 if (!built || nodes.empty()) return {0, 0, 0};
 float r2 = radius * radius;
 std::vector<std::pair<float, const Photon*>> results;
 knn_search(root, p, r2, results);
 Color result = {0, 0, 0};
 int count = 0;
 for (auto& [dist2, ph] : results) {
 float w = 1.0f - std::sqrt(dist2) / radius;
 result += ph->power * w;
 count++;
 }
 if (count == 0) return {0, 0, 0};
 float area = 3.14159265f * r2;
 return result / area;
 }

 size_t size() const { return photons.size(); }

private:
 int build_recursive(int start, int end, int depth) {
 if (start >= end) return -1;
 int axis = depth % 3;
 int mid = (start + end) / 2;
 std::nth_element(photons.begin() + start, photons.begin() + mid,
 photons.begin() + end,
 [axis](const Photon& a, const Photon& b) {
 return a.position[axis] < b.position[axis];
 });
 int idx = mid;
 nodes[idx].photon = photons[mid];
 nodes[idx].axis = axis;
 nodes[idx].left = build_recursive(start, mid, depth + 1);
 nodes[idx].right = build_recursive(mid + 1, end, depth + 1);
 return idx;
 }

 void knn_search(int node_idx, const Vec3& p, float r2,
 std::vector<std::pair<float, const Photon*>>& results) const {
 if (node_idx < 0 || node_idx >= (int)nodes.size()) return;
 const KDNode& node = nodes[node_idx];
 Vec3 d = node.photon.position - p;
 float dist2 = d.dot(d);
 if (dist2 < r2) {
 results.push_back({dist2, &node.photon});
 }
 int axis = node.axis;
 float diff = p[axis] - node.photon.position[axis];
 float diff2 = diff * diff;
 if (diff < 0) {
 knn_search(node.left, p, r2, results);
 if (diff2 < r2) knn_search(node.right, p, r2, results);
 } else {
 knn_search(node.right, p, r2, results);
 if (diff2 < r2) knn_search(node.left, p, r2, results);
 }
 }
};

} // namespace pencil
