#include "vertex_detector/nms_filter.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>
#include <unordered_map>

namespace vertex_detector
{

// ============================================================================
// KONSTRUKTOR
// ============================================================================
NMSFilter::NMSFilter(float suppression_radius)
  : suppression_radius_(suppression_radius)
  , radius_squared_(suppression_radius * suppression_radius)
{
  workspace_.reserve(2048);
  suppressed_cache_.reserve(2048);
  indices_cache_.reserve(2048);
}

// ============================================================================
// SPATIAL HASHING GRID - ZOPTYMALIZOWANY
// ============================================================================
std::vector<Vertex> NMSFilter::filter(const std::vector<Vertex>& vertices) const
{
  if (vertices.empty()) return {};
  if (vertices.size() == 1) return vertices;

  const float cell_size = suppression_radius_ * 2.0f;
  
  float min_x = vertices[0].x, max_x = vertices[0].x;
  float min_y = vertices[0].y, max_y = vertices[0].y;
  
  for (const auto& v : vertices) {
    min_x = std::min(min_x, v.x);
    max_x = std::max(max_x, v.x);
    min_y = std::min(min_y, v.y);
    max_y = std::max(max_y, v.y);
  }
  
  int grid_w = static_cast<int>((max_x - min_x) / cell_size) + 1;
  int grid_h = static_cast<int>((max_y - min_y) / cell_size) + 1;
  
  grid_cache_.clear();
  grid_cache_.reserve(vertices.size() / 4);
  
  indices_cache_.resize(vertices.size());
  std::iota(indices_cache_.begin(), indices_cache_.end(), static_cast<size_t>(0));
  
  std::sort(indices_cache_.begin(), indices_cache_.end(),
    [&vertices](size_t a, size_t b) {
      return vertices[a].score > vertices[b].score;
    });
  
  for (size_t idx : indices_cache_) {
    const auto& v = vertices[idx];
    int gx = static_cast<int>((v.x - min_x) / cell_size);
    int gy = static_cast<int>((v.y - min_y) / cell_size);
    int grid_key = gy * grid_w + gx;
    grid_cache_[grid_key].push_back(idx);
  }
  
  if (suppressed_cache_.size() < vertices.size()) {
    suppressed_cache_.resize(vertices.size());
  }
  std::fill_n(suppressed_cache_.begin(), vertices.size(), 0);
  
  workspace_.clear();
  workspace_.reserve(vertices.size() / 2);
  
  for (size_t i : indices_cache_) {
    if (suppressed_cache_[i]) continue;
    
    const auto& v_i = vertices[i];
    workspace_.push_back(v_i);
    
    int gx = static_cast<int>((v_i.x - min_x) / cell_size);
    int gy = static_cast<int>((v_i.y - min_y) / cell_size);
    
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        int nx = gx + dx;
        int ny = gy + dy;
        
        if (nx < 0 || nx >= grid_w || ny < 0 || ny >= grid_h) continue;
        
        int neighbor_key = ny * grid_w + nx;
        auto it = grid_cache_.find(neighbor_key);
        if (it == grid_cache_.end()) continue;
        
        for (size_t j : it->second) {
          if (suppressed_cache_[j] || j <= i) continue;
          
          const auto& v_j = vertices[j];
          float dx_val = v_i.x - v_j.x;
          float dy_val = v_i.y - v_j.y;
          float dist_sq = dx_val * dx_val + dy_val * dy_val;
          
          if (dist_sq < radius_squared_) {
            suppressed_cache_[j] = 1;
          }
        }
      }
    }
  }
  
  return workspace_;
}

// ============================================================================
// Fixed Grid - ZOPTYMALIZOWANY
// ============================================================================
std::vector<Vertex> NMSFilter::filterGridBased(const std::vector<Vertex>& vertices) const
{
  if (vertices.empty()) return {};
  if (vertices.size() == 1) return vertices;

  const int GRID_SIZE = 64; 
  const float INV_CELL_SIZE = GRID_SIZE / 2048.0f;  
  
  struct Cell {
    std::vector<size_t> indices;
    Cell() { indices.reserve(4); }
  };
  
  std::vector<Cell> grid(GRID_SIZE * GRID_SIZE);
  
  indices_cache_.resize(vertices.size());
  std::iota(indices_cache_.begin(), indices_cache_.end(), static_cast<size_t>(0));
  
  std::sort(indices_cache_.begin(), indices_cache_.end(),
    [&vertices](size_t a, size_t b) { 
      return vertices[a].score > vertices[b].score; 
    });
  
  workspace_.clear();
  workspace_.reserve(vertices.size() / 2);
  
  for (size_t idx : indices_cache_) {
    const auto& v = vertices[idx];
    
    int gx = std::min(std::max(0, static_cast<int>(v.x * INV_CELL_SIZE)), GRID_SIZE - 1);
    int gy = std::min(std::max(0, static_cast<int>(v.y * INV_CELL_SIZE)), GRID_SIZE - 1);
    
    bool is_suppressed = false;
    
    for (int dy = -1; dy <= 1 && !is_suppressed; ++dy) {
      for (int dx = -1; dx <= 1 && !is_suppressed; ++dx) {
        int nx = gx + dx;
        int ny = gy + dy;
        if (nx < 0 || nx >= GRID_SIZE || ny < 0 || ny >= GRID_SIZE) continue;
        
        const auto& cell = grid[ny * GRID_SIZE + nx];
        for (size_t other_idx : cell.indices) {
          const auto& other = vertices[other_idx];
          float dx_val = v.x - other.x;
          float dy_val = v.y - other.y;
          
          if (dx_val * dx_val + dy_val * dy_val < radius_squared_) {
            is_suppressed = true;
            break;
          }
        }
      }
    }
    
    if (!is_suppressed) {
      workspace_.push_back(v);
      grid[gy * GRID_SIZE + gx].indices.push_back(idx);
    }
  }
  
  return workspace_;
}

// ============================================================================
// SELECT TOP-K 
// ============================================================================
std::vector<Vertex> NMSFilter::selectTopK(
  const std::vector<Vertex>& vertices,
  int k) const
{
  if (k <= 0) return {};
  if (static_cast<int>(vertices.size()) <= k) return vertices;
  
  std::vector<Vertex> result = vertices;
  
  auto mid = result.begin() + std::min(k, static_cast<int>(result.size()));
  std::partial_sort(result.begin(), mid, result.end(),
    [](const Vertex& a, const Vertex& b) {
      return a.score > b.score;
    });
  
  result.resize(k);
  return result;
}

} // namespace vertex_detector