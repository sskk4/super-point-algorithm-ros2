#ifndef VERTEX_DETECTOR_NMS_FILTER_HPP
#define VERTEX_DETECTOR_NMS_FILTER_HPP

#include "vertex_detector/vertex.hpp"
#include <vector>
#include <unordered_map>
#include <cstddef>
#include <cstdint>

namespace vertex_detector
{

/**
 * @brief Non-Maximum Suppression filter for vertex detection
 * 
 * Optimized implementation using spatial hashing for O(N) complexity
 * instead of naive O(N²) approach.
 */
class NMSFilter
{
public:
  /**
   * @brief Constructor
   * @param suppression_radius Distance threshold for suppressing nearby vertices (in pixels)
   */
  explicit NMSFilter(float suppression_radius);
  
  /**
   * @brief Apply NMS using spatial hashing grid
   * @param vertices Input vertices to filter
   * @return Filtered vertices after NMS
   * 
   * Complexity: O(N) average case using spatial grid
   * Best for: Any number of vertices, especially > 100
   */
  std::vector<Vertex> filter(const std::vector<Vertex>& vertices) const;
  
  /**
   * @brief Apply NMS using fixed-size grid (cache-friendly)
   * @param vertices Input vertices to filter
   * @return Filtered vertices after NMS
   * 
   * Complexity: O(N) with better cache locality
   * Best for: High vertex counts (> 500)
   */
  std::vector<Vertex> filterGridBased(const std::vector<Vertex>& vertices) const;
  
  /**
   * @brief Select top K vertices by score
   * @param vertices Input vertices
   * @param k Number of top vertices to select
   * @return Top K vertices sorted by score
   * 
   * Uses partial_sort for efficiency: O(N log K) instead of O(N log N)
   */
  std::vector<Vertex> selectTopK(const std::vector<Vertex>& vertices, int k) const;

private:
  float suppression_radius_;
  float radius_squared_;
  
  
  mutable std::vector<Vertex> workspace_;
  
  mutable std::unordered_map<int, std::vector<size_t>> grid_cache_;
  mutable std::vector<uint8_t> suppressed_cache_;
  mutable std::vector<size_t> indices_cache_;
};

} // namespace vertex_detector

#endif // VERTEX_DETECTOR_NMS_FILTER_HPP