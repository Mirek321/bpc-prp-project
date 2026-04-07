#include <cmath>
#include <vector>
#include <numeric>
#include <limits>

namespace algorithms {

    struct LidarFilterResults {
        float front;   
        float back;    
        float left;   
        float right;   
    };

    class LidarFilter {
    public:
        LidarFilter() = default;

        LidarFilterResults apply_filter(
            const std::vector<float>& points,  
            float angle_start,                  
            float angle_increment,            
            float range_min,                    
            float range_max                    
        ) {
            std::vector<float> left, right, front, back;
            
            constexpr float SECTOR_WIDTH = M_PI_4;

           for (size_t i = 0; i < points.size(); ++i) {
                float range = points[i];
                
                float angle = angle_start + static_cast<float>(i) * angle_increment;

                if (!std::isfinite(range) || range < range_min || range > range_max) {
                    continue;
                }

                if (angle >= -SECTOR_WIDTH && angle <= SECTOR_WIDTH) {
                    front.push_back(range);          
                } 
                else if (angle >= M_PI_2 - SECTOR_WIDTH && angle <= M_PI_2 + SECTOR_WIDTH) {
                    left.push_back(range);            
                } 
                else if (angle >= -M_PI_2 - SECTOR_WIDTH && angle <= -M_PI_2 + SECTOR_WIDTH) {
                    right.push_back(range);          
                } 
                else {
                    back.push_back(range);        
                }
            }

            auto safe_average = [](const std::vector<float>& v, float default_val) -> float {
                if (v.empty()) return default_val;
                return std::accumulate(v.begin(), v.end(), 0.0f) / static_cast<float>(v.size());
            };

            return LidarFilterResults{
                .front = safe_average(front, 999.0f),
                .back  = safe_average(back, 999.0f),
                .left  = safe_average(left, 999.0f),
                .right = safe_average(right, 999.0f),
            };
        }
    };
}