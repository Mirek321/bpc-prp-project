#include <cmath>
#include <vector>
#include <numeric>
#include <limits>
#include <algorithm> 

namespace algorithms {

    struct LidarFilterResults {
        float front;   
        float back;    
        float left;   
        float right;
        float front_left;   // ← NOVÉ: medzi front a left
        float front_right;  // ← NOVÉ: medzi front a right
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
            std::vector<float> left, right, front, back, front_left, front_right;
            
            // Šírka hlavných sektorov (45° = π/4)
            constexpr float MAIN_SECTOR = M_PI_4;        // 45°
            // Šírka diagonálnych sektorov (22.5° = π/8)
            constexpr float DIAG_SECTOR = M_PI_4 / 2.0f;  // 22.5°

            for (size_t i = 0; i < points.size(); ++i) {
                float range = points[i];
                float angle = angle_start + static_cast<float>(i) * angle_increment;

                // Normalizácia uhla do [-π, π]
                while (angle > M_PI) angle -= 2.0f * M_PI;
                while (angle < -M_PI) angle += 2.0f * M_PI;

                if (!std::isfinite(range) || range < range_min || range > range_max) {
                    continue;
                }

                 if (angle >= -MAIN_SECTOR && angle <= MAIN_SECTOR) {
                    front.push_back(range);          
                } 


                // FRONT_RIGHT (PŘEDEK-PRAVO): -22.5° až -67.5° (medzi front a left)
                else if (angle >= M_PI_2 + DIAG_SECTOR && angle <= M_PI - DIAG_SECTOR) {
                    front_right.push_back(range);
                    
                }
                  // FRONT_LEFT: -112.5° až -157.5°
                else if (angle >= -M_PI + DIAG_SECTOR && angle <= -M_PI_2 - DIAG_SECTOR) {
                   front_left.push_back(range);
                }
                // LEFT: -67.5° až -112.5° (okolo -90°)
                else if (angle >= -M_PI_2 - DIAG_SECTOR && angle < -MAIN_SECTOR - DIAG_SECTOR) {
                    left.push_back(range);
                } 
                // RIGHT: +67.5° až +112.5° (okolo +90°)
                else if (angle > MAIN_SECTOR + DIAG_SECTOR && angle <= M_PI_2 + DIAG_SECTOR) {
                    right.push_back(range);
                } 
                // BACK: všetko ostatné (okolo ±180°)
                else {
                    back.push_back(range);
                }
            }

            auto safe_median = [](const std::vector<float>& v, float default_val) -> float {
                if (v.empty()) return default_val;
                std::vector<float> temp = v;
                size_t mid = temp.size() / 2;
                std::nth_element(temp.begin(), temp.begin() + mid, temp.end());
                return temp[mid];
            };

            return LidarFilterResults{
                .front = safe_median(front, 0.15f),
                .back  = safe_median(back, 0.15f),
                .left  = safe_median(left, 0.15f),
                .right = safe_median(right, 0.15f),
                .front_left  = safe_median(front_left, 0.15f),   // ← NOVÉ
                .front_right = safe_median(front_right, 0.15f),  // ← NOVÉ
            };
        }
    };
}