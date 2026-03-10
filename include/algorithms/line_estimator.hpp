class LineEstimator {
public:
    static DiscreteLinePose estimate_discrete_line_pose(uint16_t left_val, uint16_t right_val);
private:
    enum class DiscreteLinePose {
        LineOnLeft,
        LineOnRight,
        LineNone,
        LineBoth,
    };
    float threshold_l = 0.01;
    float threshold_r = 0.01;
};
