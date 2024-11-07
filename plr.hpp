#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

class PLR
{
private:
    struct Slope
    {
        int64_t dx;
        int64_t dy;

        bool operator<(const Slope &p) const { return dy * p.dx < dx * p.dy; }
        bool operator>(const Slope &p) const { return dy * p.dx > dx * p.dy; }
        bool operator==(const Slope &p) const { return dy * p.dx == dx * p.dy; }
        bool operator!=(const Slope &p) const { return dy * p.dx != dx * p.dy; }
        explicit operator double() const { return dy / (double)dx; }
    };

    struct Point
    {
        int64_t x;
        int64_t y;

        Slope operator-(const Point &p) const { return {x - p.x, y - p.y}; }
    };

    const int64_t epsilon;
    std::vector<Point> lower;
    std::vector<Point> upper;
    int64_t first_x = 0;
    int64_t last_x = 0;
    size_t lower_start = 0;
    size_t upper_start = 0;
    size_t points_in_hull = 0;
    Point rectangle[4];

    int64_t max_y = std::numeric_limits<int64_t>::max();
    int64_t min_y = std::numeric_limits<int64_t>::lowest();

    int64_t cross(const Point &O, const Point &A, const Point &B) const
    {
        auto OA = A - O;
        auto OB = B - O;
        return OA.dx * OB.dy - OA.dy * OB.dx;
    }

public:
    class CanonicalSegment
    {
        Point rectangle[4];
        int64_t first;

        bool one_point() const
        {
            return rectangle[0].x == rectangle[2].x &&
                   rectangle[0].y == rectangle[2].y &&
                   rectangle[1].x == rectangle[3].x &&
                   rectangle[1].y == rectangle[3].y;
        }

    public:
        CanonicalSegment() = default;

        CanonicalSegment(const Point &p0, const Point &p1, int64_t first)
            : rectangle{p0, p1, p0, p1}, first(first) {};

        CanonicalSegment(const Point (&rectangle)[4], int64_t first)
            : rectangle{rectangle[0], rectangle[1], rectangle[2], rectangle[3]},
              first(first) {};

        int64_t get_first_x() const { return first; }

        std::pair<double, double> get_intersection() const
        {
            auto &p0 = rectangle[0];
            auto &p1 = rectangle[1];
            auto &p2 = rectangle[2];
            auto &p3 = rectangle[3];
            auto slope1 = p2 - p0;
            auto slope2 = p3 - p1;

            if (one_point() || slope1 == slope2)
                return {p0.x, p0.y};

            auto p0p1 = p1 - p0;
            auto a = slope1.dx * slope2.dy - slope1.dy * slope2.dx;
            auto b = (p0p1.dx * slope2.dy - p0p1.dy * slope2.dx) /
                     static_cast<double>(a);
            auto i_x = p0.x + b * slope1.dx;
            auto i_y = p0.y + b * slope1.dy;
            return {i_x, i_y};
        }

        std::pair<double, double> get_slope_intercept() const
        {
            if (one_point())
                return {0, (rectangle[0].y + rectangle[1].y) / 2};

            std::pair<double, double> intersection = get_intersection();
            auto i_x = intersection.first;
            auto i_y = intersection.second;

            std::pair<double, double> slope_range = get_slope_range();
            auto min_slope = slope_range.first;
            auto max_slope = slope_range.second;

            auto slope = (min_slope + max_slope) / 2.;
            auto intercept = i_y - i_x * slope;
            return {slope, intercept};
        }

        std::pair<double, double> get_slope_range() const
        {
            if (one_point())
                return {0, 1};

            auto min_slope = static_cast<double>(rectangle[2] - rectangle[0]);
            auto max_slope = static_cast<double>(rectangle[3] - rectangle[1]);
            return {min_slope, max_slope};
        }
    };

    explicit PLR(int64_t epsilon) : epsilon(epsilon), lower(), upper()
    {
        if (epsilon < 0)
            printf("Error!! Epsilon cannot be negative\n");
        upper.reserve(1u << 16);
        lower.reserve(1u << 16);
    }

    bool add_point(const int64_t &x, const int64_t &y)
    {
        if (points_in_hull > 0 && x <= last_x)
            printf("Error!! Points must be increasing by x.\n");

        last_x = x;
        Point p1{x, y >= max_y - epsilon ? max_y : y + epsilon};
        Point p2{x, y <= min_y + epsilon ? min_y : y - epsilon};

        if (points_in_hull == 0)
        {
            first_x = x;
            rectangle[0] = p1;
            rectangle[1] = p2;
            upper.clear();
            lower.clear();
            upper.push_back(p1);
            lower.push_back(p2);
            upper_start = lower_start = 0;
            ++points_in_hull;
            return true;
        }

        if (points_in_hull == 1)
        {
            rectangle[2] = p2;
            rectangle[3] = p1;
            upper.push_back(p1);
            lower.push_back(p2);
            ++points_in_hull;
            return true;
        }

        auto slope1 = rectangle[2] - rectangle[0];
        auto slope2 = rectangle[3] - rectangle[1];
        bool outside_line1 = p1 - rectangle[2] < slope1;
        bool outside_line2 = p2 - rectangle[3] > slope2;

        if (outside_line1 || outside_line2)
        {
            points_in_hull = 0;
            return false;
        }

        if (p1 - rectangle[1] < slope2)
        {
            // Find extreme slope
            auto min = lower[lower_start] - p1;
            auto min_i = lower_start;
            for (auto i = lower_start + 1; i < lower.size(); i++)
            {
                auto val = lower[i] - p1;
                if (val > min)
                    break;
                min = val;
                min_i = i;
            }

            rectangle[1] = lower[min_i];
            rectangle[3] = p1;
            lower_start = min_i;

            // Hull update
            auto end = upper.size();
            for (; end >= upper_start + 2 &&
                   cross(upper[end - 2], upper[end - 1], p1) <= 0;
                 --end)
                continue;
            upper.resize(end);
            upper.push_back(p1);
        }

        if (p2 - rectangle[0] > slope1)
        {
            // Find extreme slope
            auto max = upper[upper_start] - p2;
            auto max_i = upper_start;
            for (auto i = upper_start + 1; i < upper.size(); i++)
            {
                auto val = upper[i] - p2;
                if (val < max)
                    break;
                max = val;
                max_i = i;
            }

            rectangle[0] = upper[max_i];
            rectangle[2] = p2;
            upper_start = max_i;

            // Hull update
            auto end = lower.size();
            for (; end >= lower_start + 2 &&
                   cross(lower[end - 2], lower[end - 1], p2) >= 0;
                 --end)
                continue;
            lower.resize(end);
            lower.push_back(p2);
        }

        ++points_in_hull;
        return true;
    }

    CanonicalSegment get_segment()
    {
        if (points_in_hull == 1)
            return CanonicalSegment(rectangle[0], rectangle[1], first_x);
        return CanonicalSegment(rectangle, first_x);
    }

    void reset()
    {
        points_in_hull = 0;
        lower.clear();
        upper.clear();
    }
};
