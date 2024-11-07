#include "plr.hpp"

int64_t num_x = 0;
int test_num = 20000;
int64_t Epsilon = 3;

int64_t get_x()
{
    int64_t num;
    num = num_x * num_x + num_x;
    num_x++;
    return num;
}

struct model
{
    double slope;
    double intercept;
    int64_t max_x;
    int64_t min_x;
    int64_t max_y;
    int64_t min_y;

    model(double s, double i, int64_t maxX, int64_t minX, int64_t maxY, int64_t minY)
        : slope(s), intercept(i), max_x(maxX), min_x(minX), max_y(maxY), min_y(minY) {}
};

std::vector<struct model> models;

int64_t search_model(int64_t x)
{
    int left = 0;
    int right = models.size() - 1;

    if (right == 0)
        return 0;

    while (left <= right)
    {
        int mid = (right + left) / 2;
        if (models[mid].min_x <= x && x <= models[mid].max_x)
            return mid;
        else if (x < models[mid].min_x)
            right = mid - 1;
        else
            left = mid + 1;
    }

    return -1;
}

int64_t search_addr(int64_t *x_s, int64_t x, int64_t down, int64_t up)
{
    while (down <= up)
    {
        int mid = (down + up) / 2;
        if (x_s[mid] == x)
            return mid;
        else if (x < x_s[mid])
            up = mid - 1;
        else if (x > x_s[mid])
            down = mid + 1;
        else
            printf("Search_value Error\n");
    }

    return -1;
}

int main()
{
    int64_t x_s[20000];
    int64_t y_s[20000];

    PLR *opt = new PLR(Epsilon);

    int64_t pos = 0;
    int64_t max_y = 0;
    int64_t max_x = 0;
    int64_t min_y = 0;
    int64_t min_x = 0;

    int64_t x_1 = get_x();
    y_s[0] = pos;
    x_s[0] = x_1;

    min_x = x_1;
    min_y = pos;

    opt->add_point(x_1, pos);

    pos++;

    for (; num_x < test_num;)
    {
        int64_t x = get_x();

        x_s[num_x - 1] = x;
        y_s[num_x - 1] = pos;

        if (!opt->add_point(x, pos))
        {
            auto cs = opt->get_segment();

            std::pair<double, double> param = cs.get_slope_intercept();
            auto cs_slope = param.first;
            auto cs_intercept = param.second;

            printf("cs_slope is %f, cs_intercept is %f, max_x is %ld, min_x is %ld,max_y is %ld, min_y is %ld\n",
                   cs_slope, cs_intercept, max_x, min_x, max_y, min_y);

            models.emplace_back(cs_slope, cs_intercept, max_x, min_x, max_y, min_y);
            opt->reset();

            min_x = x;
            min_y = pos;

            opt->add_point(x, pos);
        }
        max_x = x;
        max_y = pos;
        pos++;
    }
    auto cs = opt->get_segment();

    std::pair<double, double> param = cs.get_slope_intercept();
    auto cs_slope = param.first;
    auto cs_intercept = param.second;

    printf("cs_slope is %f, cs_intercept is %f, max_x is %ld, min_x is %ld,max_y is %ld, min_y is %ld\n",
           cs_slope, cs_intercept, max_x, min_x, max_y, min_y);
    models.emplace_back(cs_slope, cs_intercept, max_x, min_x, max_y, min_y);

    delete opt;

    printf("--------------------------------------\n\n");

    printf("models.size() is %ld\n", models.size());

    for (int i = 0; i < models.size(); i++)
        printf("cs_slope is %f, cs_intercept is %f, max_x is %ld, min_x is %ld,max_y is %ld, min_y is %ld\n",
               models[i].slope, models[i].intercept, models[i].max_x, models[i].min_x, models[i].max_y, models[i].min_y);

    printf("--------------------------------------\n\n");

    for (int i = 0; i < test_num; i++)
    {
        int up;
        int down;
        int64_t search_x = x_s[i];

        int model_num = search_model(search_x);

        // printf("search x %ld, model num is %d\n", search_x, model_num);

        double p = models[model_num].slope * search_x + models[model_num].intercept;

        if (p + Epsilon > models[model_num].max_y)
            up = models[model_num].max_y;
        else
            up = (int64_t)(p + Epsilon);

        if (p - Epsilon < models[model_num].min_y)
            down = models[model_num].min_y;
        else
            down = (int64_t)(p - Epsilon);

        // printf("down is  %d, up is %d\n", down, up);
        int addr = search_addr(x_s, search_x, down, up);

        if (addr == -1)
            printf("Error, addr = -1\n");
        else if (y_s[addr] != y_s[i])
            printf("Error i = %d, addr is %d\n", i, addr);
        else
            printf("y_s[addr] is %ld, y_s[i] is %ld, result is right!!!\n", y_s[addr], y_s[i]);
    }
}
