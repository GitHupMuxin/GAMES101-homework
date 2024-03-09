#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}


cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    int size = control_points.size();
    if (size == 1)
        return control_points[0];
    std::vector<cv::Point2f> help;
    for (int i = 1; i < size; i++)
    {
        cv::Point2f p = control_points[i - 1];
        float dX = control_points[i].x - control_points[i - 1].x;
        float dY = control_points[i].y - control_points[i - 1].y;
        p.x += dX * t;
        p.y += dY * t;
        help.push_back(p);
    }
    return recursive_bezier(help, t);
}

void setColor(const cv::Point2f point, cv::Mat &window, cv::Vec3b color)
{
    cv::Vec3b &help = window.at<cv::Vec3b>(point.y, point.x);
    float col = float(color[0]) * color[0] + float(color[1]) * color[1] + float(color[2]) * color[2];
    float win = float(help[0]) * help[0] + float(help[1]) * help[1] + float(help[2]) * help[2];
    if (col > win)
    {
        std::cout << col << " " << win << std::endl;
        window.at<cv::Vec3b>(point.y, point.x) = color;
    }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    cv::Vec3f help, color = { 0, 255, 0 };
    cv::Point2f point, p;
    float w = 1.5;
    for (float t = 0.f; t < 1.f; t += 0.001)
    {
        p = recursive_bezier(control_points, t);
        //distance algorithm anti-aliasing
        for (int i = -1; i < 2; i++)
        {
            for (int j = -1; j < 2; j++)
            {
                point.x = int(p.x) + i + 0.5;
                point.y = int(p.y) + j + 0.5;
                float times = 1 - (sqrt(2) / 3 * sqrt(pow(point.x - p.x, 2) + pow(point.y - p.y, 2)));
                setColor(point, window, color * times);
            }
        }
        //bilinear interpolation algorithm anti-aliasing
        // int i = p.x - 1, maxI = p.x + 1, maxJ = p.y + 1;
        // while (i <= maxI)
        // {
        //     int j = p.y - 1;
        //     while (j <= maxJ)
        //     {
        //         point.x = float(i) + 0.5;
        //         point.y = float(j) + 0.5;
        //         float times = (1 - abs(point.x - p.x) / 1.5f + 1 - abs(point.y - p.y) / 1.5f) / 2;
        //         setColor(point, window, color * times);
        //         j++;
        //     }
        //     i++;
        // }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }   

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
