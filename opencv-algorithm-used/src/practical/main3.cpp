#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <vector>

using namespace cv;
using namespace std;

int main() {
    // 1. 读取图像
    Mat img = imread("../resources/12.png");
    if (img.empty()) {
        cout << "无法读取图像！" << endl;
        return -1;
    }

    Mat result = img.clone();

    // 2. 预处理：灰度化 + 高斯模糊
    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(5, 5), 0);

    // 3. 提取高亮区域（固定阈值）
    Mat brightMask;
    threshold(gray, brightMask, 220, 255, THRESH_BINARY); // 灰度值 > 220 的为白色

    // 4. 提取蓝色区域（HSV）
    Mat hsv;
    cvtColor(img, hsv, COLOR_BGR2HSV);
    Scalar lowerBlue(90, 150, 150);
    Scalar upperBlue(140, 255, 255);
    Mat blueMask;
    inRange(hsv, lowerBlue, upperBlue, blueMask);

    // 5. 组合掩码：高亮 OR 蓝色
    Mat combinedMask = brightMask | blueMask;

    // 6. 形态学操作：开运算去噪 + 闭运算连接
    Mat kernel = getStructuringElement(MORPH_RECT, Size(7, 7));
    morphologyEx(combinedMask, combinedMask, MORPH_OPEN, kernel);   // 去噪
    morphologyEx(combinedMask, combinedMask, MORPH_CLOSE, kernel);  // 连接断开的区域

    // 7. 查找轮廓
    vector<vector<Point>> contours;
    findContours(combinedMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 8. 筛选轮廓：多特征联合筛选
    vector<RotatedRect> valid_rects;
    for (const auto& contour : contours) {
        double area = contourArea(contour);
        if (area < 50 ) continue; // 面积范围

        RotatedRect min_rect = minAreaRect(contour);

        // 长宽比筛选（1.5 ~ 5）
        double w = min_rect.size.width;
        double h = min_rect.size.height;
        double aspect_ratio = max(w, h) / min(w, h);
        if (aspect_ratio < 1.5 || aspect_ratio > 5) continue;

        // 方向筛选（45 ~ 135 度）
        if (abs(min_rect.angle) < 45 || abs(min_rect.angle) > 135) continue;

        // 紧密度筛选（面积 / 外接矩形面积 > 0.5）
        double rect_area = w * h;
        double compactness = area / rect_area;
        if (compactness < 0.3) continue;

        valid_rects.push_back(min_rect);
    }

    // 9. 寻找“最匹配”的一对灯柱
    if (valid_rects.size() >= 2) {
        double best_distance = 10000;
        int best_i = -1, best_j = -1;

        for (size_t i = 0; i < valid_rects.size(); ++i) {
            for (size_t j = i + 1; j < valid_rects.size(); ++j) {
                Point2f center1 = valid_rects[i].center;
                Point2f center2 = valid_rects[j].center;

                double distance = norm(center1 - center2);
                double y_diff = abs(center1.y - center2.y);

                // Y轴对齐筛选
                if (y_diff > 100) continue;

                // 选择距离最近的一对
                if (distance < best_distance) {
                    best_distance = distance;
                    best_i = i;
                    best_j = j;
                }
            }
        }

        if (best_i != -1 && best_j != -1) {
            // 10. 计算装甲板区域（包围两个灯柱的矩形）
            Point2f pts1[4], pts2[4];
            valid_rects[best_i].points(pts1); // 获取灯柱1的4个顶点
            valid_rects[best_j].points(pts2); // 获取灯柱2的4个顶点

            // 将两个灯柱的8个顶点合并
            vector<Point2f> all_points;
            for (int k = 0; k < 4; ++k) {
                all_points.push_back(pts1[k]);
                all_points.push_back(pts2[k]);
            }

            // 计算包围所有点的最小外接矩形（即装甲板区域）
            Rect armorBox = boundingRect(all_points);

            // 11. 绘制结果
            rectangle(result, armorBox, Scalar(0, 255, 0), 2); // 装甲板框
            drawContours(result, contours, best_i, Scalar(0, 0, 255), 2); // 灯柱1
            drawContours(result, contours, best_j, Scalar(0, 0, 255), 2); // 灯柱2
        }
    }

    // 12. 显示结果
    //imshow("原图", img);
    imshow("bright", brightMask);
    imshow("blue", blueMask);
    imshow("combined", combinedMask);
    imshow("result", result);
    waitKey(0);

    // 13. 保存结果
    imwrite("build/result_armor_final.png", result);

    cout << "装甲板识别完成！" << endl;
    return 0;
}