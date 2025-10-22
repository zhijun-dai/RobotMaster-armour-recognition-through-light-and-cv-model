#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <vector>

using namespace cv;
using namespace std;

int main() {
    // 打开视频文件
    VideoCapture cap("../resources/video.mp4"); // 替换为你自己的视频路径
    if (!cap.isOpened()) {
        cout << "无法打开视频文件！" << endl;
        return -1;
    }

    Mat img;
    Mat kernel3 = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat kernel5 = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat kernel7 = getStructuringElement(MORPH_RECT, Size(7, 7));
    Mat kernel9 = getStructuringElement(MORPH_RECT, Size(9, 9));

    while (true) {
        cap >> img; // 逐帧读取
        if (img.empty()) break; // 视频结束

        Mat result = img.clone(); // 每帧都创建一个新的结果图

        // 预处理：灰度化 + 高斯模糊
        Mat gray;
        cvtColor(img, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, gray, Size(5, 5), 0);

        // 提取高亮区域（固定阈值）
        Mat brightMask;
        threshold(gray, brightMask, 220, 255, THRESH_BINARY); // 灰度值 > 220 的为白色
        morphologyEx(brightMask, brightMask, MORPH_OPEN, kernel7);
        morphologyEx(brightMask, brightMask, MORPH_CLOSE, kernel7);
        morphologyEx(brightMask, brightMask, MORPH_DILATE, kernel7);
        morphologyEx(brightMask, brightMask, MORPH_DILATE, kernel7);

        // 提取蓝色区域（HSV）
        Mat hsv;
        //H-色调 S-饱和度 V-亮度
        cvtColor(img, hsv, COLOR_BGR2HSV);
        //Scalar lowerBlue(90, 150, 150);
        //Scalar upperBlue(140, 255, 255);
        Scalar lowerBlue(160, 200, 200);
        Scalar upperBlue(180, 255, 255);
        Mat blueMask;
        inRange(hsv, lowerBlue, upperBlue, blueMask);
        /*
        morphologyEx(blueMask, blueMask, MORPH_OPEN, kernel7);
        morphologyEx(blueMask, blueMask, MORPH_CLOSE, kernel7);
        morphologyEx(blueMask, blueMask, MORPH_DILATE, kernel9);
        morphologyEx(blueMask, blueMask, MORPH_DILATE, kernel9);
        morphologyEx(blueMask, blueMask, MORPH_DILATE, kernel7);
        */
        imshow("bright", brightMask);
        imshow("blue", blueMask);

        // 组合掩码：先 AND 得到核心，再膨胀扩展
        Mat andMask = brightMask & blueMask;
        morphologyEx(andMask, andMask, MORPH_OPEN, kernel3);
        morphologyEx(andMask, andMask, MORPH_CLOSE, kernel3);

        Mat orMask = brightMask | blueMask;
        morphologyEx(andMask, orMask, MORPH_OPEN, kernel3);
        morphologyEx(andMask, orMask, MORPH_CLOSE, kernel3);

        imshow("and", andMask);
        imshow("or", orMask);

        Mat combinedMask = brightMask; // 使用膨胀后的核心作为最终掩码
        //最终掩码是combinedMask
        // 查找轮廓
        vector<vector<Point>> contours;
        findContours(combinedMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // 筛选轮廓：多特征联合筛选
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

        // 寻找“最匹配”的一对灯柱
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
                // 计算装甲板区域（包围两个灯柱的矩形）
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

                // 绘制结果
                rectangle(result, armorBox, Scalar(0, 255, 0), 2); // 装甲板框
            }
        }

        // 显示结果
        imshow("original", img);
        imshow("combined", combinedMask);
        imshow("result", result);

        // 按 'q' 键退出
        char key = (char)waitKey(30); // 30ms 延迟，约 33fps
        if (key == 'q' || key == 27) break; // ESC 或 q 退出
    }

    cap.release(); // 释放视频捕获对象
    destroyAllWindows(); // 关闭所有窗口

    cout << "视频处理完成！" << endl;
    return 0;
}