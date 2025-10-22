#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    Mat img = imread("../resources/12.png");
    if (img.empty()) {
        cout << "无法读取图像！" << endl;
        return -1;
    }

    //转灰度;高斯模糊
    Mat blurred;
    GaussianBlur(img, blurred, Size(5, 5), 0);
    //imshow("blurred", blurred);

    Mat gray;
    cvtColor(blurred, gray, COLOR_BGR2GRAY);

    // 自适应阈值二值化（处理光照不均）
    Mat binary;
    adaptiveThreshold(gray, binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY_INV, 11, 2); 
    // 使用 _INV 因为我们要提取白色
    imshow("binary1",binary);
    //Canny
    //Mat edges;
    //Canny(blurred, edges, 50, 150); // 阈值可根据图像调整

    //imshow("edges", edges);
    /*
    Mat hsv, blueMask;
    cvtColor(img, hsv, COLOR_BGR2HSV);
    // 蓝色范围（根据你的图片调整）
    Scalar lowerBlue(100, 150, 100), upperBlue(130, 255, 255);
    inRange(hsv, lowerBlue, upperBlue, blueMask);
    */
    
    Mat kernel1 = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat kernel2 = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(binary, binary, MORPH_OPEN, kernel1); // 开运算去噪
    //imshow("binary_open",binary);

    morphologyEx(binary, binary, MORPH_CLOSE, kernel1); // 闭运算填
    imshow("binary2",binary);

    // 查找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);


    // 筛选轮廓：增加多重条件
    Mat result = img.clone();
    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area < 500) continue;

        Rect bbox = boundingRect(contours[i]);
        double aspect_ratio = (double)bbox.width / bbox.height;

        // 筛选,长宽比在0.8~1.5之间
        if (aspect_ratio < 0.8 || aspect_ratio > 1.5) continue;

        // 矩形度（轮廓面积 / 外接矩形面积）
        //double rect_area = bbox.width * bbox.height;
        //double contour_ratio = area / rect_area;
        //if (contour_ratio < 0.6) continue;
        
        // 凸包面积比
        vector<Point> hull;
        convexHull(contours[i], hull);
        double hull_area = contourArea(hull);
        double hull_ratio = area / hull_area;
        if (hull_ratio < 0.7) continue;
        
        // 最小外接矩形角度（接近0度）
        RotatedRect min_rect = minAreaRect(contours[i]);
        if (abs(min_rect.angle) > 30) continue;

        // 内部白色像素比例（可选）
        //Mat mask_roi = binary(bbox);
        //double white_ratio = countNonZero(mask_roi) / (double)(bbox.width * bbox.height);
        //if (white_ratio < 0.8) continue;

        // 绘制矩形框
        rectangle(result, bbox, Scalar(0, 255, 0), 2);
    }

    imshow("result", result);
    waitKey(0);


    return 0;
}