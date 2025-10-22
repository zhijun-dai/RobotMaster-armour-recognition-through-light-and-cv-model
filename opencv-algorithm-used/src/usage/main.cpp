#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // 1. 读取图片
    Mat img = imread("../resources/11.png"); // 使用相对路径
    if (img.empty()) {
        cout << "无法读取图像！" << endl;
        return -1;
    }

    // 创建一个用于显示的窗口
    imshow("img", img);

    // 2. 图像颜色空间转换
    Mat gray, hsv;
    cvtColor(img, gray, COLOR_BGR2GRAY);   // 转为灰度图
    cvtColor(img, hsv, COLOR_BGR2HSV);     // 转为 HSV 图片

    // 显示灰度图和 HSV 图
    imshow("gray", gray);
    imshow("hsv", hsv);

    // 3. 应用滤波操作
    Mat blurImg, gaussianImg;
    blur(img, blurImg, Size(5, 5));                    // 均值滤波
    GaussianBlur(img, gaussianImg, Size(5, 5), 1.5);    // 高斯滤波

    imshow("blurImg", blurImg);
    imshow("gaussianImg", gaussianImg);

    // 4. 特征提取：提取红色颜色区域 (HSV 方法)
    Mat mask_red;
    Scalar lower_red(0, 50, 50);   // 红色在 HSV 中的下限
    Scalar upper_red(10, 255, 255); // 红色在 HSV 中的上限
    inRange(hsv, lower_red, upper_red, mask_red);

    imshow("mask_red", mask_red);

    // 5. 寻找图像中红色的外轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask_red, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 绘制轮廓和 bounding box
    Mat contourImg = Mat::zeros(img.size(), CV_8UC3);
    for (size_t i = 0; i < contours.size(); i++) {
        double area = contourArea(contours[i]);
        if (area > 500) { // 过滤小噪声
            Rect bbox = boundingRect(contours[i]);
            drawContours(contourImg, contours, (int)i, Scalar(0, 255, 0), 2);
            rectangle(img, bbox, Scalar(0, 0, 255), 2); // 用红色框出
        }
    }

    imshow("contourImg", contourImg);
    imshow("img", img); // 在原图上画了红框

    // 6. 提取亮颜色区域并进行图形学处理
    // 先对灰度图进行二值化
    Mat binary, morphed;
    threshold(gray, binary, 128, 255, THRESH_BINARY); // 全局阈值

    // 形态学操作：开运算 (先腐蚀后膨胀)，去除小噪点
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(binary, morphed, MORPH_OPEN, kernel);

    imshow("binary", binary);
    imshow("morphed", morphed);

    // 7. 图像绘制：绘制任意圆形、方形和文字
    Mat drawImg = img.clone(); // 复制一份原图用于绘制
    circle(drawImg, Point(100, 100), 50, Scalar(255, 0, 0), 3); // 蓝色圆
    rectangle(drawImg, Rect(200, 200, 100, 50), Scalar(0, 255, 0), 3); // 绿色方块
    putText(drawImg, "Hello OpenCV!", Point(300, 300), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);

    imshow("drawImg", drawImg);

    // 8. 对图像进行处理：旋转 35 度
    Point2f center(img.cols / 2.0, img.rows / 2.0);
    Mat rotMat = getRotationMatrix2D(center, 35, 1.0); // 35度，缩放1.0
    Mat rotated;
    warpAffine(img, rotated, rotMat, img.size());

    imshow("rotated", rotated);

    // 9. 图像裁剪为原图的左上角 1/4
    Mat cropped = img(Rect(0, 0, img.cols / 2, img.rows / 2));

    imshow("cropped", cropped);

    // 10. 等待按键，让所有窗口保持打开
    waitKey(0);

    return 0;
}