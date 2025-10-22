#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace cv;
using namespace std;

int main() {
    VideoCapture cap("../resources/video.mp4");
    if (!cap.isOpened()) {
        cout << "无法打开视频文件！" << endl;
        return -1;
    }

    Mat kernel3 = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat kernel7 = getStructuringElement(MORPH_RECT, Size(7, 7));

    while (true) {
        Mat img;
        cap >> img;
        if (img.empty()) break;

        // --- OpenCV 装甲板识别逻辑 ---
        Mat gray;
        cvtColor(img, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, gray, Size(5, 5), 0);

        Mat brightMask;
        threshold(gray, brightMask, 180, 255, THRESH_BINARY);
        morphologyEx(brightMask, brightMask, MORPH_OPEN, kernel7);
        morphologyEx(brightMask, brightMask, MORPH_CLOSE, kernel7);
        morphologyEx(brightMask, brightMask, MORPH_DILATE, kernel7);
        morphologyEx(brightMask, brightMask, MORPH_DILATE, kernel7);

        Mat hsv;
        cvtColor(img, hsv, COLOR_BGR2HSV);
        Scalar lowerBlue(160, 200, 200), upperBlue(180, 255, 255);
        Mat blueMask;
        inRange(hsv, lowerBlue, upperBlue, blueMask);

        Mat combinedMask = brightMask;

        vector<vector<Point>> contours;
        findContours(combinedMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        vector<RotatedRect> valid_rects;
        for (const auto& contour : contours) {
            double area = contourArea(contour);
            if (area < 50) continue;
            RotatedRect min_rect = minAreaRect(contour);
            double w = min_rect.size.width, h = min_rect.size.height;
            double aspect_ratio = max(w, h) / min(w, h);
            if (aspect_ratio < 1.5 || aspect_ratio > 7.0) continue;
            if (abs(min_rect.angle) < 30 || abs(min_rect.angle) > 150) continue;
            double rect_area = w * h;
            double compactness = area / rect_area;
            if (compactness < 0.3) continue;
            valid_rects.push_back(min_rect);
        }

        RotatedRect armor_rrect;
        bool armor_found = false;
        if (valid_rects.size() >= 2) {
            double best_distance = 1e9; int best_i = -1, best_j = -1;
            for (size_t i = 0; i < valid_rects.size(); ++i) {
                for (size_t j = i + 1; j < valid_rects.size(); ++j) {
                    double d = norm(valid_rects[i].center - valid_rects[j].center);
                    if (d < best_distance) { best_distance = d; best_i = (int)i; best_j = (int)j; }
                }
            }
            if (best_i != -1 && best_j != -1) {
                Point2f pts1[4], pts2[4];
                valid_rects[best_i].points(pts1);
                valid_rects[best_j].points(pts2);
                vector<Point2f> all_points;
                for (int k = 0; k < 4; ++k) { all_points.push_back(pts1[k]); all_points.push_back(pts2[k]); }
                armor_rrect = minAreaRect(all_points);

                if (armor_rrect.size.width < armor_rrect.size.height) {
                    armor_rrect.size.width *= 2.0f;
                } else {
                    armor_rrect.size.height *= 2.0f;
                }
                armor_found = true;
            }
        }

        // 1. 预处理调试图
        Mat mask_vis;
        cvtColor(combinedMask, mask_vis, COLOR_GRAY2BGR);
        for (const auto& rect : valid_rects) {
            Point2f pts[4]; rect.points(pts);
            for (int i = 0; i < 4; ++i)
                line(mask_vis, pts[i], pts[(i+1)%4], Scalar(255, 0, 0), 1);
        }
        if (armor_found) {
            Point2f rpts[4]; armor_rrect.points(rpts);
            for (int i = 0; i < 4; ++i)
                line(mask_vis, rpts[i], rpts[(i+1)%4], Scalar(0, 255, 0), 4);
        }
        imshow("armor_preprocessed_cpp", mask_vis);


        // 2. 裁剪后的 20x28 canvas，并用PyTorch模型识别，结果画在原图上
        Mat result = img.clone();

        std::string pred_text = "";
        if (armor_found) {
            Mat masked = img.clone();
            for (const auto &r : valid_rects) {
                RotatedRect er = r;
                er.size.width *= 1.5f;
                er.size.height *= 1.5f;
                Point2f bpts[4]; er.points(bpts);
                vector<Point> poly(4);
                for (int i = 0; i < 4; ++i) poly[i] = bpts[i];
                fillConvexPoly(masked, poly, Scalar(0, 0, 0));
            }

            Point2f src_pts[4]; armor_rrect.points(src_pts);
            int w_box = max(1, static_cast<int>(round(armor_rrect.size.width)));
            int h_box = max(1, static_cast<int>(round(armor_rrect.size.height)));
            vector<Point2f> dst_pts = {
                {0.f, 0.f},
                {static_cast<float>(w_box - 1), 0.f},
                {static_cast<float>(w_box - 1), static_cast<float>(h_box - 1)},
                {0.f, static_cast<float>(h_box - 1)}
            };
            Mat M = getPerspectiveTransform(src_pts, dst_pts.data());
            Mat armor_warp;
            warpPerspective(masked, armor_warp, M, Size(w_box, h_box));

            Mat gray2; cvtColor(armor_warp, gray2, COLOR_BGR2GRAY);
            double mean_val = gray2.empty() ? 127.0 : mean(gray2)[0];
            int thr = max(1, min(254, static_cast<int>(round(mean_val))));
            Mat bin_img; threshold(gray2, bin_img, thr, 255, THRESH_BINARY);
            Mat k3 = getStructuringElement(MORPH_RECT, Size(3, 3));
            morphologyEx(bin_img, bin_img, MORPH_OPEN, k3);
            morphologyEx(bin_img, bin_img, MORPH_CLOSE, k3);

                const int target_h = 28, target_w = 20;
            int rh = bin_img.rows, rw = bin_img.cols;
            float scale = (rh > 0 && rw > 0) ? min(static_cast<float>(target_h) / rh,
                                                  static_cast<float>(target_w) / rw) : 1.f;
            int new_w = max(1, static_cast<int>(round(rw * scale)));
            int new_h = max(1, static_cast<int>(round(rh * scale)));
            Mat resized; resize(bin_img, resized, Size(new_w, new_h), 0, 0, INTER_AREA);
            Mat canvas = Mat::zeros(target_h, target_w, CV_8UC1);
            int top = (target_h - new_h) / 2;
            int left = (target_w - new_w) / 2;
            resized.copyTo(canvas(Rect(left, top, new_w, new_h)));

            // canvas已是28x20（高×宽），直接用于显示和推理
            imshow("armor_crop_canvas", canvas);

            int pred = -1;
            FILE* fp = popen("python3 ../predict.py --from-stdin", "w+");
            if (fp) {
                fwrite(canvas.data, 1, canvas.total(), fp);
                fflush(fp);
                char buf[32] = {0};
                rewind(fp);
                if (fgets(buf, sizeof(buf), fp)) {
                    pred = atoi(buf);
                }
                pclose(fp);
            }
            // if (pred >= 0) {
            //     pred_text = std::to_string(pred);
            // } else {
            //     pred_text = "?";
            // }
            // std::cout << "预测类别: " << pred_text << std::endl;

            // 在原图上画绿色框和类别
            Point2f rpts[4]; armor_rrect.points(rpts);
            for (int i = 0; i < 4; ++i)
                line(result, rpts[i], rpts[(i+1)%4], Scalar(0, 255, 0), 4);
            // 在框左上角写类别
            // int font = FONT_HERSHEY_SIMPLEX;
            // Point text_org = rpts[1]; // 取左上角
            // putText(result, pred_text, text_org, font, 1.2, Scalar(0, 255, 0), 3);
        } else {
            Mat blank = Mat::zeros(20, 28, CV_8UC1);
            imshow("armor_crop_canvas", blank);
        }

        // 显示原图+识别结果
        imshow("armor_result", result);

        char key = (char)waitKey(30);
        if (key == 'q' || key == 27) break;
    }

    //imshow("combined", combinedMask);

    cap.release();
    destroyAllWindows();
    cout << "视频处理完成！" << endl;
    return 0;
}