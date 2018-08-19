#include<string>
#include<iostream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <exception>
using namespace std;
using namespace cv;
using namespace rs2;

bool selectObject = false;
cv::Point origin;
cv::Rect selection;
cv::Mat src;
cv::Mat srcMarks;
cv::Mat result;
cv::Mat foreground;
cv::Mat surImgColor;

void GrabCutSegment(){
    cv::Mat bgModel, fgModel;
    cv::grabCut(src, result, selection, bgModel, fgModel, 5, cv::GC_INIT_WITH_RECT);
    cv::compare(result, cv::GC_PR_FGD, result, cv::CMP_EQ);
    // Generate output image
    foreground = cv::Mat::ones(src.size(), CV_8UC3);

    src.copyTo(foreground, result); // bg pixels not copied
    cv::namedWindow("segment");
    cv::imshow("segment", foreground);
 //   Mat foreground2 = cv::Mat::ones(surImgColor.size(), CV_8UC3);
//    surImgColor.copyTo(foreground2,result);
//    cv::namedWindow("segments");
//    cv::imshow("segments", foreground2);
}

void onMouse(int event, int x, int y, int, void*)
{
    if (selectObject)
    {
        selection.x = MIN(x, origin.x);
        selection.y = MIN(y, origin.y);
        selection.width = std::abs(x - origin.x);
        selection.height = std::abs(y - origin.y);
        selection &= cv::Rect(0, 0, src.cols, src.rows);
    }
    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN:
        origin = cv::Point(x, y);
        selection = cv::Rect(x, y, 0, 0);
        selectObject = true;
        break;
    case cv::EVENT_LBUTTONDOWN && cv::EVENT_MOUSEMOVE:
        src.copyTo(srcMarks);
        cv::rectangle(srcMarks, selection, cv::Scalar(255, 0, 0),5);
        imshow("srcMarks", srcMarks);
        break;
    case cv::EVENT_LBUTTONUP:
        selectObject = false;
        if (selection.width != 0 && selection.height != 0){
          // GrabCutSegment();
        }
        break;
    }
}



// Convert rs2::frame to cv::Mat
cv::Mat frame_to_mat(const rs2::frame& f)
{
    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        cvtColor(r, r, CV_RGB2BGR);
        return r;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f)
{
    Mat dm = frame_to_mat(f);
    dm.convertTo(dm, CV_64F);
    auto depth_scale = pipe.get_active_profile()
        .get_device()
        .first<depth_sensor>()
        .get_depth_scale();
    dm = dm * depth_scale;
    return dm;
}

Mat gray2pseudocolor(const Mat& scaledGray)
{
    Mat outputPseudocolor(scaledGray.size(), CV_8UC3);
    unsigned char grayValue;
    for (int y = 0; y < scaledGray.rows; y++)
        for (int x = 0; x < scaledGray.cols; x++)
        {
            grayValue = scaledGray.at<uchar>(y, x);
            Vec3b& pixel = outputPseudocolor.at<Vec3b>(y, x);
            pixel[0] = abs(255 - grayValue);
            pixel[1] = abs(127 - grayValue);
            pixel[2] = abs(0 - grayValue);
        }

    return outputPseudocolor;
}

Mat gray2rainbow(const Mat& scaledGray)
{
    Mat outputRainbow(scaledGray.size(), CV_8UC3);
    unsigned char grayValue;
    for (int y = 0; y < scaledGray.rows; y++)
        for (int x = 0; x < scaledGray.cols; x++)
        {
            grayValue = scaledGray.at<uchar>(y, x);
            Vec3b& pixel = outputRainbow.at<Vec3b>(y, x);
            if (grayValue <= 51)
            {
                pixel[0] = 255;
                pixel[1] = grayValue * 5;
                pixel[2] = 0;
            }
            else if (grayValue <= 102)
            {
                grayValue -= 51;
                pixel[0] = 255 - grayValue * 5;
                pixel[1] = 255;
                pixel[2] = 0;
            }
            else if (grayValue <= 153)
            {
                grayValue -= 102;
                pixel[0] = 0;
                pixel[1] = 255;
                pixel[2] = grayValue * 5;
            }
            else if (grayValue <= 204)
            {
                grayValue -= 153;
                pixel[0] = 0;
                pixel[1] = 255 - static_cast<unsigned char>(grayValue * 128.0 / 51 + 0.5);
                pixel[2] = 255;
            }
            else if (grayValue <= 255)
            {
                grayValue -= 204;
                pixel[0] = 0;
                pixel[1] = 127 - static_cast<unsigned char>(grayValue * 127.0 / 51 + 0.5);
                pixel[2] = 255;
            }
        }
    return outputRainbow;
}

int main(int argc, char * argv[])
{
//        VideoCapture cap;
//        cap.open(0);
//        if(!cap.isOpened())
//            return 0;

    // Define colorizer and align processing-blocks
    colorizer colorize;
    rs2::align align_to(RS2_STREAM_COLOR);

    // Start the camera
    pipeline pipe;
    pipe.start();

    // We are using StructuringElement for erode / dilate operations
    auto gen_element = [](int erosion_size)
    {
        return getStructuringElement(MORPH_RECT,
            Size(erosion_size + 1, erosion_size + 1),
            Point(erosion_size, erosion_size));
    };

    const int erosion_size = 3;
    auto erode_less = gen_element(erosion_size);
    auto erode_more = gen_element(erosion_size * 2);

    // The following operation is taking grayscale image,
    // performs threashold on it, closes small holes and erodes the white area
    auto create_mask_from_depth = [&](Mat& depth, int thresh, ThresholdTypes type)
    {
        threshold(depth, depth, thresh, 255, type);
        dilate(depth, depth, erode_less);
        erode(depth, depth, erode_more);
    };

    // Skips some frames to allow for auto-exposure stabilization
    for (int i = 0; i < 10; i++)
        pipe.wait_for_frames();

    while (waitKey(30) < 0 )
    {
        frameset data = pipe.wait_for_frames();
        // Make sure the frameset is spatialy aligned
        // (each pixel in depth image corresponds to the same pixel in the color image)
        frameset aligned_set = align_to.process(data);
        frame depth = aligned_set.get_depth_frame();
        auto color_mat = frame_to_mat(aligned_set.get_color_frame());

        // Colorize depth image with white being near and black being far
        // This will take advantage of histogram equalization done by the colorizer
        colorize.set_option(RS2_OPTION_COLOR_SCHEME, 2);
        frame bw_depth = colorize(depth);

        // Generate "near" mask image:
        surImgColor=color_mat;
        //cv::namedWindow("sourceImgColor");
        //imshow("sourceImgColor",surImgColor);

        auto near = frame_to_mat(bw_depth);
        cvtColor(near, near, CV_BGR2GRAY);
        //imshow("depth",near);
        Mat ImgDepth=gray2rainbow(near);//gray2pseudocolor(near);
        //imshow("ImgColor",ImgDepth);

        src=surImgColor;//ImgDepth;
        cv::namedWindow("srcMarks");
        src.copyTo(srcMarks);
        cv::setMouseCallback("srcMarks", onMouse, 0);
        cv::imshow("srcMarks", srcMarks);
    }

    return 0;
}








