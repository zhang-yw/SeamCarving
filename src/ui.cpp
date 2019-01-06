#include <iostream>
#include <algorithm>
#include "ui.h"

bool drawing;

struct WindowParam
{
    int type;//0 for delete, 1 for protect
    cv::Mat3b img;
    cv::Mat1b mask; /// Using reference counting, no need to be reference
};

void onMonse(int event, int x, int y, int flags, void *_param)
{
    WindowParam *param = (WindowParam*)_param;
    int radius =10;
    switch(event){
        case CV_EVENT_LBUTTONDOWN:
            drawing = true;
            for (int i = std::max(0, y - radius); i <= std::min(param->img.rows - 1, y + radius); i++)
                for (int j = std::max(0, x - radius); j <= std::min(param->img.cols - 1, x + radius); j++)
                    if (cv::norm(cv::Point(j, i) - cv::Point(x, y)) <= radius && !param->mask(i, j))
                    {
                        if(param->type)
                            param->img(i, j) = param->img(i, j) * 0.7 + cv::Vec3b(0, 255, 0) * 0.3; // BGR
                        else
                            param->img(i, j) = param->img(i, j) * 0.7 + cv::Vec3b(0, 0, 255) * 0.3; // BGR
                        param->mask(i, j) = 1;
                    }
            cv::imshow("Draw Mask Area", param->img);
            break;
        case CV_EVENT_LBUTTONUP:
            drawing = false;
            break;
        case CV_EVENT_MOUSEMOVE:
            if(!drawing)
                break;
            for (int i = std::max(0, y - radius); i <= std::min(param->img.rows - 1, y + radius); i++)
                for (int j = std::max(0, x - radius); j <= std::min(param->img.cols - 1, x + radius); j++)
                    if (cv::norm(cv::Point(j, i) - cv::Point(x, y)) <= radius && !param->mask(i, j))
                    {
                        if(param->type)
                            param->img(i, j) = param->img(i, j) * 0.7 + cv::Vec3b(0, 255, 0) * 0.3; // BGR
                        else
                            param->img(i, j) = param->img(i, j) * 0.7 + cv::Vec3b(0, 0, 255) * 0.3; // BGR
                        param->mask(i, j) = 1;
                    }
            cv::imshow("Draw Mask Area", param->img);
            break;
    }
}

void getMask(const cv::Mat3b img, cv::Mat1b &delMask_, cv::Mat1b &protectMask_)
{
    std::cout << "Please use the cursor to mark the area(s) you want to delete(Red area)" << std::endl;
    std::cout << "Press any key to proceed to select areas to protect." << std::endl;
    
    drawing = false;
    cv::namedWindow("Draw Mask Area", CV_WINDOW_AUTOSIZE);
    cv::Mat1b delMask(img.rows, img.cols, (unsigned char)0);
    WindowParam param_del({0, img.clone(), delMask});
    cv::setMouseCallback("Draw Mask Area", onMonse, (void*)&param_del);
    cv::imshow("Draw Mask Area", param_del.img);
    cv::waitKey(0);
    cv::setMouseCallback("Draw Mask Area", 0, 0);
    delMask_ = delMask;
    //printf("%d %d\n", delMask_.rows, delMask_.cols);

    std::cout << "Please use the cursor to mark the area(s) you want to protect(Green area)" << std::endl;
    std::cout << "Press any key to finish" << std::endl;
    
    drawing = false;
    cv::Mat1b protectMask(img.rows, img.cols, (unsigned char)0);
    WindowParam param_protect({1, param_del.img, protectMask});
    cv::setMouseCallback("Draw Mask Area", onMonse, (void*)&param_protect);
    cv::imshow("Draw Mask Area", param_protect.img);
    cv::waitKey(0);
    cv::setMouseCallback("Draw Mask Area", 0, 0);
    protectMask_ = protectMask;
    cv::destroyAllWindows();
    cv::waitKey(1);
    return;
}

