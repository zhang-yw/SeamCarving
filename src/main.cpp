#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "ui.h"
int state = 0;

std::vector<int> DP(cv::Mat3b &image, cv::Mat1i &coord, cv::Mat1b &delMask, cv::Mat1b proMask, cv::Mat2i &oriCoord, cv::Mat3b &seamsOut, cv::Mat3b &seams_inc){
    assert(coord.empty() || image.rows == coord.rows && image.cols == coord.cols);
    assert(image.rows == oriCoord.rows && image.cols == oriCoord.cols);
    assert(delMask.empty() || image.rows == delMask.rows && image.cols == delMask.cols);
    assert(proMask.empty() || image.rows == proMask.rows && image.cols == proMask.cols);
    cv::Mat3d dx, dy;
    cv::Sobel(image, dx, CV_64F, 1, 0, 5);
    cv::Sobel(image, dy, CV_64F, 0, 1, 5);
    cv::Mat1f cost(image.rows, image.cols);
    for (int i = 0; i < image.rows; i++)
        for (int j = 0; j < image.cols; j++)
            cost(i, j) = !proMask.empty() && proMask(i, j) ? 1e10 : (!delMask.empty() && delMask(i, j)? 1e-10 : norm(dx(i, j)) + norm(dy(i, j)));

    cv::Mat1i last(image.rows, image.cols);
    for (int i = 1; i < image.rows; i++)
        for (int j = 0; j < image.cols; j++)
        {
            double val(cost(i - 1, j)), pos(j);
            if (j > 0 && cost(i - 1, j - 1) < val)
                val = cost(i - 1, j - 1), pos = j - 1;
            if (j < image.cols - 1 && cost(i - 1, j + 1) < val)
                val = cost(i - 1, j + 1), pos = j + 1;
            cost(i, j) += val, last(i, j) = pos;
        }

    std::vector<int> seam(coord.rows); // if coord.empty() than seam.empty()
    cv::Mat3b _image(image.rows, image.cols - 1);
    cv::Mat1i _coord(coord.rows, coord.empty() ? 0 : coord.cols - 1);
    cv::Mat2i _oriCoord(oriCoord.rows, oriCoord.cols - 1);
    cv::Mat1b _delMask(delMask.rows, delMask.empty() ? 0 : delMask.cols - 1);
    cv::Mat1b _proMask(proMask.rows, proMask.empty() ? 0 : proMask.cols - 1);
    cv::Mat3b _seams_inc(seams_inc.rows, seams_inc.empty() ? 0 : seams_inc.cols - 1);
    double val(cost(image.rows - 1, 0)), pos(0);
    for (int j = 1; j < image.cols; j++)
        if (cost(image.rows - 1, j) < val)
            val = cost(image.rows - 1, j), pos = j;
    for (int i = image.rows - 1; i >= 0; i--)
    {
        if (!coord.empty())
            seam[i] = coord(i, pos);
        if(!seamsOut.empty()){
            if(state == 0)
                seamsOut(oriCoord(i, pos)[0], oriCoord(i, pos)[1]) *= 0.2;
            else
                seamsOut(oriCoord(i, pos)[1], oriCoord(i, pos)[0]) *= 0.2;
        }
        for (int j = 0; j < image.cols - 1; j++)
        {
            _image(i, j) = image(i, j < pos ? j : j + 1);
            if(!seams_inc.empty())
                _seams_inc(i ,j) = seams_inc(i, j < pos ? j : j + 1);
            if (!coord.empty())
                _coord(i, j) = coord(i, j < pos ? j : j + 1);
            _oriCoord(i, j) = oriCoord(i, j < pos ? j : j + 1);
            if (!delMask.empty())
                _delMask(i, j) = delMask(i, j < pos ? j : j + 1);
            if (!proMask.empty())
                _proMask(i, j) = proMask(i, j < pos ? j : j + 1);
        }
        pos = last(i, pos);
    }
    image = _image, coord = _coord, oriCoord = _oriCoord, delMask = _delMask, proMask = _proMask;
    if(!seams_inc.empty())
        seams_inc = _seams_inc;
    return seam;
}

cv::Mat3b carveHoriDecline(cv::Mat3b image, int target, cv::Mat1b delMask, cv::Mat1b proMask, cv::Mat2i &oriCoord, cv::Mat3b &seams_dec, cv::Mat3b &seam_inc){
    cv::Mat1i coord_inc;
    while(image.cols > target){
        DP(image, coord_inc, delMask, proMask, oriCoord, seams_dec, seam_inc);
    }
    return image;
}

cv::Mat3b carveHoriIncrease(cv::Mat3b image, int target, cv::Mat2i &oriCoord, cv::Mat3b &seams_inc)
{
    cv::Mat3b imageDp;
    cv::Mat1i coordDp, coord;
    cv::Mat2i oriCoordDp;
    int origin(0);
    while (image.cols < target)
    {
        if (coord.empty() || imageDp.cols < origin * 0.66)
        {
            coord = cv::Mat1i(image.rows, image.cols);
            for (int i = 0; i < image.rows; i++)
                for (int j = 0; j < image.cols; j++)
                    coord(i, j) = j;
            image.copyTo(imageDp);
            coord.copyTo(coordDp);
            oriCoord.copyTo(oriCoordDp);
            origin = image.cols;
        }
        cv::Mat1b delMask;
        cv::Mat1b proMask;
        cv::Mat3b seams_dec, seams_inc_;
        auto seam = DP(imageDp, coordDp, delMask, proMask, oriCoordDp, seams_dec, seams_inc_);
        cv::Mat3b _image(image.rows, image.cols + 1);
        cv::Mat1i _coord(coord.rows, coord.cols + 1);
        cv::Mat2i _oriCoord(oriCoord.rows, oriCoord.cols + 1);
        cv::Mat3b _seams_inc(seams_inc.rows, seams_inc.cols + 1);
        for (int i = 0; i < image.rows; i++)
        {
            int dec(0);
            int flag;
            for (int j = 0; j < image.cols + 1; j++)
            {
                _image(i, j) = image(i, j - dec);
                _seams_inc(i, j) = seams_inc(i, j - dec);
                _coord(i, j) = coord(i, j - dec);
                _oriCoord(i, j) = oriCoord(i, j - dec);
                if(coord(i, j) == seam[i]){
                    dec = 1;
                    flag = j;
                }
            }
            _seams_inc(i, flag) *= 0.2;
        }
        image = _image, coord = _coord, oriCoord = _oriCoord, seams_inc = _seams_inc;
    }
    return image;
}

inline cv::Mat3b carveHorizon(cv::Mat3b image, int target, cv::Mat1b delMask, cv::Mat1b proMask, cv::Mat2i &oriCoord, cv::Mat3b &seams_dec, cv::Mat3b &seams_inc){
    if(target <= image.cols){
        return carveHoriDecline(image, target, delMask, proMask, oriCoord, seams_dec, seams_inc);
    }
    else{
        return carveHoriIncrease(image, target, oriCoord, seams_inc);
    }
}

int main(int argc, const char **argv)
{
    if (argc != 5)
    {
        std::cout << "Usage:" << " ./seamCarving <input_file> <output_file> <target_width> <target_height>" << std::endl;
        return -1;
    }
    const char *inputFile = argv[1], *outputFile = argv[2];
    int targetWidth, targetHeight;
    targetWidth = atoi(argv[3]);
    targetHeight = atoi(argv[4]);
    cv::Mat3b image = cv::imread(inputFile);
    cv::Mat2i oriCoord(image.rows, image.cols);
    for (int i = 0; i < image.rows; i++)
        for (int j = 0; j < image.cols; j++)
            oriCoord(i, j)[0] = i, oriCoord(i, j)[1] = j;
    cv::Mat3b seams_dec = image.clone();
    cv::Mat3b seams_inc = image.clone();

    cv::Mat1b delMask, proMask;
    getMask(image, delMask, proMask);

    std::cout << "Processing" << std::endl;

    image = carveHorizon(image, targetWidth, delMask, proMask, oriCoord, seams_dec, seams_inc);
    oriCoord = oriCoord.t();
    seams_inc = seams_inc.t();
    cv::Mat3b image_t = image.t();
    delMask = delMask.t();
    proMask = proMask.t();
    seams_dec = seams_dec.t();
    state++;
    image = carveHorizon(image_t, targetHeight, delMask, proMask, oriCoord, seams_dec, seams_inc).t();
    seams_inc = seams_inc.t();
    seams_dec = seams_dec.t();
    oriCoord = oriCoord.t();

    assert(image.cols == targetWidth);
    assert(image.rows == targetHeight);
    cv::imwrite(outputFile, image);
    cv::imwrite((std::string(outputFile) + ".seams_dec.jpg").c_str(), seams_dec);
    cv::imwrite((std::string(outputFile) + ".seams_inc.jpg").c_str(), seams_inc);
    return 0;
}