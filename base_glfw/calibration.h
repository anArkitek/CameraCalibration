#pragma once

#include <vector>
#include <cmath>
#include <iostream>

#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "dlib/matrix.h"

# define M_PI           3.14159265358979323846  /* pi */


namespace zw {
     /*
      * After edge detection and simple Hough transform, we have a lot of duplicate lines,
      * this function helps us to get the unique lines of our target horizontal line nums and vertical line nums
      * Args:
      *      lines: [ [rho0, theta0], [rho1, theta1], ... ]
      *      horiNum: the number of horizontal lines
      *      verNum: the number of vertical lines
      * Return:
      *      [ 
      *        [ [horizontal_line_rho0, horizontal_line_theta0], [horizontal_line_rho1, horizontal_line_theta1], ...],
      *        [ [vertical_line_rho0, vertical_line_theta0], [vertical_line_rho1, vertical_line_theta1], ...],
      *      ]
      */
    std::vector<std::vector<cv::Vec2f>> getTargetLines(std::vector<cv::Vec2f> lines, int horiNum, int verNum);


    /*
     * Get the intersections of two set lines (horizontal lines and vertical lines)
     * Args:
     *      horiLines: [ [rho theta], ...]
     *      verLines: [ [rho theta], ...]
     * Return:
     *      [ [x0, y0], [x1, y1], ...]
     */
    std::vector<cv::Point2d> twoSetLinesIntersectOnPlane(std::vector<cv::Vec2f> horiLines, std::vector<cv::Vec2f> verLines);

    /*
     * Get the intersection of two lines
     * Args:
     *      line1: [rho theta]
     *      line2: [rho theta]
     * Return:
     *      [x, y]
     */
    cv::Point2d twoLinesIntersectOnPlane(cv::Vec2f line1, cv::Vec2f line2);

    /* 
     *Transform a line represented by rho-theta to homogeneous form: ax + by + c = 0
     * Args:
     *      line: [rho theta]
     * Return:
     *      [a, b, c]
     */
    glm::vec3 rhoThetaToHomo(cv::Vec2f line);

    /*
     * Get the real coordinates of corners on the calibration pad
     * Args:
     *      row : row number
     *      col : col number
     *      length : lenth of edges of the blocks
     * Return:
     *      coordinates [[x0, y0], [x1, y1], ...]
     */
    std::vector<cv::Point2d> initializeCornersOnCalibPad(int row, int col, int length);


    /*
     * Get the matrix A in equation Ah = 0; h is the vector composed of [h11, h12, ..., h33]
     * Args:
     *      pW: the vector of point coordinates on the pad
     *      pImg: the vector of point coordinates on the image
     * Return:
     *      matrix A of shape (2*pW.size()) * 9
     */
    std::vector<std::vector<double>> getMatrixA(const std::vector<cv::Point2d>& pW, const std::vector<cv::Point2d>& pImg);


    void test();
}