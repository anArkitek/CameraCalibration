#pragma once

#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "dlib/matrix.h"
#include "dlib/matrix/matrix_exp_abstract.h"
#include "dlib/optimization.h"

#include "debug.h"

#define M_PI                    3.14159265358979323846  /* pi */

#define bDrawProcessedLines     false
#define bDrawPoints             false
#define bSavePointsAndCorners   true
#define bShowImgs               false
#define bRadialDistort          true


namespace zw {

    int calibration(int nImgs, int nCorners = 28, int nHoriLines = 4, int nColLines = 7, int lenBlock = 20);
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

    dlib::matrix<double, 3, 3> homographyestimation(std::vector<cv::Point2d> pw, std::vector<cv::Point2d> pimg);

    // Vb = 0
    dlib::matrix<double> getMatrixV(const std::vector<dlib::matrix<double, 3, 3>>& Hall);

    dlib::matrix<double> getVectorb(const dlib::matrix<double>& V);

    dlib::matrix<double, 3, 3> getIntrinsicMatrix(const dlib::matrix<double, 6, 1>& b);

    /*
     * [ax, s, x0, ay, y0, k1, k2, 6 * nImg]
     */
    dlib::matrix<double> getParamters(const dlib::matrix<double, 3, 3>& K, const std::vector<dlib::matrix<double, 3, 3>> Hall);

    dlib::matrix<double, 3, 1> R2AxisAngle(const dlib::matrix<double, 3, 3> R);


    class ParaRefine {
    public:
        ParaRefine() = default;

        // input: coordinates on the calibration pad
        // parameters: 5 instrinsic + 2 distortion + 6 extrinsic
        static dlib::matrix<double> model(const dlib::matrix<double>& cornersW, const dlib::matrix<double, 13, 1> parameters) {
            
            assert(cornersW.nr() == 2 * nCorners, "Input for LM Optimization is incorrect!");
            
            double ax = parameters(0);
            double s  = parameters(1);
            double x0 = parameters(2);
            double ay = parameters(3);
            double y0 = parameters(4);

            dlib::matrix<double, 3, 3> K, K1;
            K = ax, s, x0, 0, ay, y0, 0, 0, 1;
            int cnt = -1;
            double k1 = -1;
            double k2 = -1;
            if (bRadial) {
                K1 = ax, 0, x0, 0, ay, y0, 0, 0, 1;
                k1 = 0;
                k2 = 0;
                cnt = 6;    // 5 intrinsic parameters + 2 radial distortion parameters
            }
            else {
                K1 = ax, 0, x0, 0, ay, y0, 0, 0, 1;
                cnt = 4;
            }

            dlib::matrix<double, 3, 1> w, t;
            w = parameters(cnt + 1), parameters(cnt + 2), parameters(cnt + 3);
            t = parameters(cnt + 4), parameters(cnt + 5), parameters(cnt + 6);
            cnt += 6;

            dlib::matrix<double, 3, 3> wcross;
            wcross = 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

            dlib::matrix<double, 3, 3> U, D, V;
            dlib::svd(wcross, U, D, V);
            double phi = D(0, 0) > D(1, 1) ? (D(0, 0) > D(2, 2) ? D(0, 0) : D(2, 2)) : (D(1, 1) > D(2, 2) ? D(1, 1) : D(2, 2));
            dlib::matrix<double, 3, 3> R;
            R = dlib::identity_matrix<double>(3) + std::sin(phi) / phi * wcross + (1 - std::cos(phi)) / phi * wcross * wcross;

            dlib::matrix<double, 3, 4> extrinsicMatrix;
            dlib::set_colm(extrinsicMatrix, dlib::range(0, 2)) = R;
            dlib::set_colm(extrinsicMatrix, 3) = t;

            // the points calculated from the projection
            dlib::matrix<double> cornersHat(nCorners * 2, 1);
            int n2 = 0; // for xW
            for (int iCorners = 0; iCorners < nCorners; ++iCorners) {
                dlib::matrix<double, 4, 1> xW;
                xW = cornersW(n2), cornersW(n2 + 1), 0, 1;
                dlib::matrix<double, 3, 1> x;
                x = K * extrinsicMatrix * xW;
                cornersHat(n2) = x(0) / x(2);
                cornersHat(n2 + 1) = x(1) / x(2);

                if (bRadial) {
                    dlib::matrix<double, 3, 1>xp;
                    xp = cornersHat(n2), cornersHat(n2 + 1), 1;

                    dlib::matrix<double, 3, 1> xw = dlib::inv(K1) * xp;
                    double r2 = xw(0) * xw(0) + xw(1) * xw(1);
                    double xp1 = xw(0) + xw(0) * (k1 * r2 + k2 * r2 * r2);
                    double xp2 = xw(1) + xw(1) * (k1 * r2 + k2 * r2 * r2);
                    dlib::matrix<double, 3, 1> tmpx;
                    tmpx = xp1, xp2, 1;
                    x = K1 * tmpx;
                    cornersHat(n2) = x(0) / x(2);
                    cornersHat(n2 + 1) = x(1) / x(2);
                }
                n2 += 2;
            }
            return cornersHat;
        }


        static double residual(const std::pair<dlib::matrix<double>, dlib::matrix<double>>& data,
            const dlib::matrix<double>& parameters) {

            double error = 0;

            assert(data.first.nr() == 2 * nCorners, "numbers of world corners have issues!");
            assert(data.second.nr() == 2 * nImgs * nCorners, "numbers of image corners have issues!");
            if (bRadial) {
                assert(parameters.nr() == 7 + 6 * nImgs, "numbers of parameters have issues!");
            }
            else {
                assert(parameters.nr() == 5 + 6 * nImgs, "numbers of parameters have issues!");
            }
            for (int iImg = 0; iImg < nImgs; ++iImg) {
                dlib::matrix<double> cornersPerImage = dlib::rowm(data.second, dlib::range(iImg * 2 * nCorners, (iImg + 1) * 2 * nCorners - 1));
                
                dlib::matrix<double> parasPerImage(0, 1);
                if (bRadial) {
                    parasPerImage.set_size(13, 1);
                    dlib::set_rowm(parasPerImage, dlib::range(0, 6)) = dlib::rowm(parameters, dlib::range(0, 6));
                    dlib::set_rowm(parasPerImage, dlib::range(7, 12)) = dlib::rowm(parameters, dlib::range(7 + iImg * 6, (iImg + 1) * 6 + 6));
                }
                else {
                    parasPerImage.set_size(11, 1);
                    dlib::set_rowm(parasPerImage, dlib::range(0, 4)) = dlib::rowm(parameters, dlib::range(0, 4));
                    dlib::set_rowm(parasPerImage, dlib::range(5, 10)) = dlib::rowm(parameters, dlib::range(5 + iImg * 6, (iImg + 1) * 6 + 4));
                }
                
                assert(cornersPerImage.nr() == nCorners * 2);
                dlib::matrix<double> cornersHat(2 * nCorners, 1);
                cornersHat = model(data.first, parasPerImage);

                error += dlib::sum(dlib::squared(cornersHat - cornersPerImage));
            }

            return error;
        }

        static int                     nImgs;
        static int                     nCorners;
        static bool                    bRadial;
    };

    void test();
}