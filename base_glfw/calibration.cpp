#include "calibration.h"

namespace zw {

    std::vector<std::vector<cv::Vec2f>> getTargetLines(std::vector<cv::Vec2f> lines, int horiNum, int verNum) {

        std::vector<cv::Vec2f> uniqueLines; // [rho theta]
        std::vector<cv::Vec3f> memo; // accumulation of rho, theta and number

        for (int i = 0; i < lines.size(); ++i) {

            double rho = lines[i][0], theta = lines[i][1];
            bool bMerged = false;

            if (rho < 0 && theta * 180.0 / M_PI > 135) {
                rho *= -1;
                theta -= M_PI;
            }

            for (int j = 0; j < uniqueLines.size(); ++j) {

                if (fabs(uniqueLines[j][0] - rho) < 100 && fabs(uniqueLines[j][1] - theta) * 180.0 / M_PI < 10) {
                    memo[j][0] += rho;
                    memo[j][1] += theta;
                    memo[j][2] += 1;
                    bMerged = true;
                }
            }

            if (bMerged) {
                bMerged = false;
                continue;
            }
            uniqueLines.push_back(cv::Vec2f(rho, theta));
            memo.push_back(cv::Vec3f(rho, theta, 1));
        }

        for (int i = 0; i < uniqueLines.size(); ++i) {
            if (uniqueLines[i][1] < 0) {
                uniqueLines[i][0] *= -1;
                uniqueLines[i][1] += M_PI;
            }
        }

        for (int i = 0; i < uniqueLines.size(); ++i) {
            std::cout << uniqueLines[i][0] << ", " << uniqueLines[i][1] << std::endl;
        }

        assert(memo.size() == uniqueLines.size(), "Issues with accumulating uniqueLines");

        // Divde the sum of rhos and thetas by its number
        for (int i = 0; i < memo.size(); ++i) {
            uniqueLines[i][0] = static_cast<double>(memo[i][0]) / memo[i][2];
            uniqueLines[i][1] = static_cast<double>(memo[i][1]) / memo[i][2];
        }

        std::cout << uniqueLines.size() << " unique lines are detected from the calibration pad" << std::endl;
        std::cout << "There are supposed to be " << horiNum << " horizontal lines" << std::endl;
        std::cout << "There are supposed to be " << verNum << " vertical lines" << std::endl;
        
        //for (auto line : uniqueLines) {
        //    std::cout << line[0] << ", " << line[1] * 180.0f / M_PI << std::endl;
        //}

        /*cv::Mat img;
        img = cv::imread("srcImgs/003.jpeg");
        for (size_t i = 0; i < uniqueLines.size(); i++)
        {
            double rho = uniqueLines[i][0], theta = uniqueLines[i][1];
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a * rho, y0 = b * rho;
            pt1.x = cvRound(x0 + 1000 * (-b));
            pt1.y = cvRound(y0 + 1000 * (a));
            pt2.x = cvRound(x0 - 1000 * (-b));
            pt2.y = cvRound(y0 - 1000 * (a));
            cv::line(img, pt1, pt2, cv::Scalar(0, 0, 255), 3);
        }

        cv::namedWindow("img", cv::WINDOW_GUI_NORMAL);
        cv::imshow("img", img);
        cv::waitKey();*/

        assert(uniqueLines.size() == horiNum + verNum + 4, "The number of target lines is not the same as the number of lines after hough transform!");

        // Remove all the side lines
        std::vector<cv::Vec2f> horiLines;
        std::vector<cv::Vec2f> verLines;

        for (auto line : uniqueLines) {
            /*std::cout << "rho: " << line[0] << std::endl;
            std::cout << "theta: " << line[1] * 180.0 / M_PI << std::endl;
            std::cout << "===============" << std::endl;*/
            double rho = line[0];
            double theta = line[1] * 180.0f / M_PI;

            if (theta > -30 && theta < 30) {
                verLines.push_back(line);
            }
            else {
                horiLines.push_back(line);
            }
        }

       /* std::cout << "horiLines:" << horiLines.size() << std::endl;
        std::cout << "verLines:" << verLines.size() << std::endl;*/

        assert(horiLines.size() == horiNum + 2 && verLines.size() == verNum + 2, "Fail to separate the vertical and horizontal lines!");

        sort(horiLines.begin(), horiLines.end(), [](cv::Vec2f vec1, cv::Vec2f vec2) {
            return vec1[0] < vec2[0];
        });

        sort(verLines.begin(), verLines.end(), [](cv::Vec2f vec1, cv::Vec2f vec2) {
            return vec1[0] < vec2[0];
        });

        horiLines.erase(horiLines.begin());
        horiLines.pop_back();
        verLines.erase(verLines.begin());
        verLines.pop_back();

        std::vector<std::vector<cv::Vec2f>> result = { horiLines, verLines };

        return result;
    }


    std::vector<cv::Point2d> twoSetLinesIntersectOnPlane(std::vector<cv::Vec2f> horiLines, std::vector<cv::Vec2f> verLines) {
        
        std::vector<cv::Point2d> points;
        
        for (auto horiLine : horiLines) {
            for (auto verLine : verLines) {
                cv::Point2d point = twoLinesIntersectOnPlane(horiLine, verLine);
                points.push_back(point);
            }
        }
        
        return points;
    }


    cv::Point2d twoLinesIntersectOnPlane(cv::Vec2f line1, cv::Vec2f line2) {

        glm::vec3 line1Homo = rhoThetaToHomo(line1);
        glm::vec3 line2Homo = rhoThetaToHomo(line2);

        glm::vec3 intersect = glm::cross(line1Homo, line2Homo);

        intersect[0] = static_cast<double>(intersect[0]) / intersect[2];
        intersect[1] = static_cast<double>(intersect[1]) / intersect[2];
        intersect[2] = 1.0f;

        return cv::Point2d(intersect[0], intersect[1]);
    }

    
    glm::vec3 rhoThetaToHomo(cv::Vec2f line) {
        double rho = line[0];
        double theta = line[1];
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;

        glm::vec3 pt1 = glm::vec3(x0 + 1000 * (-b), y0 + 1000 * (a), 1.0f);
        glm::vec3 pt2 = glm::vec3(x0 - 1000 * (-b), y0 - 1000 * (a), 1.0f);

        glm::vec3 lineHomo = glm::cross(pt1, pt2);

        return lineHomo;
    }


    std::vector<cv::Point2d> initializeCornersOnCalibPad(int row, int col, int length) {
        std::vector<cv::Point2d> corners;
        for (int i = 0; i < row; ++i) {
            for (int j = 0; j < col; ++j) {
                double x = static_cast<double>(i) * length;
                double y = static_cast<double>(j) * length;
                corners.push_back(cv::Point2d(x, y));
            }
        }

        return corners;
    }

    
    std::vector<std::vector<double>> getMatrixA(const std::vector<cv::Point2d>& pW, const std::vector<cv::Point2d>& pImg) {

        std::vector<std::vector<double>> A;

        assert(pW.size() == pImg.size(), "The number of points in the world is not the same as the number on the image!");

        for (int i = 0; i < pW.size(); ++i) {
            std::vector<double> a1 = { pW[i].x, pW[i].y, 1, 0,       0,       0, -pW[i].x * pImg[i].x, -pW[i].y * pImg[i].x, -pImg[i].x };
            std::vector<double> a2 = { 0,       0,       0, pW[i].x, pW[i].y, 1, -pW[i].x * pImg[i].y, -pW[i].y * pImg[i].y, -pImg[i].y };
            A.push_back(a1);
            A.push_back(a2);
        }

        assert(A.size() == 2 * pW.size() && A[0].size() == 9, "The matrix A is of wrong shape!");

        return A;
    }


    dlib::matrix<double, 3, 3> homographyEstimation(std::vector<cv::Point2d> pW,
                                              std::vector<cv::Point2d> pImg,
                                              int rowNum, int colNum) {
        std::vector<std::vector<double>> A0 = getMatrixA(pW, pImg);

        dlib::matrix<double> A(A0.size(), 9);

        for (int r = 0; r < A0.size(); ++r) {
            for (int c = 0; c < A0[0].size(); ++c) {
                A(r, c) = A0[r][c];
            }
        }

        dlib::matrix<double> U, D, V;

        dlib::svd(A, U, D, V);

        // ? How to make D ordered?
        dlib::matrix<double, 9, 1> H = dlib::colm(V, 8);
        H.set_size(3, 3);

        return H;
    }

    void test() {
        dlib::matrix<double> U(4, 4), D(4, 2), V(2, 2);
        dlib::matrix<double> M(4, 2);

        M = 1, 3,
            1, 2,
            1, -1,
            2, 1;

        //M = -0.37796, -0.37796, -0.37796,
        //    -0.75593, -0.68252, -0.36401,
        //    0.59152, 0.22751, -0.17643,
        //    0.73034, 0.43629, -0.4951,
        //    -0.60015, 0.43731, -0.56293, 0.36288;

        dlib::svd(M, U, D, V);

        dlib::matrix<double, 2, 1> v0 = dlib::colm(V, 0);
        dlib::matrix<double, 2, 1> v1 = dlib::colm(V, 1);
        /*
        std::cout << "U : \n " << U << std::endl;
        std::cout << "D : \n " << D << std::endl;
        std::cout << "V : \n " << V << std::endl;
        std::cout << "M : \n " << U * D * dlib::trans(V) << std::endl;*/

        std::cout << "M * v0 : " << std::endl << M * v0 << std::endl;
        std::cout << "M * v1 : " << std::endl << M * v1 << std::endl;

        //std::cout << "M * x - y : " << M * x - y << std::endl;

        std::cout << "*******************************" << std::endl;
    }
}