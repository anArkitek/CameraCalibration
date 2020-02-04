#include "calibration.h"

namespace zw {

    int calibration(int nImgs, int nCorners, int nHoriLines, int nColLines, int lenBlock) {

        std::vector<dlib::matrix<double, 3, 3>> Hall;
        dlib::matrix<double> params;
        dlib::matrix<double> cornersW (2 * nCorners, 1);      // nCorners * 2
        dlib::matrix<double> cornersImg (2 * nImgs * nCorners, 1);    // nImgs * nCorners * 2
        std::vector<cv::Point2d> cornersWorld = zw::initializeCornersOnCalibPad(nHoriLines, nColLines, lenBlock);
        //dbg::printMatrix("cornersW", cornersW);

        for (int iPt = 0; iPt < cornersWorld.size(); ++iPt) {
            cornersW(iPt * 2) = cornersWorld[iPt].x;
            cornersW(iPt * 2 + 1) = cornersWorld[iPt].y;
        }

        for (int iImg = 0; iImg < nImgs; ++iImg) {
            std::string filename = "srcImgs/00" + std::to_string(iImg) + ".jpeg";
            cv::Mat src = cv::imread(filename);

            cv::Mat dst, cdst;
            Canny(src, dst, 100, 200, 3);
            cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);

            std::vector<cv::Vec2f> lines; // will hold the results of the detection
            HoughLines(dst, lines, 1, CV_PI / 180, 180, 0, 0); // runs the actual detection

            std::vector<std::vector<cv::Vec2f>> processedLines = getTargetLines(lines, 4, 7);

            if (bDrawProcessedLines) {
                for (size_t i = 0; i < processedLines.size(); i++)
                {
                    for (size_t j = 0; j < processedLines[i].size(); ++j)
                    {
                        double rho = processedLines[i][j][0], theta = processedLines[i][j][1];

                        cv::Point pt1, pt2;
                        double a = cos(theta), b = sin(theta);
                        double x0 = a * rho, y0 = b * rho;
                        pt1.x = cvRound(x0 + 5000 * (-b));
                        pt1.y = cvRound(y0 + 5000 * (a));
                        pt2.x = cvRound(x0 - 5000 * (-b));
                        pt2.y = cvRound(y0 - 5000 * (a));
                        line(cdst, pt1, pt2, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
                    }
                }
            }

            std::vector<cv::Point2d> pointsImg = twoSetLinesIntersectOnPlane(processedLines[0], processedLines[1]);

            int startPos = iImg * nCorners * 2;
            for (int iPt = 0; iPt < pointsImg.size(); ++iPt) {
                cornersImg(startPos + iPt * 2) = pointsImg[iPt].x;
                cornersImg(startPos + iPt * 2 + 1) = pointsImg[iPt].y;
            }

            if (bDrawPoints) {
                int cnt = 0;
                for (auto point : pointsImg) {
                    cv::circle(cdst, point, 10, cv::Scalar(0, 255, 0), 10);
                    std::cout << cnt << ": " << point.x << ", \t" << point.y << std::endl;
                    ++cnt;
                }
            }

            if (bSavePointsAndCorners) {
                std::ofstream myfile("points.txt", std::ofstream::out | std::ofstream::app);

                if (myfile.is_open())
                {

                    myfile << "==============================================\n";
                    myfile << "Image Order " << std::to_string(iImg) << "\n";
                    myfile << "Points on images: \n";

                    for (int i = 0; i < pointsImg.size(); ++i) {
                        myfile << pointsImg[i].x << " " << pointsImg[i].y << std::endl;
                    }

                    myfile << "Corners on Clibration Pad: \n";
                    for (int i = 0; i < cornersWorld.size(); ++i) {
                        myfile << cornersWorld[i].x << " " << cornersWorld[i].y << std::endl;
                    }
                }
            }
            
            dlib::matrix<double, 3, 3> H = zw::homographyestimation(cornersWorld, pointsImg);
            Hall.push_back(H);

            if (bShowImgs) {
                cv::namedWindow("source", cv::WINDOW_GUI_NORMAL);
                cv::namedWindow("detected lines", cv::WINDOW_GUI_NORMAL);

                cv::imshow("source", src);
                cv::imshow("detected lines", cdst);

                cv::waitKey();
            }
            
        }

        //dbg::printMatrix("cornersImg", cornersImg);

        dlib::matrix<double> V(2 * Hall.size(), 6);
        V = getMatrixV(Hall);
        assert(V.nr() == 2 * nImgs, "Incorrect Dimension of Matrix V!");

        dlib::matrix<double, 6, 1> b = getVectorb(V);
        
        dlib::matrix<double, 3, 3> K = getIntrinsicMatrix(b);
        /*std::cout << "K : " << std::endl << K << std::endl << "=============" << std::endl;*/

        dlib::matrix<double> parameters;

        parameters = getParamters(K, Hall);

        //dbg::printMatrix("parameters", parameters);

        ParaRefine paraRefine;
        dlib::solve_least_squares_lm(dlib::objective_delta_stop_strategy(1e-7).be_verbose(),
            paraRefine.residual,
            dlib::derivative(paraRefine.residual),
            cornersW,
            parameters);

        return 1;
    }
    

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

        //for (int i = 0; i < uniqueLines.size(); ++i) {
        //    std::cout << uniqueLines[i][0] << ", " << uniqueLines[i][1] << std::endl;
        //}

        assert(memo.size() == uniqueLines.size(), "Issues with accumulating uniqueLines");

        // Divde the sum of rhos and thetas by its number
        for (int i = 0; i < memo.size(); ++i) {
            uniqueLines[i][0] = static_cast<double>(memo[i][0]) / memo[i][2];
            uniqueLines[i][1] = static_cast<double>(memo[i][1]) / memo[i][2];
        }

        //std::cout << uniqueLines.size() << " unique lines are detected from the calibration pad" << std::endl;
        //std::cout << "There are supposed to be " << horiNum << " horizontal lines" << std::endl;
        //std::cout << "There are supposed to be " << verNum << " vertical lines" << std::endl;
        
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
                double y = static_cast<double>(i) * length;
                double x = static_cast<double>(j) * length;
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

        //for (int i = 0; i < A.size(); ++i) {
        //    for (int j = 0; j < A[0].size(); ++j) {
        //        std::cout << A[i][j] << " ";
        //    }
        //    std::cout << std::endl;
        //}

        return A;
    }


    dlib::matrix<double, 3, 3> homographyestimation(std::vector<cv::Point2d> pw, std::vector<cv::Point2d> pimg) {
        std::vector<std::vector<double>> A0 = getMatrixA(pw, pimg);

        dlib::matrix<double> A(A0.size(), 9);

        for (int r = 0; r < A0.size(); ++r) {
            for (int c = 0; c < A0[0].size(); ++c) {
                A(r, c) = A0[r][c];
            }
        }

        dlib::matrix<double> U, D, V;

        dlib::svd(A, U, D, V);

        dlib::matrix<double> Hv(9, 1);
        Hv = dlib::colm(V, 8);
        
        dlib::matrix<double, 3, 3> H;
        H = Hv(0), Hv(1), Hv(2), Hv(3), Hv(4), Hv(5), Hv(6), Hv(7), Hv(8);
        // std::cout << "H matrix is \n" << H << std::endl << "=================" << std::endl;
        return H;
    }


    dlib::matrix<double> getMatrixV(const std::vector<dlib::matrix<double, 3, 3>>& Hall) {
        dlib::matrix<double> V(Hall.size() * 2, 6);
        int cnt = 0;
        for (auto H : Hall) {
            int i = 0; int j = 1;
            dlib::matrix<double> v12(1, 6);
            v12 = H(0, i) * H(0, j),                     H(0, i) * H(1, j) + H(1, i) * H(0, j), H(1, i) * H(1, j), 
                  H(2, i) * H(0, j) + H(0, i) * H(2, j), H(2, i) * H(1, j) + H(1, i) * H(2, j), H(2, i) * H(2, j);

            i = 0; j = 0;
            dlib::matrix<double> v11(1, 6);
            v11 = H(0, i) * H(0, j),                     H(0, i) * H(1, j) + H(1, i) * H(0, j), H(1, i) * H(1, j), 
                  H(2, i) * H(0, j) + H(0, i) * H(2, j), H(2, i) * H(1, j) + H(1, i) * H(2, j), H(2, i) * H(2, j);
            
            i = 1; j = 1;
            dlib::matrix<double> v22(1, 6);
            v22 = H(0, i) * H(0, j),                     H(0, i) * H(1, j) + H(1, i) * H(0, j), H(1, i) * H(1, j), 
                  H(2, i) * H(0, j) + H(0, i) * H(2, j), H(2, i) * H(1, j) + H(1, i) * H(2, j), H(2, i) * H(2, j);

            dlib::set_rowm(V, cnt) = v12;
            dlib::set_rowm(V, cnt + 1) = v11 - v22;

            //std::cout << "v11" << std::endl;
            //std::cout << v11 << std::endl << "=====================" << std::endl;
            //std::cout << "v22" << std::endl;
            //std::cout << v22 << std::endl << "=====================" << std::endl;
            //std::cout << "v11-v22" << std::endl;
            //std::cout << v11-v22 << std::endl << "=====================" << std::endl;
            cnt += 2;
        }
        return V;
    }
    

    dlib::matrix<double> getVectorb(const dlib::matrix<double>& V) {
        dlib::matrix<double> U, D, T;
        dlib::svd(V, U, D, T);
        assert(T.nr() == 6, "Issues with get Vector b");
        dlib::matrix<double, 6, 1> b = dlib::colm(T, 5);
        return b;
    }
    

    dlib::matrix<double, 3, 3> getIntrinsicMatrix(const dlib::matrix<double, 6, 1>& b) {
        double y0 = (b(1) * b(3) - b(0) * b(4)) / (b(0) * b(2) - b(1) * b(1));
        double lambda = b(5) - (b(3) * b(3) + y0 * (b(1) * b(3) - b(0) * b(4))) / b(0);
        double ax = std::sqrt(lambda / b(0));
        double ay = std::sqrt(lambda * b(0) / (b(0) * b(2) - b(1) * b(1)));
        double s = - b(1) * ax * ax * ay / lambda;
        double x0 = s * y0 / ay - b(3) * ax * ax / lambda;
        dlib::matrix<double, 3, 3> K;
        K = ax, s,  x0, 
            0,  ay, y0, 
            0,  0,  1;
        return K;
    }

    dlib::matrix<double> getParamters(const dlib::matrix<double, 3, 3>& K, 
                                      const std::vector<dlib::matrix<double, 3, 3>> Hall) {

        dlib::matrix<double, 0, 1> parameters;
        int cnt = -1;

        if (bRadialDistort) {
            parameters.set_size(7 + Hall.size() * 6);
            parameters(5) = 0;
            parameters(6) = 0;
            cnt = 6;
        }
        else {
            parameters.set_size(5 + Hall.size() * 6);
            cnt = 4;
        }

        // Instrinsics
        parameters(0) = K(0, 0);
        parameters(1) = K(0, 1);
        parameters(2) = K(0, 2);
        parameters(3) = K(1, 1);
        parameters(4) = K(1, 2);

        dlib::matrix<double, 3, 3> KInverse = dlib::inv(K);

        for (int iH = 0; iH < Hall.size(); ++iH) {
            dlib::matrix<double, 3, 3> H = Hall.at(iH);
            dlib::matrix<double, 3, 1> t = KInverse * dlib::colm(H, 2);

            double r1Norm = std::sqrt(dlib::sum(dlib::squared(KInverse * dlib::colm(H, 0))));
            double r2Norm = std::sqrt(dlib::sum(dlib::squared(KInverse * dlib::colm(H, 1))));

            //std::cout << "r1Norm: " << r1Norm << std::endl;
            //std::cout << "r2Norm: " << r2Norm << std::endl;
        
            if (t(2) < 0) {
                r1Norm *= -1;
            }


            dlib::vector < double, 3> r1 = KInverse * dlib::colm(H, 0) / r1Norm;
            dlib::vector < double, 3> r2 = KInverse * dlib::colm(H, 1) / r2Norm;
            dlib::vector < double, 3> r3 = r1.cross(r2);
            
            dlib::matrix<double, 3, 3> R;
            dlib::set_colm(R, 0) = r1;
            dlib::set_colm(R, 1) = r2;
            dlib::set_colm(R, 2) = r3;


            t /= r1Norm;

            // Condition the matrix by setting singular values all to be 1
            //dbg::printMatrix("R", R);
            //dlib::matrix<double, 3, 3> U, D, T;
            //dlib::svd(K, U, D, T);
           
            //R = U * dlib::trans(T);

            dbg::printMatrix("R", R);

            dlib::matrix<double, 3, 1> axisAngle = R2AxisAngle(R);
            //dbg::printMatrix("t", t);
            //dbg::printMatrix("axisAngle", axisAngle);

            parameters(cnt + 1) = axisAngle(0);
            parameters(cnt + 2) = axisAngle(1);
            parameters(cnt + 3) = axisAngle(2);
            parameters(cnt + 4) = t(0);
            parameters(cnt + 5) = t(1);
            parameters(cnt + 6) = t(2);
            cnt += 6;
        }

        return parameters;
    }


    dlib::matrix<double, 3, 1> R2AxisAngle(const dlib::matrix<double, 3, 3> R) {
        double phi = std::acos((static_cast<double>(dlib::trace(R)) - 1) / 2);
        dlib::matrix<double, 3, 1> w_temp;
        w_temp = R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
        dlib::matrix<double, 3, 1> w = phi / (2 * std::sin(phi)) * w_temp;

        return w;
    }


    void test() {
        dlib::matrix<double> U(4, 4), D(4, 2), V(2, 2);
        dlib::matrix<double> M(4, 2);

        M = 1, 3,
            1, 2,
            1, -1,
            2, 1;


        dlib::svd(M, U, D, V);

        dlib::matrix<double, 2, 1> v0 = dlib::colm(V, 0);
        dlib::matrix<double, 2, 1> v1 = dlib::colm(V, 1);

        std::cout << "M * v0 : " << std::endl << M * v0 << std::endl;
        std::cout << "M * v1 : " << std::endl << M * v1 << std::endl;

        //std::cout << "M * x - y : " << M * x - y << std::endl;

        std::cout << "*******************************" << std::endl;
    }
}