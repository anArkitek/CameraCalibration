#include "calibration.h"

namespace zw {

    std::vector<std::vector<cv::Vec2f>> getTargetLines(std::vector<cv::Vec2f> lines, int horiNum, int verNum) {

        std::vector<cv::Vec2f> uniqueLines; // [rho theta]
        std::vector<cv::Vec3f> memo; // accumulation of rho, theta and number

        for (int i = 0; i < lines.size(); ++i) {

            double rho = lines[i][0], theta = lines[i][1];
            bool bMerged = false;

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

        assert(memo.size() == uniqueLines.size(), "Issues with accumulating uniqueLines");

        // Divde the sum of rhos and thetas by its number
        for (int i = 0; i < memo.size(); ++i) {
            uniqueLines[i][0] = static_cast<double>(memo[i][0]) / memo[i][2];
            uniqueLines[i][1] = static_cast<double>(memo[i][1]) / memo[i][2];
        }

        //std::cout << "number of lines: " << lines.size() << std::endl;
        //std::cout << "number of uniqueLines: " << uniqueLines.size() << std::endl;

        assert(uniqueLines.size() == horiNum + verNum + 4, 
            "The number of target lines is not the same as the number of lines after hough transform!");

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


    std::vector<cv::Point> linesIntersect(std::vector<cv::Vec2f> horiLines, std::vector<cv::Vec2f> verLines) {
        
        std::vector<cv::Point> points;
        
        for (auto horiLine : horiLines) {
            for (auto verLine : verLines) {
                cv::Point point = twoLinesIntersectOnPlane(horiLine, verLine);
                points.push_back(point);
            }
        }
        
        return points;
    }


    cv::Point twoLinesIntersectOnPlane(cv::Vec2f line1, cv::Vec2f line2) {

        glm::vec3 line1Homo = rhoThetaToHomo(line1);
        glm::vec3 line2Homo = rhoThetaToHomo(line2);

        glm::vec3 intersect = glm::cross(line1Homo, line2Homo);

        intersect[0] = static_cast<double>(intersect[0]) / intersect[2];
        intersect[1] = static_cast<double>(intersect[1]) / intersect[2];
        intersect[2] = 1.0f;

        return cv::Point(intersect[0], intersect[1]);
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
}