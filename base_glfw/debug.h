#pragma once
#include <iostream>
#include <string>
#include <vector>
#include "dlib/matrix.h"
#include "dlib/matrix/matrix_exp_abstract.h"

namespace dbg {
 
    template <typename T>
    void printMatrix(std::string varName, T matrix) {

        std::cout << "==================" << std::endl;
        std::cout << varName << std::endl;
        std::cout << matrix << std::endl;
        std::cout << "==================" << std::endl;
    }


    template <typename T>
    void print1dVector(std::string varName, std::vector<T> vec) {
        std::cout << "==================" << std::endl;
        std::cout << varName << std::endl;
        for (auto ele : vec) {
            std::cout << ele << " ";
        }
        std::cout << std::endl << "==================" << std::endl;
    }

}