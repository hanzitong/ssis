
#ifndef SSIS_SSIS_TOOLS_HPP
#define SSIS_SSIS_TOOLS_HPP

#include <nlopt.h>

namespace compare_func
{

double select_min(const std::vector<double> input){
    double min = input[0];
    int method = 1; // 1:tether1, 2:tether12, 3:tether123

    for(int i=1; i<input.size(); i++){
        if(min >= input[i]){// change min
           min = input[i];
           method = i + 1;
        }else{
            std::cout << "no compare !!!!!!!!!!!" << std::endl;
        }
    }

    for(int j=0; j<input.size(); j++){
        if(std::isnan(input[j])){return -1;}
    }

    // return min;
    return method;
}

}   // namespace compare_func

#endif
