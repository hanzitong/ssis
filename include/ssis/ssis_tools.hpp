
#ifndef SSIS_SSIS_TOOLS_HPP
#define SSIS_SSIS_TOOLS_HPP

#include <nlopt.h>

namespace compare_func
{

// use variadic template in the future
double select_min(const std::vector<double> input){
    // argument input contains tension_uav sum-> [0]:tether1, [1]:tether1&2, [2]:tether1&2&3

    double min = input[0];  // set initial result tether1, which means colum 0.
    int method = 1; // 1:tether1, 2:tether12, 3:tether123, 4:tether1_winch2

    for(int j=0; j<input.size(); j++){
        if(std::isnan(input[j])){return -1;}
    }

    for(int i=1; i<input.size(); i++){
        if(min >= input[i]){// change min
           min = input[i];
           method = i + 1;
        }else{
            std::cout << "no compare !!!!!!!!!!!" << std::endl;
        }
    }


    // return min;
    return method;
}

}   // namespace compare_func

#endif
