#include <iostream>
#include "SA.h"
#include "ctime"
//matlab 61.4737s
//cpp
clock_t start,end;

int main() {

    std::vector<double> costArray;
    double minCost;
    double costTime;
    std::string str="D:\\jetbrainsProject\\CLion\\Cpp\\SA\\SA.txt";

    Model model(60,3);
    SA_VRP sa;
    model.createMap();
    model.calculateDismap();
    start=clock();
    std::vector<int> minRoute=sa.saVpr(model,costArray,minCost);
    end=clock();
    costTime=(double)(end-start)/CLOCKS_PER_SEC;
    sa.printResult(str,costArray,minCost,model,costTime,minRoute);
//    std::vector<int> route={0,1,2,3,15,4,5,6,16,7,8,9,10,11,12,13,14};
//    std::vector<int> b;//保存大于point的下标，用于判断是否一架无人机飞太多点
//    b.push_back(-1);
//    for(int i=0;i<route.size();i++){
//        if(route[i]>=15){
//            b.push_back(i);
//        }
//    }
//    b.push_back(15+3-1);
//    int a;
//    for(int j=1;j<=3;j++){
//        a=b[j]-b[j-1]-1;
//        if(a>7){
//            return false;
//        }
//    }
    return 0;
}
