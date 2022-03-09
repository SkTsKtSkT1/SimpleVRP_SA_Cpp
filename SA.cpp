//
// Created by lzw on 2022/3/8.
//

#include "SA.h"
#include "random"
#include "cmath"
#include "algorithm"
#include "fstream"
#include "string"

std::random_device rd;
void Model::createMap() {
    std::default_random_engine e;
    std::uniform_real_distribution<double>  urdx(0.0,200.0);
    std::uniform_real_distribution<double>  urdy(10.0,200.0);
    for(int i=0;i<_pointMap.size();i++){
        for(int j=0;j<_pointMap[0].size()-_drone;j++){
            if(i==0){
                _pointMap[i][j]=urdx(e);
            }
            else{
                _pointMap[i][j]=urdy(e);
            }
        }
    }
    for(int i=0;i<_drone;i++){
        _pointMap[0][i+_point]=_x0;
        _pointMap[1][i+_point]=_y0;
    }
}

void Model::calculateDismap() {
    std::vector<std::vector<double>> map=getPointMap();
    for(int i=0;i<_distanceMap.size();i++){
        for(int j=0;j<_distanceMap[0].size();j++){
            _distanceMap[j][i]= sqrt(pow((map[0][i]-map[0][j]),2)+pow(map[1][i]-map[1][j],2));
            _distanceMap[i][j]=_distanceMap[j][i];
        }
    }
}

bool SA_VRP::isFeasible( std::vector<int> route, Model model) {
    int len=route.size();
    int point=model.getPoint();
    int drone=model.getDrone();
    if(route[0]>=point || route[len-1]>=point){
        return false;//没用完drone；
    }
    for(int i=1;i<len;i++){
        if(route[i]>=point && route[i-1]>=point){
            return false;
        }
    }
//    std::vector<int>::iterator it ;
//    it= std::upper_bound(route.begin(),route.end(),point);
//    int pos=it-route.begin();
    std::vector<int> b;//保存大于point的下标，用于判断是否一架无人机飞太多点
    b.push_back(-1);
    for(int i=0;i<len;i++){
        if(route[i]>=point){
            b.push_back(i);
        }
    }
    b.push_back(point+drone-1);
    int a;
    for(int j=1;j<=drone;j++){
        a=b[j]-b[j-1]-1;
        if(a>20){
            return false;
        }
    }
    return true;



}


std::vector<int> SA_VRP::randomSol(Model model) {
    std::vector<int> route;
    for(int i=0;i<model.getPoint()+model.getDrone()-1;i++){
        route.push_back(i);
    }
    std::random_shuffle(route.begin(),route.end());
    return route;
}

double SA_VRP::calculateCost( std::vector<int> route, Model model) {
    int point=model.getPoint();
    int drone=model.getDrone();
    int len=route.size();
    std::vector<std::vector<double>> map=model.getDistanceMap();
    double cost=0;
    std::vector<int> temp=route;
    temp.insert(temp.begin(),point+drone);
    temp.push_back(point+drone);
    for(int i=0;i<len-1;i++){
        cost+=map[route[i]][route[i+1]];
    }
    return cost;


}

std::vector<int> Swap(std::vector<int> route){
    int len=route.size();
    int i1;
    int i2;
    std::default_random_engine dre{rd()};
    std::uniform_int_distribution<int> uid(1,len);
    while(1){
    i1=uid(dre)-1;
    i2=uid(dre)-1;
    if(i1!=i2){
        break;
    }
    }
    std::swap(route[i1],route[i2]);
    return route;
}

std::vector<int> Reversion(std::vector<int> route){
    int len=route.size();
    int i1;
    int i2;
    std::default_random_engine dre{rd()};
    std::uniform_int_distribution<int> uid(1,len);
    while(1){
        i1=uid(dre)-1;
        i2=uid(dre)-1;
        if(i1!=i2){
            break;
        }
    }
    if(i1>i2){
        std::reverse(route.begin()+i2,route.begin()+i1);
    }
    else{
        std::reverse(route.begin()+i1,route.begin()+i2);
    }
    return route;
}

std::vector<int> Insertion(std::vector<int> route){
    int len=route.size();
    int i1;
    int i2;
    std::default_random_engine dre{rd()};
    std::uniform_int_distribution<int> uid(1,len);
    while(1){
        i1=uid(dre)-1;
        i2=uid(dre)-1;
        if(i1!=i2){
            break;
        }
    }
    if(i1>i2){
        int numI1=route[i1];
        route.erase(route.begin()+i1);
        route.insert(route.begin()+i2,numI1);
    }
    else{
        int numI2=route[i2];
        route.erase(route.begin()+i2);
        route.insert(route.begin()+i1,numI2);
    }
    return route;
}

std::vector<int> SA_VRP::createNeighbor( std::vector<int> route, Model model, int mode) {
    std::vector<int> newRoute;
    while(1){
        switch (mode) {
            case 1:newRoute= Swap(route);break;
            case 2:newRoute= Reversion(route);break;
            case 3:newRoute= Insertion(route);break;
        }
        if(isFeasible(newRoute,model)){
            break;
        }
    }
    return newRoute;
}

std::vector<int> SA_VRP::saVpr(Model model,std::vector<double> &costArray,double &minCost) {
    int cnt=1;
    int mode;//modify solution(route);
    double cost=0;
    double newCost=0;
    double T=_T0;
    double deltaCost=0;

    std::vector<int> route;
    std::vector<int> newRoute;
    std::vector<int> minRoute;

    std::default_random_engine dre{rd()};

    while(1){
        route= randomSol(model);
        if(isFeasible(route,model)){
            break;
        }
    }//生成初始可行解
    cost= calculateCost(route,model);
    minCost=cost;
    minRoute=route;

    while(T>_Ts){
        for(int i=0;i<_Lk;i++){
            std::uniform_int_distribution<int> uid(1,3);
            mode=uid(dre);
            newRoute= createNeighbor(route,model,mode);
            newCost= calculateCost(newRoute,model);
            deltaCost=newCost- cost;
            if(deltaCost<0){
                cost=newCost;
                route=newRoute;//接受新解
            }
            else{
                std::uniform_real_distribution<double> uidd(0,1);
                if(uidd(dre)<=std::exp(-deltaCost/T)){
                    cost=newCost;
                    route=newRoute;//接受新解
                }
            }
        }//create new solution and verify accept

        costArray.push_back(cost);
        if(cost<minCost){
            minCost=cost;
            minRoute=route;
        }
        T=T*_r;
        //std::cout<<"times:"<<cnt<<",minCost:"<<minCost<<",currentCost:"<<cost<<",T="<<T<<std::endl;

        if(cnt>_maxIter){
            break;
        }
        cnt+=1;

    }
    return minRoute;

}

void SA_VRP::printResult(const std::string str, const std::vector<double> costArray, const double minCost, Model model,
                         double costTime, std::vector<int> minRoute) {
    std::ofstream  fout;
    int flag=1;
    int cnt=1;
    fout.open(str,std::ios::out);
    if(!fout.is_open()){
        std::cerr<<"open fail";
    }
    fout<<"Point:"<<model.getPoint()<<"\t"<<"Drone:"<<model.getDrone()<<"\t"<<"minCost:"<<minCost<<std::endl;
    fout<<"cost "<<costTime<<" sec"<<std::endl;
    for(int i=0;i<minRoute.size();i++){
        if(flag){
            fout<<"The "<<cnt<<"-th drone path:center"<<"\t";
            flag=0;
            cnt++;
        }
        if(minRoute[i]>=model.getPoint()){
            fout<<"center";
            fout<<std::endl;
            flag=1;
            continue;
        }
        fout<<minRoute[i]<<"\t";
    }
    fout<<"center";
    fout<<std::endl;
    fout<<"The calculate state:"<<std::endl;
    for(int i=0;i<costArray.size();i++){
        fout<<"times:"<<i+1<<",currentCost:"<<costArray[i]<<std::endl;
    }
}