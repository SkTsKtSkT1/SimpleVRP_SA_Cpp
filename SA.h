//
// Created by lzw on 2022/3/8.
//

#ifndef SA_SA_H
#define SA_SA_H

#include "vector"
#include "iostream"
#include "random"

class Model{
private:
    int _point=60;
    int _drone=3;
    int _x0=100;
    int _y0=0;
    std::vector<std::vector<double>> _distanceMap
            =std::vector<std::vector<double>>((_point+_drone),
                    std::vector<double>(_point+_drone));
    std::vector<std::vector<double>> _pointMap=
            std::vector<std::vector<double>>(2,std::vector<double>
                    ((_point+_drone)));

public:
    Model(int point,int drone) {
        _point=point;
        _drone=drone;
        _distanceMap.resize(_drone+_point);
        for(int i=0;i<_distanceMap.size();i++){
            _distanceMap[i].resize(_drone+_point);
        }

        for(int i=0;i<_pointMap.size();i++){
            _pointMap[i].resize(_drone+_point);
        }
    }
    void createMap();
    void calculateDismap();
    std::vector<std::vector<double>> getPointMap(){
        return _pointMap;
    }
    std::vector<std::vector<double>> getDistanceMap(){
        return _distanceMap;
    }
    int getPoint(){
        return _point;
    }
    int getDrone(){
        return _drone;
    }
    void printVector(std::vector<std::vector<double>> vec){
        for(std::vector<std::vector<double>>::iterator it=vec.begin();
        it!=vec.end();it++){
            for(int i=0;i<(*it).size();i++){
                std::cout<<(*it)[i]<<" ";
            }
            std::cout<<std::endl;
        }
    }
};

class SA_VRP {
private:
    double _T0=3000;
    double _r=0.997;
    double _Ts=0.01;
    int _Lk=300;//每次新解的链长度
    int _maxIter=5000;
public:
    SA_VRP(){};
    SA_VRP(int T0,int r,int Ts,int Lk,int max){
        _T0=T0;
        _r=r;
        _Ts=Ts;
        _Lk=Lk;
        _maxIter=max;
    }

    std::vector<int> saVpr(Model model,std::vector<double> &costArray,double &minCost);

    std::vector<int> randomSol(Model model);

    double calculateCost(std::vector<int>  route, Model model);

    std::vector<int> createNeighbor(std::vector<int> route, Model model, int mode);

    bool isFeasible(std::vector<int>  route, Model model);

    void printVector(std::vector<int> vec) {
        for (std::vector<int>::iterator it = vec.begin();
             it != vec.end(); it++) {

            std::cout << (*it) << " ";
        }
        std::cout << std::endl;
    }

    void printResult(std::string const str,std::vector<double> const costArray,double const minCost,Model model,double costTime, std::vector<int> minRoute);

};
#endif //SA_SA_H
