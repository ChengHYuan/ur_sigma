//
// Created by jiy on 2021/11/23.
//
#include "confidence.h"

using std::vector;

// small positive constant
static const double psi = 0.01;

// steps of shortest path
static int Traceback(const vector<vector<double>>& dtw,int x, int y){
    if(x==0 && y==0) return 0;
    if(x==0) {
        return Traceback(dtw,x,y-1)+1;
    }
    if(y==0){
        return Traceback(dtw,x-1,y)+1;
    }
    if(dtw[x-1][y-1]<dtw[x][y-1] && dtw[x-1][y-1]<dtw[x-1][y]){
        return 1 + Traceback(dtw, x - 1, y - 1);
    }else if(dtw[x][y-1]<dtw[x-1][y-1] && dtw[x][y-1]<dtw[x-1][y]){
        return 1 + Traceback(dtw, x, y - 1);
    }else{
        return 1 + Traceback(dtw, x-1, y);
    }
}

// DP for dtw distance
static double DTWDistance(const std::vector<dmp::DMPPoint>& temp_dmp, const std::vector<dmp::DMPPoint>& sign_dmp){
    size_t m = temp_dmp.size();
    size_t n = sign_dmp.size();
    std::vector<std::vector<double>> temp(2,std::vector<double>(m));
    std::vector<std::vector<double>> sign(2,std::vector<double>(n));
    int begin_m = 0;
    int begin_n = 0;
    int threth_m = m;
    int threth_n = n;
    if(m>threth_m){
        begin_m = m-threth_m;
    }
    if(n>threth_n){
        begin_n = n-threth_n;
    }

    for (int i = begin_m; i < m; ++i) {
        temp[0][i-begin_m] = temp_dmp[i].positions[0];
        temp[1][i-begin_m] = temp_dmp[i].positions[1];
    }
    for (int i = begin_n; i < n; ++i) {
        sign[0][i-begin_n] = sign_dmp[i].positions[0];
        sign[1][i-begin_n] = sign_dmp[i].positions[1];
    }

    vector<vector<double>> dtw(m-begin_m,vector<double>(n-begin_n,1000));
    vector<vector<double>> steps(m-begin_m,vector<double>(n-begin_n,0));
    dtw[0][0] = std::sqrt(pow(temp[0][0]-sign[0][0],2) + pow(temp[1][0]-sign[1][0],2));;
    double cost;
    for (int i = 1; i < m-begin_m; ++i){
        cost = std::sqrt(pow(temp[0][i]-sign[0][0],2) + pow(temp[1][i]-sign[1][0],2));
        dtw[i][0] = dtw[i-1][0] + cost;
        steps[i][0] = steps[i-1][0] + 1;
    }
    for (int j = 1; j < n-begin_n; ++j) {
        cost = std::sqrt(pow(temp[0][0]-sign[0][j],2) + pow(temp[1][0]-sign[1][j],2));
        dtw[0][j] = dtw[0][j-1] + cost;
        steps[0][j] = steps[0][j-1] + 1;
    }
    for (int i = 1; i < m-begin_m; ++i) {
        for (int j = 1; j < n-begin_n; ++j) {
            cost = std::sqrt(pow(temp[0][i]-sign[0][j],2) + pow(temp[1][i]-sign[1][j],2));
//            dtw[i][j] = cost + std::min(dtw[i-1][j-1],std::min(dtw[i][j-1],dtw[i-1][j]));
            if(dtw[i-1][j-1]<=dtw[i][j-1] && dtw[i-1][j-1]<=dtw[i-1][j]){
                steps[i][j] =  1 + steps[i-1][j-1];
                dtw[i][j] = cost +dtw[i-1][j-1];
            }else if(dtw[i][j-1]<=dtw[i-1][j-1] && dtw[i][j-1]<=dtw[i-1][j]){
                steps[i][j] =  1 + steps[i][j-1];
                dtw[i][j] =  cost + dtw[i][j-1];
            }else{
                steps[i][j] =  1 + steps[i-1][j];
                dtw[i][j] =  cost + dtw[i-1][j];
            }
        }
    }
//    int num = Traceback(dtw,m-1-begin_m,n-1-begin_n);
//    double ans = dtw[m-1-begin_m][n-1-begin_n]/num;
//    return ans;
    return dtw[m-1-begin_m][n-1-begin_n] / steps[m-1-begin_m][n-1-begin_n];

//    return dtw[m-1-begin_m][n-1-begin_n] / steps[m-1-begin_m][n-1-begin_n];
}

// DP for dtw distance
static double DTWDistance(const std::vector<vector<double>>& temp, const std::vector<vector<double>>& sign){
    size_t m = temp[0].size();
    size_t n = sign[0].size();
    vector<vector<double>> dtw(m,vector<double>(n,1000));
    dtw[0][0] = std::sqrt(pow(temp[0][0]-sign[0][0],2) + pow(temp[1][0]-sign[1][0],2));;
    double cost;
    for (int i = 1; i < m; ++i){
        cost = std::sqrt(pow(temp[0][i]-sign[0][0],2) + pow(temp[1][i]-sign[1][0],2));
        dtw[i][0] = dtw[i-1][0] + cost;
    }
    for (int j = 1; j < n; ++j) {
        cost = std::sqrt(pow(temp[0][0]-sign[0][j],2) + pow(temp[1][0]-sign[1][j],2));
        dtw[0][j] = dtw[0][j-1] + cost;
    }
    for (int i = 1; i < m; ++i) {
        for (int j = 1; j < n; ++j) {
            cost = std::sqrt(pow(temp[0][i]-sign[0][j],2) + pow(temp[1][i]-sign[1][j],2));
            dtw[i][j] = cost + std::min(dtw[i-1][j-1],std::min(dtw[i][j-1],dtw[i-1][j]));
        }
    }
//    int num = Traceback(dtw,m-1,n-1);
//    double ans = dtw[m-1][n-1]/num;
//    return ans;
    return dtw[m-1][n-1];
}

// calculate confidances between all templates and signal

vector<double> Confidances(const vector<vector<dmp::DMPPoint>>& temps, const vector<dmp::DMPPoint>& sign){

    vector<double> dtw_dists(temps.size());
    for (int i=0;i<temps.size();++i) {
        dtw_dists[i] = DTWDistance(temps[i],sign);
    }
    double dtw_dist_min = *std::min_element(dtw_dists.begin(),dtw_dists.end());
    double beta = -log(psi)/dtw_dist_min;
    vector<double> confidances(temps.size());
    vector<double> components(temps.size());
    for (int i = 0; i < temps.size(); ++i) {
        components[i] = exp(-beta*dtw_dists[i]);
    }
    double sum = std::accumulate(components.begin(),components.end(),0.0);
    for (int i = 0; i < temps.size(); ++i) {
        confidances[i] = components[i] / sum;
    }
    return confidances;
}

vector<double> Confidances(const std::vector<vector<vector<double>>>& temps, const std::vector<vector<double>>& sign){

    vector<double> dtw_dists(temps.size());
    for (int i=0;i<temps.size();++i) {
        dtw_dists[i] = DTWDistance(temps[i],sign);
    }
    double dtw_dist_min = *std::min_element(dtw_dists.begin(),dtw_dists.end());
    double beta = -log(psi)/dtw_dist_min;
    vector<double> confidances(temps.size());
    vector<double> components(temps.size());
    for (int i = 0; i < temps.size(); ++i) {
        components[i] = exp(-beta*dtw_dists[i]);
    }
    double sum = std::accumulate(components.begin(),components.end(),0.0);
    for (int i = 0; i < temps.size(); ++i) {
        confidances[i] = components[i] / sum;
    }
    return confidances;
}


