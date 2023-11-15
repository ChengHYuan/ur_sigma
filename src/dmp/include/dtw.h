//
// Created by jiyguo on 2022/3/22.
//

#ifndef SRC_DTW_H
#define SRC_DTW_H
#include "vector"
#include <cmath>
#include "algorithm"
#include <numeric>
using namespace std;
namespace dtw{

const int m_size = 2000;
//const double f_factor = 0.999;
const double f_factor = 1;


typedef vector<double> DVector;
double distance(const DVector& x,const DVector& y);
class Dtw {
private:
    vector<DVector> dtw_table;
    vector<DVector> steps;
    vector<DVector> dtw_templt;
    vector<DVector> dtw_signl;
    size_t row, colum;

    void update(size_t i , size_t j);
public:
    Dtw(const vector<DVector>& templt, const vector<DVector> & signl);
    Dtw();
    explicit Dtw(const DVector& start);

    bool add_temp(const vector<DVector>& temp_serial);
    bool add_temp(const DVector& temp_num);
    bool add_sign(const vector<DVector>& sign_serial);
    bool add_sign(const DVector& sign_num);
    bool add_data(const vector<DVector> & temp_serial, const vector<DVector> & sign_serial);
    bool add_data(const DVector& temp_num, const DVector& sign_num);
    bool add_data(const vector<DVector>& temp_serial, const DVector& sign_num);
    bool add_data(const DVector& temp_num, const vector<DVector>& sign_serial);

    double get_distance() const;
};

bool Confidances(const DVector& dtw_dists, vector<double> &confidances);

}

#endif //SRC_DTW_H
