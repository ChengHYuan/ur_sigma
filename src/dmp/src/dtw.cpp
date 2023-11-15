//
// Created by jiyguo on 2022/3/22.
//

#include "dtw.h"
namespace dtw {

static const double psi = 0.01;
bool Confidances(const DVector& dtw_dists, vector<double> &confidances){
    size_t n_dtw = dtw_dists.size();
    double dtw_dist_min = *std::min_element(dtw_dists.begin(),dtw_dists.end());
    double beta = -log(psi)/(dtw_dist_min+0.01*psi);
    confidances.resize(n_dtw);
    vector<double> components(n_dtw);
    for (int i = 0; i < n_dtw; ++i) {
        components[i] = exp(-beta*dtw_dists[i]);
    }
    double sum = std::accumulate(components.begin(),components.end(),0.0);
    for (int i = 0; i < n_dtw; ++i) {
        confidances[i] = components[i] / sum;
    }
    return true;
}

Dtw::Dtw(const vector<DVector> &templt, const vector<DVector> &signl){
    this->dtw_table.assign(m_size,DVector(m_size,0));
    this->steps.assign(m_size,DVector(m_size,0));
    if (templt.empty() || signl.empty()){
        throw;
    }
    //添加
    for (const auto& data:templt) {
        dtw_templt.push_back(data);
    }
    for (const auto& data:signl) {
        dtw_signl.push_back(data);
    }

    size_t m = templt.size();
    size_t n = signl.size();
    row = m;
    colum = n;
    dtw_table[0][0] = distance(templt[0] , signl[0]);
    double cost;
    for (int i = 1; i < m; ++i){
        dtw_table[i][0] = dtw_table[i-1][0] + distance(templt[i] , signl[0]);
        steps[i][0] = steps[i-1][0] + 1;
    }
    for (int j = 1; j < n; ++j) {
        dtw_table[0][j] = dtw_table[0][j-1] + distance(templt[0] , signl[j]);
        steps[0][j] = steps[0][j-1] + 1;
    }
    for (int i = 1; i < m; ++i) {
        for (int j = 1; j < n; ++j) {
            cost = distance(templt[i] , signl[j]);
            if(this->dtw_table[i-1][j-1] <= dtw_table[i][j-1] && dtw_table[i-1][j-1] <= dtw_table[i-1][j]){
                steps[i][j] =  1 + steps[i-1][j-1];
                dtw_table[i][j] = dtw_table[i-1][j-1] + cost;
            }else if(this->dtw_table[i][j-1] <= dtw_table[i-1][j-1] && dtw_table[i][j-1] <= dtw_table[i-1][j]){
                steps[i][j] =  1 + steps[i][j-1];
                dtw_table[i][j] = dtw_table[i][j-1] + cost;
            }else{
                steps[i][j] =  1 + steps[i-1][j];
                dtw_table[i][j] =  dtw_table[i-1][j] + cost;
            }
        }
    }
}

Dtw::Dtw(const DVector& start){
    this->dtw_table.assign(m_size,DVector(m_size,0));
    this->steps.assign(m_size,DVector(m_size,0));
    this->dtw_table[0][0] = 0;
    row = 1;
    colum = 1;
    dtw_templt.push_back(start);
    dtw_signl.push_back(start);
}

Dtw::Dtw(){
    dtw_table.assign(m_size,DVector(m_size,0));
    steps.assign(m_size,DVector(m_size,0));
    row = 0;
    colum = 0;
}

// 仅扩充模板
bool Dtw::add_temp(const vector<DVector> &temp_vec){
    if (temp_vec.empty()){
        return false;
    }
    //添加
    for (const auto& data:temp_vec) {
        dtw_templt.push_back(data);
    }
    size_t m = temp_vec.size();
    size_t begin_i = row;
    row += m;
    for (size_t i = begin_i; i < row; ++i){
        dtw_table[i][0] = dtw_table[i-1][0] + distance(dtw_templt[i] , dtw_signl[0]);
        steps[i][0] = steps[i-1][0] + 1;
    }
    for (size_t i = begin_i; i < row; ++i) {
        for (size_t j = 1; j < colum; ++j) {
            update(i , j);
        }
    }
    return true;
}

bool Dtw::add_temp(const DVector &temp_num){
    vector<DVector> temp_vec(1,temp_num);
    return add_temp(temp_vec);
}

// 仅扩充信号
bool Dtw::add_sign(const vector<DVector> &sign_vec){
    if (sign_vec.empty()){
        return false;
    }
    //添加
    for (const auto& data:sign_vec) {
        dtw_signl.push_back(data);
    }
    size_t n = sign_vec.size();
    size_t begin_j = colum;
    colum += n;
    for (size_t j = begin_j; j < colum; ++j) {
        dtw_table[0][j] = dtw_table[0][j-1] + distance(dtw_templt[0] , dtw_signl[j]);
        steps[0][j] = steps[0][j-1] + 1;
    }
    for (size_t i = 1; i < row; ++i) {
        for (size_t j = begin_j; j < colum; ++j) {
            update(i , j);
        }
    }
    return true;
}

bool Dtw::add_sign(const DVector &sign_num){
    vector<DVector> sign_vec(1,sign_num);
    return add_sign(sign_vec);
}

// 以数据序列的形式，往模板和数据里同时添加新的数据
bool Dtw::add_data(const vector<DVector>& temp_vec, const vector<DVector>& sign_vec){
    if (temp_vec.empty() && sign_vec.empty()){
        return false;
    }
    //如果有空的，分包给其他函数
    if (temp_vec.empty()) return add_sign(sign_vec);
    if (sign_vec.empty()) return add_temp(temp_vec);

    //添加
    for (const auto& data:temp_vec) {
        dtw_templt.push_back(data);
    }
    for (const auto& data:sign_vec) {
        dtw_signl.push_back(data);
    }
    size_t m = temp_vec.size();
    size_t n = sign_vec.size();
    size_t begin_i = row;
    size_t begin_j = colum;
    double cost;
    row += m;
    colum += n;
    for (size_t i = begin_i; i < row; ++i){
        dtw_table[i][0] = dtw_table[i-1][0] + distance(dtw_templt[i] , dtw_signl[0]);
        steps[i][0] = steps[i-1][0] + 1;
    }
    for (size_t j = begin_j; j < colum; ++j) {
        dtw_table[0][j] = dtw_table[0][j-1] + distance(dtw_templt[0] , dtw_signl[j]);
        steps[0][j] = steps[0][j-1] + 1;
    }
    for (size_t i = begin_i; i < row; ++i) {
        for (size_t j = 1; j < begin_j; ++j) {
            update(i , j);
        }
    }
    for (size_t i = 1; i < row; ++i) {
        for (size_t j = begin_j; j < colum; ++j) {
            update(i , j);
        }
    }
    return true;
}

bool Dtw::add_data(const DVector& temp_num, const DVector& sign_num){
    vector<DVector> temp_vec(1,temp_num);
    vector<DVector> sign_vec(1,sign_num);
    return add_data(temp_vec, sign_vec);
}

bool Dtw::add_data(const DVector &temp_num, const vector<DVector> &sign_vec){
    vector<DVector> temp_vec(1,temp_num);
    return add_data(temp_vec, sign_vec);
}

bool Dtw::add_data(const vector<DVector> &temp_vec, const DVector &sign_num){
    vector<DVector> sign_vec(1,sign_num);
    return add_data(temp_vec, sign_vec);
}

// 获取dtw距离，总距离/总步长
double Dtw::get_distance() const{
    return dtw_table[this->row-1][this->colum-1] / steps[this->row-1][this->colum-1];
}

// 更新两个表中的(i,j)
void Dtw::update(size_t i , size_t j){
    double cost = distance(dtw_templt[i] , dtw_signl[j]);
    if(dtw_table[i-1][j-1] <= dtw_table[i][j-1] && dtw_table[i-1][j-1] <= dtw_table[i-1][j]){
        steps[i][j] =  1 + steps[i-1][j-1];
        dtw_table[i][j] = f_factor * dtw_table[i-1][j-1] + cost;
    }else if(dtw_table[i][j-1] <= dtw_table[i-1][j-1] && dtw_table[i][j-1] <= dtw_table[i-1][j]){
        steps[i][j] =  1 + steps[i][j-1];
        dtw_table[i][j] = f_factor * dtw_table[i][j-1] + cost;
    }else{
        steps[i][j] =  1 + steps[i-1][j];
        dtw_table[i][j] =  f_factor * dtw_table[i-1][j] + cost;
    }
}

// 计算欧式距离
double distance(const DVector& x , const DVector& y){
    double ans = 0;
    for (int i = 0; i < x.size(); ++i) {
        ans += pow(x[i] - y[i],2);
    }
    return std::sqrt(ans);
}

}