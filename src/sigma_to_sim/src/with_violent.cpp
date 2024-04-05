//暴力求解最优插入点作为对比
#include <iostream>
#include <vector>
#include <algorithm>
#include "insert_point_opti.hpp"

InsertOpti* insert;

// 评价函数，计算三个数的和
double evaluate(const std::vector<int>& combination) {
    double unit = insert->get_Unit(combination);
    return unit;
}

int main() {

    insert = new InsertOpti;

    // 生成所有可能的组合
    std::vector<std::vector<int>> combinations;
    for (int i = 0; i <= 100; ++i) {
        for (int j = i + 1; j <= 100; ++j) {
            for (int k = j + 1; k <= 100; ++k) {
                combinations.push_back({i, j, k});
            }
        }
    }

    // 计算每个组合的评估结果
    std::vector<double> evaluations;
    for (const auto& combination : combinations) {
        double result = evaluate(combination);
        evaluations.push_back(result);
    }

    // 找到评估结果最大的组合及其评估结果
    auto max_it = std::max_element(evaluations.begin(), evaluations.end());
    int max_index = std::distance(evaluations.begin(), max_it);
    std::vector<int> max_combination = combinations[max_index];
    double max_evaluation = *max_it;

    // 输出最大评估结果的组合及其评估结果
    std::cout << "最大评估结果的组合为：";
    for (int num : max_combination) {
        std::cout << num << " ";
    }
    std::cout << std::endl;
    std::cout << "评估结果为：" << max_evaluation << std::endl;



    delete insert;
    return 0;
}