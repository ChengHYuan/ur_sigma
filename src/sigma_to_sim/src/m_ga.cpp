#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include "insert_point_opti.hpp"

// 定义问题维度和取值范围
const int DIMENSION = 3;
const int MIN_VALUE = 0;

// 定义个体类型
typedef std::vector<int> Individual;

// 定义种群类型
typedef std::vector<Individual> Population;

class GeneticAlgorithm {
private:
    int populationSize;
    int numGenerations;
    int numParents;
    int numOffspring;
    float mutationRate;

    InsertOpti* opti_ptr;

    int MAX_VALUE;

    Population population;

    int getRandomInt(int min, int max) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(min, max);
        return dis(gen);
    }
    float getRandomFloat(float min, float max) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(min, max);
        return dis(gen);
    }

    Population generateInitialPopulation() {
        Population initialPopulation(populationSize);
        for (int i = 0; i < populationSize; ++i) {
            Individual individual(DIMENSION);
            for (int j = 0; j < DIMENSION; ++j) {
                individual[j] = getRandomInt(MIN_VALUE, MAX_VALUE);
            }
            initialPopulation[i] = individual;
        }
        return initialPopulation;
    }

    // double calculateFitness(const Individual& individual) {
    //     // 在这里编写适应度函数的具体实现
    //     // 该函数应返回一个适应度值（double类型）
    //     return individual[0] + individual[1] + individual[2];
    // }

    // 计算适应度值
    float calculateFitness(const Individual& individual) {
        opti_ptr->get_Unit(individual);
    }

    std::function<double (const Individual&)> fitnessFunction;

    
    //轮盘赌
    Population selection1() {
        Population parents(numParents);
        float totalFitness = 0.0;

        // 计算适应度总和
        for (const Individual& individual : population) {
            totalFitness += individual[DIMENSION];
        }

        // 为每个父母选择个体
        for (int i = 0; i < numParents; ++i) {
            float spin = getRandomFloat(0, totalFitness);  // 生成轮盘赌的随机数
            float partialSum = 0.0;

            // 根据轮盘赌的随机数选择父母
            for (const Individual& individual : population) {
                partialSum += individual[DIMENSION];
                if (partialSum >= spin) {
                    parents[i] = individual;
                    break;
                }
            }
        }

        return parents;
    }
    
    //最优选择
    Population selection() {
        Population parents(numParents);

        // 选择适应度值最高的前numParents个个体作为父代
        for (int i = 0; i < numParents; ++i) {
            parents[i] = population[i];
        }

        return parents;
    }

    
    //交叉操作
    Population crossover(const Population& parents) {
        Population offspring(numOffspring);
        int parentSize = parents.size();
        for (int i = 0; i < numOffspring / 2; ++i) {
            Individual parent1 = parents[getRandomInt(0, parentSize - 1)];
            Individual parent2 = parents[getRandomInt(0, parentSize - 1)];

            // 第一个后代
            Individual child1(DIMENSION);
            for (int j = 0; j < DIMENSION; ++j) {
                child1[j] = parent1[j];
            }
            offspring[i] = child1;

            // 第二个后代
            Individual child2(DIMENSION);
            for (int j = 0; j < DIMENSION; ++j) {
                child2[j] = parent2[j];
            }
            offspring[i + numOffspring / 2] = child2;
        }
        return offspring;
    }

    void mutate(Individual& individual) {
        int index = getRandomInt(0, DIMENSION - 1);
        individual[index] = getRandomInt(MIN_VALUE, MAX_VALUE);
    }

public:
    //构造函数
    GeneticAlgorithm(int popSize, int numGen, int numPar, int numOff, float mutRate) {
        populationSize = popSize, numGenerations = numGen, numParents = numPar, numOffspring = numOff, mutationRate = mutRate;
        //初始化
        opti_ptr = new InsertOpti;
        MAX_VALUE = opti_ptr->SIZE;
        opti_ptr->get_all_param_independent();
        cout << "------------初始化完成-----------" << endl; 
    }
    
    void runGA() {
        // 生成初始种群
        population = generateInitialPopulation();
        for (int generation = 0; generation < numGenerations; ++generation) {
            // 计算每个个体的适应度值
            for (Individual& individual : population) {
                float fitness = calculateFitness(individual);
                // 将适应度值保存在个体中
                individual.push_back(fitness);
            }

            // 按适应度值降序排序
            std::sort(population.begin(), population.end(),
                [](const Individual& a, const Individual& b) {
                    return a[DIMENSION] > b[DIMENSION];
                });

            // 输出当前最佳个体的适应度值
            std::cout << "Generation " << generation << ", Best Fitness: "<< population[0][DIMENSION]<<" "<< population[0][0]<< " "<<population[0][1] <<" "<<population[0][2] << std::endl;

            // 选择操作
            Population parents = selection();

            // 交叉操作
            Population offspring = crossover(parents);

            // 变异操作
            for (Individual& individual : offspring) {
                if (getRandomInt(0, 100) < mutationRate * 100) {
                    mutate(individual);
                }
            }
            std::cout <<"population size: " <<population.size() << std::endl;
            // 更新种群
            population = offspring;
        }
    }
};


int main() {

    int populationSize = 100;
    int numGenerations = 100;
    int numParents = 50;
    int numOffspring = 100;
    float mutationRate = 0.1;

    GeneticAlgorithm ga(populationSize, numGenerations, numParents, numOffspring, mutationRate);

    ga.runGA();

    
    return 0;
}

