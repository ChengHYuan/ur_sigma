#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include "insert_point_opti.hpp"

// 定义问题维度和取值范围
const int DIMENSION = 3;
const int MIN_VALUE = 0;

// 定义个体类型

struct Individual {
    std::vector<int> values;
    double fitness;

    Individual() : fitness(0.0) {}
    Individual(std::vector<int> v, double f) : values(v), fitness(f) {}
};


// 定义种群类型
typedef std::vector<Individual> Population;

class GeneticAlgorithm {
private:
    int populationSize;
    int numGenerations;
    int numParents;
    int numOffspring;
    float mutationRate;

    std::ofstream best_pointsFile;
    std::ofstream fitness_all;

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
            std::vector<int> values(DIMENSION);
            for (int j = 0; j < DIMENSION; ++j) {
                values[j] = getRandomInt(MIN_VALUE, MAX_VALUE);
            }
            initialPopulation[i] = Individual(values, 0.0);
        }
        return initialPopulation;
    }

    // double calculateFitness(const Individual& individual) {
    //     // 在这里编写适应度函数的具体实现
    //     // 该函数应返回一个适应度值（double类型）
    //     return individual[0] + individual[1] + individual[2];
    // }

    // 计算适应度值
    double calculateFitness(const Individual& individual) {
        return opti_ptr->get_Unit(individual.values);
    }

    std::function<double (const Individual&)> fitnessFunction;

    
    //轮盘赌
    Population selection1() {
        Population parents(numParents);
        float totalFitness = 0.0;

        // 计算适应度总和
        for (const Individual& individual : population) {
            totalFitness -= individual.fitness;
        }

        // 为每个父母选择个体
        for (int i = 0; i < numParents; ++i) {
            float spin = getRandomFloat(0, totalFitness);  // 生成轮盘赌的随机数
            float partialSum = 0.0;

            // 根据轮盘赌的随机数选择父母
            for (const Individual& individual : population) {
                partialSum -= individual.fitness;
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
            std::vector<int> childValues(DIMENSION);
            for (int j = 0; j < DIMENSION; ++j) {
                if (getRandomFloat(0.0, 1.0) < 0.5) {
                    childValues[j] = parent1.values[j];
                }
                else {
                    childValues[j] = parent2.values[j];
                }
            }
            double childFitness = calculateFitness(Individual(childValues, 0.0));
            offspring[i] = Individual(childValues, childFitness);

            // 第二个后代
            std::vector<int> childValues2(DIMENSION);
            for (int j = 0; j < DIMENSION; ++j) {
                if (getRandomFloat(0.0, 1.0) < 0.5) {
                    childValues2[j] = parent2.values[j];
                }
                else {
                    childValues2[j] = parent1.values[j];
                }
            }
            double childFitness2 = calculateFitness(Individual(childValues2, 0.0));
            offspring[i + numOffspring / 2] = Individual(childValues2, childFitness2);
        }
        return offspring;
    }

    void mutate(Individual& individual) {
        int index = getRandomInt(0, DIMENSION - 1);
        individual.values[index] = getRandomInt(MIN_VALUE, MAX_VALUE);
    }

    // 计算均值和标准差的函数
    void calculateMeanAndStdDev(const std::vector<Individual>& population, double& mean, double& stdDev) {
        int size = population.size();
        double sum = 0.0;
        double sumSquaredDifferences = 0.0;

        // 计算均值
        for (const Individual& individual : population) {
            sum += individual.fitness;
        }
        mean = sum / size;

        // 计算标准差
        for (const Individual& individual : population) {
            sumSquaredDifferences += (individual.fitness - mean) * (individual.fitness - mean);
        }
        stdDev = sqrt(sumSquaredDifferences / (size-1));
    }

public:
    //构造函数
    GeneticAlgorithm(int popSize, int numGen, int numPar, int numOff, float mutRate, const std::string& name) {
        populationSize = popSize, numGenerations = numGen, numParents = numPar, numOffspring = numOff, mutationRate = mutRate;
        //初始化
        opti_ptr = new InsertOpti;
        
        MAX_VALUE = opti_ptr->SIZE-1;


        
        // opti_ptr->get_all_param_independent();


        // std::vector<std::vector<double>> combinedArray;
        // for (size_t i = 0; i < opti_ptr->orient_params.size(); i++) {
        //     std::vector<double> row;
        //     row.push_back(opti_ptr->reach_params[i]);
        //     row.push_back(opti_ptr->orient_params[i]);
        //     row.push_back(opti_ptr->joint_lim_params[i]);
        //     combinedArray.push_back(row);
        // }
        // saveArrayToFile(combinedArray, "points/init_params.txt");

        best_pointsFile.open(name);
        fitness_all.open("fitness_all.txt");


        
        std::vector<std::vector<double>> combinedArray;
        combinedArray = loadArrayFromFile("points/init_params.txt");
        for (size_t i = 0; i < combinedArray .size(); i++) {
            opti_ptr->reach_params[i] = combinedArray[i][0];
            opti_ptr->orient_params[i] = combinedArray[i][1];
            opti_ptr->joint_lim_params[i] = combinedArray[i][2];
        }
        opti_ptr->set_key_values();


        cout << "------------初始化完成-----------" << endl;
    }

    ~GeneticAlgorithm() {
        best_pointsFile.close();
        fitness_all.close();
        delete opti_ptr;
    }


    
    void runGA() {
        // 生成初始种群
        // std::cout <<"flag0"<< std::endl;
        population = generateInitialPopulation();
        
        for (int generation = 0; generation < numGenerations; ++generation) {
            // 计算每个个体的适应度值
            for (Individual& individual : population) {
                // std::cout <<individual[0]<<" "<<individual[1]<<" "<<individual[2]<< std::endl;
                double fitness = calculateFitness(individual);
                // 将适应度值保存在个体中
                

                individual.fitness=fitness;
            }
            
            // std::cout <<"flag1"<< std::endl;
            // 按适应度值降序排序
            std::sort(population.begin(), population.end(),
                [](const Individual& a, const Individual& b) {
                    return a.fitness > b.fitness;
                });

            

            // 输出当前最佳个体的适应度值
            std::cout << "Generation " << generation << ", Best Fitness: " << population[0].fitness << " " << population[0].values[0] <<
                " " << population[0].values[1] << " " << population[0].values[2] << std::endl;
            // std::cout<<" "<<std::endl;
            // std::cout << "--------------------------" << std::endl;

            // 选择操作
            Population parents = selection();
            // std::cout <<"flag2"<< std::endl;

            // 交叉操作
            Population offspring = crossover(parents);

            // std::cout <<"flag3"<< std::endl;

            // 变异操作
            for (Individual& individual : offspring) {
                if (getRandomInt(0, 100) < mutationRate * 100) {
                    mutate(individual);
                }
            }

            //存储每一行数据，包括信息：代数、平均适应度、标准差、最优适应度、点索引（e l r）、点坐标
            double mean = 0.0;
            double stdDev = 0.0;
            calculateMeanAndStdDev(population,mean,stdDev);
            best_pointsFile << generation << " " << mean << " " << stdDev << " " << population[0].fitness << ","
                << population[0].values[0] << " " << population[0].values[1] << " " << population[0].values[2] << ","
                << opti_ptr->m_invert_points[population[0].values[0]][0] << " " << opti_ptr->m_invert_points[population[0].values[0]][1] << " " << opti_ptr->m_invert_points[population[0].values[0]][2] << ","
                << opti_ptr->m_invert_points[population[0].values[1]][0] << " " << opti_ptr->m_invert_points[population[0].values[1]][1] << " " << opti_ptr->m_invert_points[population[0].values[1]][2] << ","
                << opti_ptr->m_invert_points[population[0].values[2]][0] << " " << opti_ptr->m_invert_points[population[0].values[2]][1] << " " << opti_ptr->m_invert_points[population[0].values[2]][2] << "\n";
            std::cout << "population size: " << population.size() << std::endl;

            for (const Individual& individual : population) {
                fitness_all << individual.fitness << " ";
            }
            fitness_all << "\n";

            // 更新种群
            population = offspring;
        }
    }
    
    //存储文件
    void saveArrayToFile(const std::vector<std::vector<double>>& array, const std::string& filename) {
        std::ofstream file(filename);
        if (file.is_open()) {
            for (const auto& row : array) {
                for (const auto& num : row) {
                    file << num << " ";
                }
                file << std::endl;
            }
            file.close();
            std::cout << "数组已成功保存到文件：" << filename << std::endl;
        }
        else {
            std::cerr << "无法打开文件：" << filename << std::endl;
        }
    }

    std::vector<std::vector<double>> loadArrayFromFile(const std::string& filename) {
        std::vector<std::vector<double>> loadedArray;
        std::ifstream file(filename);
        if (file.is_open()) {
            double num;
            std::vector<double> row;
            while (file >> num) {
                row.push_back(num);
                if (row.size() == 3) {
                    loadedArray.push_back(row);
                    row.clear();
                }
            }
            file.close();
            std::cout << "文件中的数据已成功加载。" << std::endl;
        }
        else {
            std::cerr << "无法打开文件：" << filename << std::endl;
        }

        return loadedArray;
    }

    
};


int main() {

    int populationSize = 100;
    int numGenerations = 100;
    int numParents = 10;
    int numOffspring = 100;
    float mutationRate = 0.6;

    GeneticAlgorithm ga(populationSize, numGenerations, numParents, numOffspring, mutationRate,"best_points.txt");

    ga.runGA();

    
    return 0;
}

