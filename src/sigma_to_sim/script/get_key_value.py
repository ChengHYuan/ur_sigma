def read_file(filename):
    # 打开文件
    file = open(filename, "r")
    
    # 存储每列数据的数组
    col1 = []
    col2 = []
    col3 = []
    
    # 逐行读取文件内容
    for line in file:
        # 分割每行数据
        data = line.strip().split()
        
        # 将数据添加到对应列的数组中
        col1.append(float(data[0]))
        col2.append(float(data[1]))
        col3.append(float(data[2]))
    
    # 关闭文件
    file.close()
    
    sorted_array1 = sorted(col1, reverse=True)
    sorted_array2 = sorted(col2, reverse=True)
    sorted_array3 = sorted(col3, reverse=True)
    a=int(180/5-1)
    b=int(180/5*4-1)
    print(a,b)
    key1_1=sorted_array1[a]
    key1_2=sorted_array1[b]
    key2_1=sorted_array2[a]
    key2_2=sorted_array2[b]
    key3_1=sorted_array3[a]
    key3_2=sorted_array3[b]
    
    # 返回每列数据的数组
    return key1_1, key1_2, key2_1, key2_2,key3_1, key3_2

# 示例用法
filename = "points/init_params.txt"  # 替换为你的文件名
key1_1, key1_2, key2_1, key2_2,key3_1, key3_2 = read_file(filename)

print(key1_1," ", key1_2,' ',key2_1,' ', key2_2,' ',key3_1, ' ',key3_2)