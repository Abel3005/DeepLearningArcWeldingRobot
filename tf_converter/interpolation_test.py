import csv
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import copy
import matplotlib.pyplot as plt
import random

class Interpolate_welding:
    
    def __init__(self, filename, test_flag):
        self.csvfile = open(filename, 'r')
        self.lookup_table = []
        self.lookup_list = []
        self.test_list = []
        self.make_lookup_list()
        if test_flag == False:
            self.make_lookup_table(len(self.lookup_list))
        return

    def make_lookup_list(self):
        del self.lookup_table[:]
        del self.lookup_list[:]
        rdr = csv.reader(self.csvfile)
        for line in rdr:
            self.lookup_list.append(line)
        return
    def make_lookup_table(self, num_points):
        random.shuffle(self.lookup_list)
        for i in range(0,num_points):
            key_point = np.array([float(self.lookup_list[i][1]), float(self.lookup_list[i][2]), float(self.lookup_list[i][3])])
            err_trans = np.array([float(self.lookup_list[i][4]), float(self.lookup_list[i][5]), float(self.lookup_list[i][6])])
            self.lookup_table.append([key_point, err_trans])
        return
    def randomdivide(self,num_testset):
        for i in range(0,num_testset):
            self.test_list.append(self.lookup_list.pop(random.randrange(0,len(self.lookup_list))))
        return
    def divide_test(self, method):
        result = []
        for i in range(0, len(self.test_list)):
            key_point = np.array([float(self.test_list[i][1]), float(self.test_list[i][2]), float(self.test_list[i][3])])
            err_trans = np.array([float(self.test_list[i][4]), float(self.test_list[i][5]), float(self.test_list[i][6])])
            result_err = err_trans - self.interpolate_point(key_point,method)
            result.append(result_err)
        return result
    def show_table_list(self):
        x_array = []
        y_array = []
        z_array = []
        x_err = []
        y_err = []
        z_err = []
        for i in range(0,len(self.lookup_list)):
            x_array.append(float(self.lookup_list[i][1]))
            y_array.append(float(self.lookup_list[i][2]))
            z_array.append(float(self.lookup_list[i][3]))
            x_err.append(float(self.lookup_list[i][4]))
            y_err.append(float(self.lookup_list[i][5]))
            z_err.append(float(self.lookup_list[i][6]))
        fig_x = plt.figure(figsize=(10,10))
        ax = fig_x.add_subplot(111, projection = '3d')
        bar_x = ax.scatter(x_array,y_array,z_array, c=x_err, marker='o', s=15, cmap='Greens')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        fig_x.colorbar(bar_x)
        fig_y = plt.figure(figsize=(10,10))
        ay = fig_y.add_subplot(111,projection = '3d')
        bar_y =ay.scatter(x_array,y_array,z_array, c =y_err, marker='o', s=15, cmap='Reds')
        ay.set_xlabel('X')
        ay.set_ylabel('Y')
        ay.set_zlabel('Z')
        fig_y.colorbar(bar_y)
        fig_z = plt.figure(figsize=(10,10))
        az = fig_z.add_subplot(111,projection = '3d')
        bar_z = az.scatter(x_array,y_array,z_array, c = z_err, marker='o', s=15, cmap ='Blues')
        az.set_xlabel('X')
        az.set_ylabel('Y')
        az.set_zlabel('Z')
        fig_z.colorbar(bar_z)
        plt.title('3D')
        plt.show()
    
        return
    def interpolate_point(self,point_cor, method):
        if len(self.lookup_table) == 0:
            return np.array([0,0,0])
        distance_list = []
        for point, err in self.lookup_table:
            distance_list.append([np.linalg.norm(point-point_cor),err])
        if method == 'nearest':
            _ , nearest_err = distance_list[distance_list.index(min(distance_list))]
            return nearest_err
        elif method == 'linear':
            dis_list = []
            trans_error_list =[]
            for i in range(0,4):
                distance , err = distance_list.pop(distance_list.index(min(distance_list)))
                dis_list.append(distance)
                trans_error_list.append(err)
            s_distance= sum(dis_list)
            linear_err = np.array([.0,.0,.0])
            for i in range(0,4):
                linear_err += ((1.0/dis_list[i]) * trans_error_list[i])
                print('err: ', trans_error_list[i])
                print('d hat: ', (1.0/dis_list[i]))
                print('linear err: ', linear_err)
            print('\n')
            linear_err /= s_distance
            return linear_err
def main():
    print('enter lookup csv filename')
    l_filename = raw_input()
    l_filename += '.csv'
    interp_module = Interpolate_welding(l_filename, True)
    interp_module.show_table_list()
    # print('select interpolate point number')
    # point_number = input()
    # if int(point_number) > len(interp_module.lookup_list):
    #     print("it's too big number to interpolte")
    #     return 
    # #select
    # print('select interpolate divide testset num')
    # set_number = input()
    # interp_module.randomdivide(int(set_number))
    # interp_module.make_lookup_table(int(point_number))
    # #interpolate
    # method = 'nearest'
    # result_csvfile = open('result_%s_%d.csv'%(method,point_number),'w')
    # result_wr = csv.writer(result_csvfile)
    # result_list= interp_module.divide_test(method)
    # for i in range(0,len(result_list)):
    #     result_wr.writerow([(i+1), str(result_list[i][0]), str(result_list[i][1]),str(result_list[i][2])])
    # return
if __name__ == '__main__':
    main()
