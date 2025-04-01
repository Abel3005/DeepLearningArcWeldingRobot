import csv

g_list = []
t_list = []

def main():
    
    #set filename
    print('enter ground csvfile name')
    g_csvname = raw_input() 
    g_csvname += '.csv'
    g_csv  = open(g_csvname,'r')
    print('enter test csvfile name')
    t_csvname = raw_input() 
    t_csvname += '.csv'
    t_csv = open(t_csvname, 'r')
    g_rdr =  csv.reader(g_csv)
    t_rdr =  csv.reader(t_csv)

    for line in g_rdr:
        g_list.append(line)
        print(line)
    for line in t_rdr:
        t_list.append(line)
        print(line)
    if len(g_list) != len(t_list):
        return
    with open('lookup.csv' , 'w') as lookupfile:
        wr = csv.writer(lookupfile)
        for idx in range(0,len(g_list)):
            x, y, z =(float(g_list[idx][1]), float(g_list[idx][2]), float(g_list[idx][3]))
            err_x, err_y, err_z = (x - float(t_list[idx][1]), y- float(t_list[idx][2]), z - float(t_list[idx][3]))
            wr.writerow([(idx+1),str(x),str(y),str(z),str(err_x),str(err_y),str(err_z)])
    return

if __name__ == '__main__':
    main()