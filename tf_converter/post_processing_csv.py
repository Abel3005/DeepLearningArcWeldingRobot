import csv

def main():
    csvfile = open('test.csv','r')
    rdr = csv.reader(csvfile)
    csv_list = []
    for line in rdr:
        csv_list.append([float(line[1])-0.402794, float(line[2])-0.00258351, float(line[3])+0.156928])
    writecsvfile = open('test2.csv','w')
    wr = csv.writer(writecsvfile)
    for i in range(0,len(csv_list)):
        wr.writerow([(i+1),str(csv_list[i][0]), str(csv_list[i][1]),str(csv_list[i][2])])
    return

if __name__ == '__main__':
    main()