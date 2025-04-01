import csv

def main():
    testfile = open('test.csv', 'r')
    groundtruthfile = open('ground_truth.csv', 'r')
    test_rdr = csv.reader(testfile)
    ground_rdr = csv.readeer(grpundtruthfile)
    
    test_list = []
    for line in test_rdr:
        print(line)
    

if __name__ == '__main__':
    main()
