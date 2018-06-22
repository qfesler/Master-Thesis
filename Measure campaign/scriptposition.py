import serial
import sys
import numpy
import matplotlib.pyplot as pyplot
import csv

def write_list_to_file(guest_list, filename):
    """Write the list to csv file."""
    with open(filename, 'wb') as f :
        writer = csv.writer(f)
        writer.writerow(guest_list)

            
Measures = []
looping = True
ser = serial.Serial('COM10')
distance = 5
row = 9
column = 20
while (looping):
    s = ser.readline()
    sys.stdout.write(s)
    if not ("end" in s):
        slist = s.rstrip()
        templist = eval(slist)
        if ((abs(templist[0]) < 3000) and (abs(templist[1]) <2000)):
            Measures.append(templist)
        #print(templist)
    if ("end" in s):
        Measures1 = [item[0] for item in Measures]
        Measures2 = [item[1] for item in Measures]
        
        pyplot.plot(Measures1,Measures2, 'r.')
        #Moyenne = numpy.mean(Measures)
        #Variance = numpy.var(Measures)
        #print("Moyenne : " + str(Moyenne) + "/Variance : " + str(Variance))
        pyplot.show()
        filename = "trajectory.csv"
        write_list_to_file(Measures, filename)
        #distance -= 0.1
        column-=1
        Measures = []
        #break
    
    
