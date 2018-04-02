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
while (looping):
    s = ser.readline()
    #sys.stdout.write(s)
    if not ("end" in s):
        slist = s.rstrip()
        templist = eval(slist)
        Measures.append(templist)
        print(templist)
    if ("end" in s):
        Measures1 = [item[0] for item in Measures]
        Measures2 = [item[1] for item in Measures]
        Measures3 = [item[2] for item in Measures]
        pyplot.plot(Measures1,'r.')
        pyplot.plot(Measures2,'b.')
        pyplot.plot(Measures3,'g.')
        #Moyenne = numpy.mean(Measures)
        #Variance = numpy.var(Measures)
        #print("Moyenne : " + str(Moyenne) + "/Variance : " + str(Variance))
        pyplot.show()
        #write_list_to_file(Measures, str(distance)+".csv")
        #distance -= 0.1
        Measures = []
    
    
