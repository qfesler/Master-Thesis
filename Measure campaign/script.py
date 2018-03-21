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
    sys.stdout.write(s)
    if not ("end" in s):
        s.rstrip()
        templist = eval(s)
        Measures.append(s)
    if ("end" in s):
        pyplot.plot(Measures[:][0],'r.')
        pyplot.plot(Measures[:][1],'b.')
        pyplot.plot(Measures[:][2],'g.')
        #Moyenne = numpy.mean(Measures)
        #Variance = numpy.var(Measures)
        #print("Moyenne : " + str(Moyenne) + "/Variance : " + str(Variance))
        pyplot.show()
        #write_list_to_file(Measures, str(distance)+".csv")
        #distance -= 0.1
        Measures = []
    
    
