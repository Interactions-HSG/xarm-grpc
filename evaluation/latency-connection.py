import subprocess
from statistics import mean, median, stdev

xarm_IP = "130.82.171.9"

def extract_response_values(response):
    response_arr = response.split();
    index = response_arr.index("TIME-1:")
    time_connection_manager = response_arr[index+1]
    time_send_command = response_arr[index+3]
    return time_connection_manager, time_send_command

N = 15
print("Volume: ", N)
ping_num = '50'
print("Ping packets: ", ping_num, "\n")

time_arr1 =[]
time_arr2 =[]

for n in range(N):
    response = subprocess.check_output(["xarm-commander",'-i', xarm_IP, 'get_state'], encoding = 'ascii')
    time1, time2 = extract_response_values(response);
    time_arr1.append(float(time1)/1000)
    time_arr2.append(float(time2)/1000)

    if not(n == 0):
        if n % 10 == 0:
            print("Progress: (", n, "/", N, ") ~ ", int((n/N)*100), "%")

max1 = max(time_arr1)
max2 = max(time_arr2)
min1 = min(time_arr1)
min2 = min(time_arr2)
mean1 = mean(time_arr1)
mean2 = mean(time_arr2)
med1 = median(time_arr1)
med2 = median(time_arr2)
stdev1 = stdev(time_arr1)
stdev2 = stdev(time_arr2) 

print("\nTime Connection Manager [ms]: ", "\n     Min: ", min1,"\n     Max: ", max1, "\n     Mean: ", mean1, "\n     Median: ", med1, "\n     Stdev:", stdev1, "\n");
print("Time Sending Command [ms]: ", "\n     Min: " , min2,"\n     Max: ", max2, "\n     Mean: ", mean2, "\n     Median: ", med2, "\n     Stdev:", stdev2, "\n");


response_ping = subprocess.check_output(["ping", '-c', ping_num, xarm_IP], encoding = 'ascii', timeout=50)

print("Ping: ", response_ping.split("rtt ")[1])
