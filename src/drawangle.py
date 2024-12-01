import matplotlib.pyplot as plt

import json

circle_left_values = []
circle_right_values = []
time_values = []

with open("circle_left_values.txt", 'r') as fr:
    circle_left_values = json.load(fr)

with open("circle_right_values.txt", 'r') as fr:
    circle_right_values = json.load(fr)

with open("time_values.txt", 'r') as fr:
    time_values = json.load(fr)

# print(data_time[0])

plt.plot(time_values, circle_left_values)
plt.plot(time_values, circle_right_values)
plt.title("angle(t)")
plt.ylabel("angle")
plt.xlabel("t")
plt.show()  

 
