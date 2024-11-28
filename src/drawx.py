import matplotlib.pyplot as plt

import json

x_values = []
y_values = []

with open("robot_simulation_course_work/src/x_values.txt", 'r') as fr:
    x_values = json.load(fr)

with open("robot_simulation_course_work/src/y_values.txt", 'r') as fr:
    y_values = json.load(fr)

# print(data_time[0])

plt.plot(x_values, y_values)
plt.title("y(x)")
plt.ylabel("y")
plt.xlabel("x")
plt.show()  

 
