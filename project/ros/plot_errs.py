import matplotlib.pyplot as plt
import numpy as np
import os

robot_names = []
errors = []

with open(os.path.expanduser("~/.ros/errors.txt"), "r") as f:
  list_of_errors = f.readlines()[0].split("}")[:-1]
  list_of_errors = [error+"}" for error in list_of_errors]
  for error in list_of_errors:
    for k, v in eval(error).items():
      robot_names.append(k)
      errors.append(v)

sampled_errs = [errors[i][::20] for i in range(len(errors))]

x = np.arange(min_len)
for i in range(len(sampled_errs)):
  plt.plot(x, sampled_errs[i])

plt.xlabel('Time')
plt.ylabel('Error')
plt.legend(robot_names)

os.remove(os.path.expanduser("~/.ros/errors.txt"))

plt.show()
