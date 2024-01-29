from math import pi, cos
import matplotlib.pyplot as plt
from numpy import linspace

def linearInterpolate(y1, y2, mu):
  return (y2 - y1) * mu + y1

def cosineInterpolate(y1, y2, mu):
  angle = mu * pi 
  mu2 = (1.0 - cos(angle)) * 0.5 # get new x-value based on cosine function
  return linearInterpolate(y1, y2, mu2)

def linear_interp_list(old_list, num_interp):
  num_old_points = len(old_list)
  new_list = [old_list[0]]
  for i in range(num_old_points-1):
    for j in range(1, num_interp+2):
      mu = float(j / (num_interp+1))
      lin_x = linearInterpolate(old_list[i], old_list[i+1], mu)
      new_list.append(lin_x)
  return new_list

def cosine_interp_list(old_list, num_interp):
  num_old_points = len(old_list)
  new_list = [old_list[0]]
  for i in range(num_old_points-1):
    for j in range(1, num_interp+2):
      mu = float(j / (num_interp+1))
      lin_x = cosineInterpolate(old_list[i], old_list[i+1], mu)
      new_list.append(lin_x)
  return new_list


def main():
  
  num_old_points = 10
  num_interp = 2

  oldx = linspace(1, num_old_points, num_old_points)
  oldy = [0, 5, 3, 8, -2, 1, 2, 9, 6, -3]
  newx = linear_interp_list(oldx, num_interp)
  # newy_linear = linear_interp_list(oldy, num_interp)
  newy_cosine = cosine_interp_list(oldy, num_interp)
  
  print(len(newy_cosine))
      
  plt.plot(oldx, oldy, 'ro-', label='original points')
  # plt.plot(newx, newy_linear, 'bo-', label='linear')
  plt.plot(newx, newy_cosine, 'go-', label='cosine')
  plt.legend()
  plt.grid()
  plt.show()
  
  
if __name__ == "__main__":
  main()