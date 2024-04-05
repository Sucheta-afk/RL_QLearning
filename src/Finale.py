#!/usr/bin/python3

import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist

env_row = 20
env_col = 20

# define actions
# numeric action codes: 0 = up, 1 = right, 2 = down, 3 = left
actions = ['up', 'right', 'down', 'left']

q_values = np.zeros((env_row, env_col, len(actions)))

# initialise rewards
rewards = np.full((env_row, env_col), -100.)


def get_euclidian_dist(x1,y1,x2,y2):
    return np.sqrt((x1-x2)**2 + (y1-y2)**2)

goal_x = 8  # int(input("Please enter goal x coordinate: "))
goal_y = 9  # int(input("Please enter goal y coordinate: "))

obstacles = [[1,1],[2,2]]

# assign reward value to goal
for i in range(len(q_values)):
    for j in range(len(q_values[0])):
        rewards[i][j] = - get_euclidian_dist(i,j,goal_x,goal_y)

# assign reward value to obstacles
for obstacle in obstacles:
    rewards[obstacle[0]][obstacle[1]] = -100

def is_final_state(row, col):
    if row == goal_x and col == goal_y:
        return True
    else:
        return False

#define a function that will choose a random, non-terminal starting location
def get_starting_location():
    #get a random row and column index
    current_row_index = np.random.randint(env_row)
    current_column_index = np.random.randint(env_col)

    #if the randomly chosen state is a final state, choose another random location
    while is_final_state(current_row_index, current_column_index) or [current_row_index, current_column_index] in obstacles:
        current_row_index = np.random.randint(env_row)
        current_column_index = np.random.randint(env_col)

    return current_row_index, current_column_index


#define an epsilon greedy algorithm that will choose which action to take next (i.e., where to move next)
def get_next_action(current_row_index, current_column_index, epsilon):
  #if a randomly chosen value between 0 and 1 is less than epsilon, 
  #then choose the most promising value from the Q-table for this state.
  if np.random.random() < epsilon:
    return np.argmax(q_values[current_row_index, current_column_index])
  else: #choose a random action
    return np.random.randint(4)


#define a function that will get the next location based on the chosen action
def get_next_location(current_row_index, current_column_index, action_index):
  new_row_index = current_row_index
  new_column_index = current_column_index
  if actions[action_index] == 'up' and current_row_index > 0:
    new_row_index -= 1
  elif actions[action_index] == 'right' and current_column_index < env_col - 1:
    new_column_index += 1
  elif actions[action_index] == 'down' and current_row_index < env_row - 1:
    new_row_index += 1
  elif actions[action_index] == 'left' and current_column_index > 0:
    new_column_index -= 1
  return new_row_index, new_column_index


# Function to navigate the turtle to the given location

x1 = 0
y1 = 0
z1 = 0
theta1 = 0

       

def pose_callback(pose_msg:Odometry):
    global x1,y1,z1,theta1
    x1 = pose_msg.pose.pose.position.x 
    y1 = pose_msg.pose.pose.position.y

    qx= pose_msg.pose.pose.orientation.x
    qy= pose_msg.pose.pose.orientation.y
    qz= pose_msg.pose.pose.orientation.z
    qw= pose_msg.pose.pose.orientation.w
    roll,pitch,yaw=euler_from_quaternion((qx,qy,qz,qw))
    theta1=yaw


def Goto_goal(x_goal, y_goal):
    global x1,y1,theta1
    msg = Twist()


    rate = rospy.Rate(10)

    distance = abs(math.sqrt(((x_goal-x1)**2)+((y_goal-y1)**2)))

    ang_dist = math.atan2((y_goal-y1),(x_goal-x1))
    while (abs(ang_dist-theta1)>0.01):
        Phi = 2
        ang_dist = math.atan2((y_goal-y1),(x_goal-x1))

        ang_speed = Phi*(ang_dist-theta1)

        msg.linear.x = 0
        msg.angular.z = ang_speed

        pub.publish(msg)

    rospy.sleep(2)

    while (distance>0.1):        
        Beta = 0.85
        distance = abs(math.sqrt(((x_goal-x1)**2)+((y_goal-y1)**2)))

        speed = distance*Beta

        msg.linear.x = speed
        msg.angular.z = 0

        pub.publish(msg)
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)
        

#define training parameters
epsilon = 0.9 #the percentage of time when we should take the best action (instead of a random action)
discount_factor = 0.9 #discount factor for future rewards
learning_rate = 0.9 #the rate at which the agent should learn


#run through 1000 training episodes
for episode in range(1000):
  #get the starting location for this episode
  row_index, column_index = get_starting_location()
  #continue taking actions (i.e., moving) until we reach a terminal state
  #(i.e., until we reach the item packaging area or crash into an item storage location)
  while not is_final_state(row_index, column_index):
    #choose which action to take (i.e., where to move next)
    action_index = get_next_action(row_index, column_index, epsilon)
    #perform the chosen action, and transition to the next state (i.e., move to the next location)
    old_row_index, old_column_index = row_index, column_index #store the old row and column indexes
    row_index, column_index = get_next_location(row_index, column_index, action_index)
    #receive the reward for moving to the new state, and calculate the temporal difference
    reward = rewards[row_index, column_index]
    old_q_value = q_values[old_row_index, old_column_index, action_index]
    temporal_difference = reward + (discount_factor * np.max(q_values[row_index, column_index])) - old_q_value
    #update the Q-value for the previous state and action pair
    new_q_value = old_q_value + (learning_rate * temporal_difference)
    q_values[old_row_index, old_column_index, action_index] = new_q_value
print('Training complete!')


#Define a function that will get the shortest path between any location within the warehouse that 
#the robot is allowed to travel and the item packaging location.
def get_shortest_path(start_row_index, start_column_index, turtle_sim=False):
  #return immediately if this is an invalid starting location
  if is_final_state(start_row_index, start_column_index):
    return []
  else: #if this is a 'legal' starting location
    current_row_index, current_column_index = start_row_index, start_column_index
    shortest_path = []
    shortest_path.append([current_row_index, current_column_index])
    #continue moving along the path until we reach the goal (i.e., the item packaging location)
    while not is_final_state(current_row_index, current_column_index):
      #get the best action to take
      action_index = get_next_action(current_row_index, current_column_index, 1.)
      #move to the next location on the path, and add the new location to the list
      current_row_index, current_column_index = get_next_location(current_row_index, current_column_index, action_index)
      shortest_path.append([current_row_index, current_column_index])
      if turtle_sim:
        print("Moving to location: ", current_row_index, current_column_index)
        Goto_goal(current_row_index, current_column_index)
    
    return shortest_path


start_pos_x = 0
start_pos_y = 0
#cm_vel we are publishing cmd_pose we subscribe using the node

rospy.init_node("Shortest_path_turtle_with_obstacles")
pub = rospy.Publisher("atom/cmd_vel",Twist, queue_size =10)
sub = rospy.Subscriber("atom/odom",Odometry, callback = pose_callback)

get_shortest_path(start_pos_x, start_pos_y, turtle_sim=True)
print("Reached goal")



# ![image.png](attachment:image.png)
# 
# ![image-2.png](attachment:image-2.png)

# ![image.png](attachment:image.png)
# 
# ![image-2.png](attachment:image-2.png)