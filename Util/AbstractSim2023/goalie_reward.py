distanceToGoal = ((self.target_x - (-self.goal_x))**2 + (self.target_y - self.goal_y)**2)**0.5
if self.check_facing_ball():
    reward += 0.2
if self.check_if_blocking_goal():
    reward += 0.7
if self.kicked:
    reward -=.001
    if self.contacted_ball:
        reward += 0.5
    if self.defender_kicking:
        reward -= 0.1
if self.target_x < self.robot_x: # ball has gone past the goalie
    reward -= 0.4
if self.robot_x < -4300:
    reward -= 0.02
if self.robot_x < -4500:
    reward -= 100
if distanceToGoal > 500:
    reward -= 0.3
reward -= (0.2/(self.goalpost_1_distance+.000000001))
reward -= (0.2/(self.goalpost_2_distance+.000000001))
# Regularization - discourage large changes in reward
reward -= 0.02 * abs(reward)
# Ensure final reward stays within range
reward = max(-1, min(1, reward))
