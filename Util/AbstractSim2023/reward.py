distanceBallGoal = ((self.target_x - self.ball_target_x)**2 + (self.target_y - self.ball_target_y)**2)**0.5
distanceToBase = ((self.robot_base_x - self.robot_x)**2 + (self.robot_base_y - self.robot_y)**2)**0.5
if self.check_facing_ball():
    reward += .03
if self.robot_x > self.target_x:
    reward -= .03
if self.pressing:
    if self.kicked and self.contacted_ball:
        reward += .6
    elif self.kicked:
        reward -= .01
    if distanceToBase < 500:
        reward += 10/(distanceToBase+0.0001)
    if distanceToBase > 1000:
        reward -= (0.001*distanceToBase)
else:
    if self.kicked and self.contacted_ball:
        reward += .2
    elif self.kicked:
        reward -= .01
    if distanceToBase < 500:
        reward += 10/(distanceToBase+0.0001)
    if distanceToBase > 1000:
        reward -= (0.001*distanceToBase)
reward -= 0.02 * abs(reward)
reward = max(-1, min(1, reward))
