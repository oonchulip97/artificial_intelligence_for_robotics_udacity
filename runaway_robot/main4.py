# ----------
# Part Four
#
# Again, you'll track down and recover the runaway Traxbot.
# But this time, your speed will be about the same as the runaway bot.
# This may require more careful planning than you used last time.
#
# ----------
# YOUR JOB
#
# Complete the next_move function, similar to how you did last time.
#
# ----------
# GRADING
#
# Same as part 3. Again, try to catch the target in as few steps as possible.

from robot import *
from math import *
from matrix import *
from ekf import ExtendedKalmanFilter
from copy import deepcopy
import random

def update_and_predict(measurement, OTHER = None):
    """Return the predicted x and P of target."""

    dt = 1
    # initialization stage: 0 measurements
    # warming-up stage: 0-2 measurements
    if OTHER == None or OTHER[0] == 0:

        if OTHER == None:
            OTHER = []
            OTHER.append(0)

        data = ExtendedKalmanFilter()
        data.set_x(measurement[0], measurement[1], 0., 0., 0.)
        data.set_P(1000.)
        data.set_dt(dt)
        OTHER.append(data)

        if len(OTHER) == 3:
            OTHER[0] = 1

    # transition stage: 3 measurements
    elif OTHER[0] == 1:

        x_prev1,y_prev1  = OTHER[1].get_coordinates()
        x_prev2,y_prev2  = OTHER[2].get_coordinates()
        x_prev3,y_prev3  = measurement[0], measurement[1]

        distance = distance_between((x_prev2,y_prev2),(x_prev3,y_prev3))
        heading = angle_trunc(atan2(y_prev3-y_prev2,x_prev3-x_prev2))
        turning = angle_trunc(atan2(y_prev3-y_prev2,x_prev3-x_prev2) - \
                  atan2(y_prev2-y_prev1,x_prev2-x_prev1))

        data = ExtendedKalmanFilter()
        data.set_x(x_prev3, y_prev3, distance, heading, turning)
        data.set_P(1000.)
        data.set_dt(dt)
        data.update(measurement)
        data.predict()
        OTHER = [data]

    # predicting stage: > 3 measurements
    else:

        OTHER[0].update(measurement)
        OTHER[0].predict()

    return OTHER

def extrapolation(measurement, OTHER, num_steps):
    """Return the predicted x, y coordinates of the
    target after a specified amount of steps."""

    dt = 1

    OTHER = update_and_predict(measurement, OTHER)
    # not in predicting stage
    if OTHER[0] == 0 or OTHER[0] == 1:
        xy_estimate = measurement

    # predicting stage
    else:

        x,P = OTHER[0].get_data()
        x_predict = x.value[0][0]
        y_predict = x.value[1][0]
        target_velocity = x.value[2][0]
        target_heading = x.value[3][0]
        target_turning = x.value[4][0]

        step = 1
        while step < num_steps:
            x_predict = x_predict + target_velocity * dt * cos(target_heading + target_turning * dt)
            y_predict = y_predict + target_velocity * dt * sin(target_heading + target_turning * dt)
            target_heading = angle_trunc(target_heading + target_turning * dt)
            step += 1

        xy_estimate = (x_predict, y_predict)
    return xy_estimate, OTHER

def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    # This function will be called after each time the target moves.

    # The OTHER variable is a place for you to store any historical information about
    # the progress of the hunt (or maybe some localization information). Your return format
    # must be as follows in order to be graded properly.

    target_node = []
    hunter_node = []
    num_expand = 15
    cost = 1
    tolerance = 0.001

    for expand_index in range(1, num_expand + 1):

        OTHER_aux = deepcopy(OTHER)
        target_loc, _ = extrapolation(target_measurement, OTHER_aux, expand_index)
        target_node.append([expand_index, target_loc])

        hunter_distance = min(max_distance, distance_between(hunter_position, target_loc))
        hunter_turning = angle_trunc(get_heading(hunter_position, target_loc) - hunter_heading)
        x_hunter = hunter_position[0] + hunter_distance * cos(hunter_heading + hunter_turning)
        y_hunter = hunter_position[1] + hunter_distance * sin(hunter_heading + hunter_turning)
        step_cost = cost
        distance_cost = distance_between((x_hunter, y_hunter), target_loc)
        total_cost = step_cost + distance_cost
        hunter_node.append([total_cost, step_cost, distance_cost, (x_hunter, y_hunter), angle_trunc(hunter_heading + hunter_turning)])

    found = False
    resign = False
    while not found and not resign:
        if len(hunter_node) == 0:
            print 'Target cannot be hunted.'
            expand_index = 1
            resign = True

        else:
            index_min = hunter_node.index(min(hunter_node))

            target_node_min = target_node.pop(index_min)
            expand_index = target_node_min[0]
            target_loc_min = target_node_min[1]

            hunter_node_min = hunter_node.pop(index_min)
            step_cost = hunter_node_min[1]
            hunter_loc_min = hunter_node_min[3]
            hunter_heading_min = hunter_node_min[4]

            if distance_between(hunter_loc_min, target_loc_min) < tolerance:
                found = True

            else:
                if step_cost / cost < expand_index:
                    target_node.append([expand_index, target_loc_min])

                    hunter_distance = min(max_distance, distance_between(hunter_loc_min, target_loc_min))
                    hunter_turning = angle_trunc(get_heading(hunter_loc_min, target_loc_min) - hunter_heading_min)
                    x_hunter = hunter_loc_min[0] + hunter_distance * cos(hunter_heading_min + hunter_turning)
                    y_hunter = hunter_loc_min[1] + hunter_distance * sin(hunter_heading_min + hunter_turning)
                    step_cost += cost
                    distance_cost = distance_between((x_hunter, y_hunter), target_loc_min)
                    total_cost = step_cost + distance_cost
                    hunter_node.append([total_cost, step_cost, distance_cost, (x_hunter, y_hunter), angle_trunc(hunter_heading_min + hunter_turning)])

    target_loc, OTHER = extrapolation(target_measurement, OTHER, expand_index)
    distance = distance_between(hunter_position,target_loc)
    turning = angle_trunc(get_heading(hunter_position,target_loc) - hunter_heading)

    return turning, distance, OTHER

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance # 0.97 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0

    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:

        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()

        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught

def demo_grading_visual(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we
    will grade your submission."""
    max_distance = 0.97 * target_bot.distance # 0.98 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0 #change size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        #Visualize it
        measuredbroken_robot.setheading(target_bot.heading*180/pi)
        measuredbroken_robot.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        chaser_robot.setheading(hunter_bot.heading*180/pi)
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
        #End of visualization
        ctr += 1
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught

def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all
    the target measurements, hunter positions, and hunter headings over time, but it doesn't
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables

    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER

target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
measurement_noise = 0.05*target.distance # VERY NOISY!!
target.set_noise(0.0, 0.0, measurement_noise)

hunter = robot(-10.0, -10.0, 0.0)

#demo_grading(hunter, target, next_move)
demo_grading_visual(hunter, target, next_move)
