import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps
import sklearn
import joblib
import numpy as np
import time
import imgclassification_sol
img_classification = imgclassification_sol.ImageClassifier()
model = joblib.load('trained_model.pkl')

def run(robot: cozmo.robot.Robot):
	robot.camera.image_stream_enabled = True
	robot.camera.color_image_enabled = False
	robot.camera.enable_auto_exposure()

def idle(robot: cozmo.robot.Robot):
    robot.say_text("Idle State").wait_for_completed()
    time.sleep(2.0)
    robot.set_head_angle(degrees(10.0)).wait_for_completed()
    images = []
    while len(images) < 3:
        if len(images) == 2:
            robot.turn_in_place(angle = cozmo.util.Angle(degrees = -5), speed = cozmo.util.Angle(degrees = 5)).wait_for_completed()
        else:
            robot.turn_in_place(angle= cozmo.util.Angle(degrees= 5), speed= cozmo.util.Angle(degrees= 5)).wait_for_completed()
        ourImage = np.asarray(robot.world.latest_image.raw_image)
        images.append(ourImage[:,:,0])


    images_features = img_classification.extract_image_features(images)
    labels = model.predict(images_features)
    print(labels)

    temp = dict()
    for label in labels:
        if label in temp:
            temp[label] + 1
        else:
            temp[label] = 1

    maxNum = 0
    max_prediction = None
    for label in temp:
        tempLabel = temp[label]
        if tempLabel > maxNum:
            maxNum = tempLabel
            max_prediction = label

    if max_prediction == 'order':
        robot.say_text("order").wait_for_completed()
        order(robot)
    elif max_prediction == 'drone':
        robot.say_text("drone").wait_for_completed()
        drone(robot)
    elif max_prediction == 'inspection':
        robot.say_text("inspection").wait_for_completed()
        inspection(robot)
    else:
        idle(robot)

def order(robot: cozmo.robot.Robot):
    # Robot at D, cube at C, pick up cube, drive forward to end of arena (A), put down cube, drive  backward to D
    robot.set_head_angle(degrees(-5.0)).wait_for_completed()
    robot.set_lift_height(height=0).wait_for_completed()
    cube = robot.world.wait_for_observed_light_cube(timeout=30)
    robot.pickup_object(cube, num_retries=5).wait_for_completed()
    robot.drive_straight(distance_mm(300), speed_mmps(50)).wait_for_completed()
    robot.place_object_on_ground_here(cube).wait_for_completed()
    robot.drive_straight(distance_mm(-355), speed_mmps(50)).wait_for_completed()
    idle(robot)

def drone(robot: cozmo.robot.Robot):
    # Make robot drive in an S formation. Show animation of choice on robot face
    robot.drive_wheels(50.0, 120.0, duration = 3.5)
    robot.drive_wheels(120.0, 50.0, duration = 4.0)
    robot.set_head_angle(degrees(30.0)).wait_for_completed()
    robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabWin).wait_for_completed()
    idle(robot)

def inspection(robot: cozmo.robot.Robot):
    for i in range(0, 4):
        if i % 2 == 0:
            robot.move_lift(0.25)
        else: 
            robot.move_lift(-0.25)
        robot.say_text("I am not a spy", in_parallel = True)
        robot.drive_straight(distance_mm(200), speed_mmps(50), in_parallel = True).wait_for_completed()
        robot.turn_in_place(degrees(90), in_parallel = True).wait_for_completed()
    robot.move_lift(-1.0)
    idle(robot)

def runner(robot: cozmo.robot.Robot):
    run(robot)
    idle(robot)

cozmo.run_program(runner, use_viewer=True, force_viewer_on_top=True)