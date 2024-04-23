import pybullet as p
import pybullet_data
import time

TEXT_LABEL_POS = [-0.1,-0.2,0]
TEXT_SIZE = 0.08
COLOR_RED = [255,0,0]
COLOR_GREEN = [0,255,0]
COLOR_BLUE = [0,0,255]

def zoom_in_to_object(object_id, distance=1.0, pitch=-45, yaw=0):
    # Get the position of the target object
    target_pos, _ = p.getBasePositionAndOrientation(object_id)

    # Calculate new camera position and orientation
    target_pos_above = [target_pos[0]+0.4, target_pos[1]-0.5, target_pos[2] + 0.3]  # Look slightly above the object

    # Set the camera position and orientation
    p.resetDebugVisualizerCamera(cameraDistance=distance, 
                                 cameraYaw=yaw, 
                                 cameraPitch=pitch,
                                 cameraTargetPosition=target_pos_above)

def update_floor_label(debugLabelId, text, color=COLOR_BLUE):
    p.addUserDebugText(text=text, 
                       textPosition=TEXT_LABEL_POS, 
                       textColorRGB=color, 
                       textOrientation = [0,0,0,1], 
                       textSize=TEXT_SIZE, 
                       parentObjectUniqueId=planeId,
                       replaceItemUniqueId=debugLabelId)

def wait(cycleNum=200):
    for _ in range(cycleNum):
            p.stepSimulation()

    
# Connection
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)
planeId = p.loadURDF("plane.urdf")

# Load gripper model
gripperId = p.loadURDF("./models/base_gripper_model.urdf")
baseJointIndex = 0
leftGripperBaseJointIndex = 1
rightGripperBaseJointIndex = 2

# Zoom in to the loaded object
zoom_in_to_object(gripperId, distance=0.05, pitch=-35, yaw=45)

# Add a label to the floor
debugLabelId = p.addUserDebugText(text="Base model", 
                                  textPosition=TEXT_LABEL_POS, 
                                  textColorRGB=COLOR_BLUE, 
                                  textOrientation = [0,0,0,1], 
                                  textSize=TEXT_SIZE, 
                                  parentObjectUniqueId=planeId)

# Create object to grasp
# Create a collision and visual shape for the box
halfExtents = [0.01, 0.01, 0.01]
boxCollisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=halfExtents)
boxVisualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=halfExtents)

# Define the starting position and orientation of the box
boxStartPos = [0, 0, 0.0]
boxStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

# Create a multi-body object with the box collision and visual shapes
boxId = p.createMultiBody(baseMass=1, baseInertialFramePosition=[0, 0, 0],
                          baseCollisionShapeIndex=boxCollisionShapeId,
                          baseVisualShapeIndex=boxVisualShapeId,
                          basePosition=boxStartPos,
                          baseOrientation=boxStartOrientation)



# Simulation: Lower the gripper to the floor
update_floor_label(debugLabelId, "Lower gripper..")
while True:
    # Step the simulation
    p.stepSimulation()

    # DEBUG: Get the current joint position
    baseJointPos, _, _, _ = p.getJointState(gripperId, baseJointIndex)
    print("base:",baseJointPos)
    
    # Check: gripper touches the floor
    contact_points = p.getContactPoints(bodyA=gripperId, 
                                        linkIndexA=leftGripperBaseJointIndex, 
                                        bodyB=planeId, 
                                        linkIndexB=-1)
    
    # Action: move to floor
    targetVelocity = -0.02
    maxForce = 500
    if contact_points: # Stop
        p.setJointMotorControl2(gripperId, 
                        baseJointIndex, 
                        p.VELOCITY_CONTROL, 
                        targetVelocity=0,
                        force=maxForce)
        break
    else: # Lowering
        p.setJointMotorControl2(gripperId, 
                        baseJointIndex, 
                        p.VELOCITY_CONTROL, 
                        targetVelocity=targetVelocity,
                        force=maxForce)

    # Wait for a short time
    time.sleep(1./240.)

# Simulation: Close gripper
update_floor_label(debugLabelId, "Close gripper..")
while True:
    # Step the simulation
    p.stepSimulation()

    # DEBUG: Get the current gripper joint position
    leftGripperJointPos, _, _, _ = p.getJointState(gripperId, leftGripperBaseJointIndex)
    rightGripperJointPos, _, _, _ = p.getJointState(gripperId, rightGripperBaseJointIndex)
    print("left:",leftGripperJointPos,"right:",rightGripperJointPos)

    # Check: gripper touches the box to grasp
    left_gripper_contact_points = p.getContactPoints(bodyA=gripperId,linkIndexA=leftGripperBaseJointIndex, bodyB=boxId, linkIndexB=-1)
    right_gripper_contact_points = p.getContactPoints(bodyA=gripperId,linkIndexA=rightGripperBaseJointIndex, bodyB=boxId, linkIndexB=-1)

    # Action: move to floor
    taregetPosition = 0.04
    maxForce = 50

    if left_gripper_contact_points and right_gripper_contact_points:
        wait(200) # Wait steady, and stop
        break
    else: # Close grippers
        p.setJointMotorControl2(gripperId, 
                            leftGripperBaseJointIndex, 
                            p.POSITION_CONTROL, 
                            targetPosition=taregetPosition,
                            force=maxForce)
        p.setJointMotorControl2(gripperId, 
                            rightGripperBaseJointIndex, 
                            p.POSITION_CONTROL, 
                            targetPosition=-taregetPosition,
                            force=maxForce)

    # Wait for a short time
    time.sleep(1./240.)

# Simulate: Lift to set height
update_floor_label(debugLabelId, "Lift gripper..")
while True:
    # Step the simulation
    p.stepSimulation()

    # DEBUG: Get the current joint position
    baseJointPos, _, _, _ = p.getJointState(gripperId, baseJointIndex)
    print("base:",baseJointPos)
    
    # Action: move to height
    targetPos = -0.00001
    targetVelocity = 0.02
    maxForce = 500
    if baseJointPos >= targetPos: # Stop
        p.setJointMotorControl2(gripperId, 
                        baseJointIndex, 
                        p.VELOCITY_CONTROL, 
                        targetVelocity=0,
                        force=maxForce)
        break
    else: # Lowering
        p.setJointMotorControl2(gripperId, 
                        baseJointIndex, 
                        p.VELOCITY_CONTROL, 
                        targetVelocity=targetVelocity,
                        force=maxForce)

    # Wait for a short time
    time.sleep(1./240.)

# Validation: Graspable
wait(400) # Wait steady

# Check: gripper touches the box to grasp
left_gripper_contact_points = p.getContactPoints(bodyA=gripperId,linkIndexA=leftGripperBaseJointIndex, bodyB=boxId, linkIndexB=-1)
right_gripper_contact_points = p.getContactPoints(bodyA=gripperId,linkIndexA=rightGripperBaseJointIndex, bodyB=boxId, linkIndexB=-1)

if left_gripper_contact_points and right_gripper_contact_points:
    update_floor_label(debugLabelId, "Grasp success!", COLOR_GREEN)
else:
    update_floor_label(debugLabelId, "Grasp failed!", COLOR_RED)


# The end
while True:
    # Step the simulation
    p.stepSimulation()
    # Wait for a short time
    time.sleep(1./240.)
