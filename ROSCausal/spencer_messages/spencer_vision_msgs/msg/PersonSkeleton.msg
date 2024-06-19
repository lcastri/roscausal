# Message describing the skeleton of a single detected or tracked person
#

uint64              person_id    # detection or track ID
float32             detection_score  # detection confidence of the entire skeleton/person. can be average over all joints, or copy of top-down detector score
BodyJoint[]         joints       # the detected or tracked body joints belonging to this skeleton

