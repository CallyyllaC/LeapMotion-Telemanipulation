#
# Depricated, no longer required due to using ros network to connect multiple PC's for dual leaps
#

#message that holds the key values required by the movegroup

#left hand
float64[3] left_location
float64[3] left_orientation

#right hand
float64[3] right_location
float64[3] right_orientation

#is the left hand grabbing
float32 grab

#is the right hand pinching
float32 pinch
