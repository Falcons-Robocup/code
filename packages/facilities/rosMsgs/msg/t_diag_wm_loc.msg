# inputs statistics (per heartbeat)
uint8      numVisionCandidates          # number of vision candidates (excluding mirrors), typically 0..2 depending on frequencies
uint8      numMotorDisplacementSamples  # number of encoder samples, typically 2..5 depending on frequencies

# internal scores
float32    confidence                   # how well does vision candidate match with worldModels running total (0.0=bad, 1.0=good)
posconf    bestVisionCandidate          # best fit of all vision candidates - if confidence < 0.70, it means vision and worldModel disagree, and we should somehow visualize one of them in red
# TODO: make 0.70 a configuration parameter which visualizer can use/fetch
# TODO: consider sending a list (typically empty) of badCandidates to be drawn red, instead of only bestVisionCandidate - this would be more efficient bandwidth wise, and more flexible diagnostics wise, but it depends on what Andre wants to see
float32    visionNoiseXY                # [m] 3sigma vision KPI: noise on vision position, lower is better, probably about 5cm when stationary
float32    visionNoisePhi               # [rad] 3sigma vision KPI: noise on vision orientation, lower is better

# output
t_posvel   ownpos                       # resulting position and speed
bool           isLocationValid
