# communication KPI's
float32 frequency   # frequency of packets arriving from robot
float32 packetLoss  # portion of packets which gets lost in wifi 
float32 latency     # average delay in seconds from robot to coach, also affected by clock sync
# note: bandwidth is measured on-robot by reading network Tx and Rx, see robotControl health module

