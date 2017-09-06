# an array of events is transmitted from each robot to coach via a regular diagnostics channel
# events are repeatedly sent for a while, to avoid them from getting lost
# also, a counter is sent, so packetloss can be measured  


int32       id        # packet counter for calculating packetloss
float64     timeStamp # for measuring latency & sync
t_event[]   events    # array of events which occurred in the recent past

