#!/usr/bin/env python
#
# FALCONS // Jan Feitsma, December 2017


from tracePlotUtils import *



nan = float("nan")


class tracePlotRobot:

    def __init__(self, robotId):
        self.robotId = robotId
        self.events = eventStore()
        self.t0 = 1e99
        self.mp = valueStore("elapsed phase deltaPhi threshold didShoot hasBall disabledBh settledOk")
        self.pp = valueStore("targetX targetY targetPhi velocityX velocityY velocityPhi")
        self.wm = valueStore("valid currentX currentY currentPhi velocityX velocityY velocityPhi")
        self.vis = valueStore("x y phi conf coord")
        self.bh = valueStore("angleL angleR liftedL liftedR hasBall velL velR enabledL enabledR")
        # ball-related
        self.bm = valueStore("robot cam tstamp x y z camX camY camZ camPhi az el r conf tId")
        ballDetails = "nT tId conf x y z vx vy vz age nM black oBF nC wO cS aS mS frS zS vS bS fitS fitQ"
        self.br = valueStore(ballDetails) # best accepted ball only, or noBall
        self.bt = valueStore(ballDetails) # all trackers, for analyzing confidence heuristics
        self.bd = valueStore("details") # best tracker details, as string only to be parsed on-demand

    def limit(self, timeRange):
        self.events.limit(timeRange)
        self.mp.limit(timeRange)
        self.pp.limit(timeRange)
        self.wm.limit(timeRange)
        self.bh.limit(timeRange)
        self.bm.limit(timeRange)
        self.br.limit(timeRange)
        self.bt.limit(timeRange)
        self.bd.limit(timeRange)
        
    def load(self, f, settings):
        #print "parsing file for robot %d: %s" % (self.robotId, f)
        def parseBH(t, words, store):
            assert(len(words) == 9)
            angleL     = float(words[0])
            angleR     = float(words[1])
            liftedL    = int(words[2])
            liftedR    = int(words[3])
            hasBall    = int(words[4])
            velL       = float(words[5])
            velR       = float(words[6])
            enabledL   = int(words[7])
            enabledR   = int(words[8])
            store.add(t, [angleL, angleR, liftedL, liftedR, hasBall, velL, velR, enabledL, enabledR])
        def parsePP(t, words, store):
            assert(len(words) == 6)
            targetX    = float(words[0])
            targetY    = float(words[1])
            targetPhi  = float(words[2])
            velocityX  = float(words[3])
            velocityY  = float(words[4])
            velocityPhi= float(words[5])
            store.add(t, [targetX, targetY, targetPhi, velocityX, velocityY, velocityPhi])
        def parseWM(t, words, store): # localization
            assert(len(words) == 7)
            isValid    = int(words[0])
            currentX   = float(words[1])
            currentY   = float(words[2])
            currentPhi = float(words[3])
            velocityX  = float(words[4])
            velocityY  = float(words[5])
            velocityPhi= float(words[6])
            if isValid:
                store.add(t, [True, currentX, currentY, currentPhi, velocityX, velocityY, velocityPhi])
            else:
                store.add(t, [False, nan, nan, nan, nan, nan, nan])
        def parseVision(t, words, store): # localization
            assert(len(words) == 6)
            t          = float(words[0]) # overwrite with latency-corrected timestamp (as exported by vision / multiCam)
            x          = float(words[1])
            y          = float(words[2])
            phi        = float(words[3])
            conf       = float(words[4])
            coord      = int(words[5]) # unused
            store.add(t, [x, y, phi, conf, coord])
        def parseBM(t, words, store):
            assert(len(words) == 15)
            robot      = int(words[0])
            cam        = words[1]
            tstamp     = float(words[2])
            x          = float(words[3])
            y          = float(words[4])
            z          = float(words[5])
            camX       = float(words[6])
            camY       = float(words[7])
            camZ       = float(words[8])
            camPhi     = float(words[9])
            az         = float(words[10])
            el         = float(words[11])
            r          = float(words[12])
            conf       = float(words[13])
            tId        = int(words[14])
            t          = tstamp # latency corrected
            store.add(t, [robot, cam, tstamp, x, y, z, camX, camY, camZ, camPhi, az, el, r, conf, tId])
        def parseBD(t, words, store):
            details = ' '.join(words)
            store.add(t, [details])
        def parseBT(t, words, store):
            assert(len(words) == 25)
            nT         = int(words[0])
            tId        = int(words[2])
            conf       = float(words[3])
            x          = float(words[4])
            y          = float(words[5])
            z          = float(words[6])
            vx         = float(words[7])
            vy         = float(words[8])
            vz         = float(words[9])
            age        = float(words[10])
            nM         = int(words[11])
            black      = int(words[12])
            oBF        = int(words[13])
            nC         = int(words[14])
            wO         = int(words[15])
            cS         = float(words[16])
            aS         = float(words[17])
            mS         = float(words[18])
            frS        = float(words[19])
            zS         = float(words[20])
            vS         = float(words[21])
            bS         = float(words[22])
            fitS       = float(words[23])
            fitQ       = float(words[24])
            store.add(t, [nT, tId, conf, x, y, z, vx, vy, vz, age, nM, black, oBF, nC, wO, cS, aS, mS, frS, zS, vS, bS, fitS, fitQ])
        def parseMP(t, words, store):
            action = words[0]
            if action in ["SHOOT", "PASS"]:
                elapsed    = float(words[1])
                phase      = words[2]
                deltaPhi   = float(words[3])
                threshold  = float(words[4])
                didShoot   = int(words[5])
                hasBall    = int(words[6])
                disabledBh = int(words[7])
                settledOk  = int(words[8])
                store.add(t, [elapsed, phase, deltaPhi, threshold, didShoot, hasBall, disabledBh, settledOk])
            elif action in ["MOVE"]:
                store.add(t, [0, "", nan, nan, nan, nan, nan, nan])
            elif action in ["SHOOTEVENT"]:
                targetX    = float(words[1])
                targetY    = float(words[2])
                targetZ    = float(words[3])
                distance   = float(words[4])
                doLob      = int(words[5])
                details = "doLob=%d, distance=%4.1f, target=(%6.2f,%6.2f,%6.2f)" % (doLob, distance, targetX, targetY, targetZ)
                self.events.add(t, "shoot", details)
            elif action in ["BALLHANDLEREVENT"]:
                if words[1] == "disable":
                    self.events.add(t, "bhOff")
                else:
                    self.events.add(t, "bhOn")
        def parseSP(t, words):
            action = words[0]
            if action == "POWER":
                details = "%.1f" % (float(words[1]))
                self.events.add(t, "kick", details)
            elif action == "HEIGHT":
                details = "%.1f" % (float(words[1]))
                self.events.add(t, "setHeight", details)
        def guessComponent(f):
            components = ["MP", "SP", "BH", "WM", "PP"]
            for cc in components:
                if '_' + cc.lower() in f:
                    return cc
                if "_halMotors" in f:
                    return "BH"
        # parse lines
        lineCounter = 0
        lines = file(f).readlines()
        for line in lines:
            lineCounter += 1
            words = line.split()
            if len(words) == 0:
                continue
            try:
                t = float(words[0])
                self.t0 = min(self.t0, t)
                cc = guessComponent(f)
                words = words[1:]
                if cc == "MP":
                    parseMP(t, words, self.mp)
                elif cc == "SP":
                    parseSP(t, words)
                elif cc == "BH":
                    if words[0] != "KST" and words[0] != "KSTM":
                        pass # TODO update, Edwin and Erik added new stuff but deleted all nice stuff which used to be plotted here
                        #parseBH(t, words, self.bh)
                elif cc == "WM":
                    mode       = words[0]
                    if mode not in ["LOC", "VISION", "BM", "BR", "BT", "BD"]:
                        mode   = "LOC"
                        words  = ["LOC"] + words
                    words = words[1:]
                    if mode == "LOC":
                        parseWM(t, words, self.wm)
                    if mode == "VISION":
                        parseVision(t, words, self.vis)
                    if mode == "BM":
                        parseBM(t, words, self.bm)
                    if mode == "BR":
                        if len(words) == 2 and words[1] == "noBall":
                            tstamp = float(words[0])
                            self.br.add(tstamp, [0] + [nan] * 23)
                        else:
                            tstamp = float(words[0])
                            parseBT(tstamp, words[1:], self.br)
                    if mode == "BT":
                        parseBT(t, words[1:], self.bt)
                    if mode == "BD":
                        parseBD(t, words, self.bd)
                elif cc == "PP":
                    if (words[0] != "KST") and (words[0] != "INIT"):
                        parsePP(t, words, self.pp)
                else:
                    raise Exception("unrecognized component " + cc)
            except:
                # do not complain if it is the last line in file
                if lineCounter == len(lines):
                    # silent return
                    return
                print ""
                print "failed to parse line %d in file %s:" % (lineCounter, f)
                print line
                raise
                
