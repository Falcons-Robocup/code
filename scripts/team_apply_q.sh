 #!/bin/bash
#
# Run a command on each robot. Allow aliases.
# 
# Examples:
#    team_apply.sh fk           # to run falcons_kill.sh on each robot
#    team_apply.sh subfab       # to SVN update & build all
#
# Optional: specify robot numbers (r1, r2, or without r)
#    team_apply.sh 1 3 subfab   # only robots 1 and 3
#    team_apply.sh r1 r3 subfab # only robots 1 and 3
# 
# 



# construct list of robots to apply to
robots=""
while :; do
    # check if it is a number
    found=0
    for irobot in `seq 1 8`; do
        if [ "$1" = "$irobot" -o "$1" = "r$irobot" ]; then
            robots="$robots r$irobot"
            found=1
        fi
    done
    if [ "$found" = "1" ]; then
        shift
    else
        break
    fi
done
if [ -z "$robots" ]; then
    # none specified = all
    robots="r1 r2 r3 r4 r5 r6 r7 r8"
fi
cmd=$*

# cleanup log
rm /var/tmp/team_apply_q_*.out 2>/dev/null

# run the command
for robot in $robots; do
    ping -W 1 -c 1 $robot >/dev/null 2>/dev/null
    if [ "$?" == "0" ]
    then
        echo "starting command silently on $robot"
        (
            ssh $robot $FALCONS_CODE_PATH/packages/coachCommands/rwrap $cmd
        ) > /var/tmp/team_apply_q_$robot.out 2>&1 &
    else
        echo "ERROR: could not reach $robot"
    fi
done

# wait
echo -n "waiting for processes to finish..."
wait
echo " done"

# a bit of feedback
egrep -i 'error|fail' /var/tmp/team_apply_q_*
for f in /var/tmp/team_apply_q_* ; do
    tail $f
done



