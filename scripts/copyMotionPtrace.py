import argparse
import subprocess

def getLatestLog(cmd):
    try:
	log = subprocess.check_output(cmd, shell=True).strip().split()[-1]
    except subprocess.CalledProcessError as e:
	print "Caught Exception:", e
        log = ""

    return log

parser = argparse.ArgumentParser(description='This script copies the latest ptrace for PathPlanning, VelocityControl and PeripheralsInterface to the given computer dir at ~/kstdata/. This data is to be used with ptrace.kst located in the components scripts dir (e.g., velocityControl/scripts/ptrace.kst). Example: python copyMotionPtrace.py -c pingu')
parser.add_argument('-c', '--computer', help='the computer to which to copy the ptrace data. Should have existing directory ~/kstdata.')
args = parser.parse_args()

# Find latest falcons logging directory (copied from 'alias logcd')
cmd = "ls -1dt /var/tmp/falco* | head -1"
logdir = subprocess.check_output(cmd, shell=True).strip()

################
# PathPlanning #
################
# Find latest ptrace file from PathPlanning
cmd = "ls -tr %s/ptrace_*_pp*.txt" % logdir
pathPlanningLog = getLatestLog(cmd)

# Grep "KST" from the ptrace to filter on motion data for KST plot.
if pathPlanningLog != "":
    pathPlanningKstLog = "ptrace_pp_kst.txt"
    cmd = "grep 'KST' %s > %s/%s" % (pathPlanningLog, logdir, pathPlanningKstLog)
    subprocess.call(cmd, shell=True)

###################
# VelocityControl #
###################
# Find latest ptrace file from VelocityControl
cmd = "ls -tr %s/ptrace_*_vc*.txt" % logdir
velocityControlLog = getLatestLog(cmd)

# Grep "KST" from the ptrace to filter on motion data for KST plot.
if velocityControlLog != "":
    velocityControlKstLog = "ptrace_vc_kst.txt"
    cmd = "grep 'KST' %s > %s/%s" % (velocityControlLog, logdir, velocityControlKstLog)
    subprocess.call(cmd, shell=True)

########################
# PeripheralsInterface #
########################
# Find latest ptrace file from PeripheralsInterface
cmd = "ls -tr %s/ptrace_*_halMotors*.txt" % logdir
peripheralsInterfaceLog = getLatestLog(cmd)

# Grep "KST" from the ptrace to filter on motion data for KST plot.
if peripheralsInterfaceLog != "":
    peripheralsInterfaceKstLog = "ptrace_pi_kst.txt"
    cmd = "grep 'KSTB' %s > %s/%s" % (peripheralsInterfaceLog, logdir, peripheralsInterfaceKstLog)
    subprocess.call(cmd, shell=True)

# Copy log files to given hostname
if pathPlanningLog != "":
    cmd = "scp %s/%s %s:~/kstdata/ptrace_pp.txt" % (logdir, pathPlanningKstLog, args.computer)
    subprocess.call(cmd, shell=True)

if velocityControlLog != "":
    cmd = "scp %s/%s %s:~/kstdata/ptrace_vc.txt" % (logdir, velocityControlKstLog, args.computer)
    subprocess.call(cmd, shell=True)

if peripheralsInterfaceLog != "":
    cmd = "scp %s/%s %s:~/kstdata/ptrace_pi.txt" % (logdir, peripheralsInterfaceKstLog, args.computer)
    subprocess.call(cmd, shell=True)

print "Done."
