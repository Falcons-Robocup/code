import subprocess
import os
import falconspy

# This script changes the Git repositories to be cloned from SSH

if __name__ == "__main__":

    # cd ~/falcons/code
    # git remote set-url origin ssh://git@git.falcons-robocup.nl:2222/falcons/code.git
    print("Changing falcons/code to SSH...")
    os.chdir(falconspy.FALCONS_CODE_PATH)
    cmd = "git remote set-url origin ssh://git@git.falcons-robocup.nl:2222/falcons/code.git"
    subprocess.call(cmd, shell=True)

    # cd ~/falcons/data
    # git remote set-url origin ssh://git@git.falcons-robocup.nl:2222/falcons/data.git
    print("Changing falcons/data to SSH...")
    os.chdir(falconspy.FALCONS_DATA_PATH)
    cmd = "git remote set-url origin ssh://git@git.falcons-robocup.nl:2222/falcons/data.git"
    subprocess.call(cmd, shell=True)

    # cd ~/falcons/teamplayData
    # git remote set-url origin ssh://git@git.falcons-robocup.nl:2222/falcons/teamplayData.git
    print("Changing falcons/teamplayData to SSH...")
    os.chdir(falconspy.FALCONS_TPDATA_PATH)
    cmd = "git remote set-url origin ssh://git@git.falcons-robocup.nl:2222/falcons/teamplayData.git"
    subprocess.call(cmd, shell=True)
