# Coen Tempelaars, September 2017

# Non-executable script that provides a function that
# checks for consistency between the branches in code.git
# and teamplayData.git according to the following rules:

# if code.git is on master:            teamplayData.git must be on master
# if code.git is on teamplay:          teamplayData.git must be on teamplay
# if code.git is on non of the above:  teamplayData.git must be on master

# The rationale for the last check is that we assume code.git to be on a
# feature branch, which must always be based on the master branch.

# We also assume that teamplay is the only feature branch that depends on
# changes in teamplayData.git. Of course, this list may be extended.

# Returns 0 on success
# Returns 1 otherwise


check_git_branches ()
{
  local COMPATIBLE_BRANCHES="master teamplay"
  local SAFE_TEAMPLAYDATA_BRANCH="master"

  local CODE_REPO=$FALCONS_CODE_PATH
  local TPDATA_REPO=$FALCONS_TPDATA_PATH

  local CODE_BRANCH=$(/usr/bin/git -C $CODE_REPO describe --contains --all HEAD)
  local TPDATA_BRANCH=$(/usr/bin/git -C $TPDATA_REPO describe --contains --all HEAD)


  for branch in ${COMPATIBLE_BRANCHES}
  do
    if [ "$CODE_BRANCH" == "$branch" ]
    then
      #code.git is on one of the to-be-checked branches
      #echo "code.git is on $CODE_BRANCH"
      if [ "$TPDATA_BRANCH" != "$branch" ]
      then
        #teamplayData.git is on another branch, this is not consistent --> failure
        #echo "teamplayData.git is on $TPDATA_BRANCH"
        return 1
      else
        #code.git and teamplayData.git are consistent --> success
        #echo "teamplayData.git is on $TPDATA_BRANCH"
        return 0
      fi
    fi
  done

  if [ "$TPDATA_BRANCH" != "$SAFE_TEAMPLAYDATA_BRANCH" ]
  then
    #teamplayData.git is not on master, this is dangerous
    #echo "teamplayData.git is on $TPDATA_BRANCH"
    return 1
  else
    #teamplayData.git is on master, this is supposed to be safe
    #echo "teamplayData.git is on $TPDATA_BRANCH"
    return 0
  fi
}
