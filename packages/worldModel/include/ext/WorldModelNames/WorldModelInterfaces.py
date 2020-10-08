""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
     
class WorldModelInterfaces:
    S_GET_TEAMMEMBERS = "s_get_teammembers"
    S_GET_OWN_LOCATION = "s_get_own_location"
    S_GET_OPPONENTS = "s_get_opponents"
    S_GET_OWN_OBSTACLES = "s_get_own_obstacles"
    S_SET_OWN_LOCATION = "s_set_own_location"
    S_SET_MEMBER_LOCATION = "s_set_member_location"
    S_SET_OWN_OBSTACLE_LOCATION = "s_set_own_obstacle_location"
    S_SET_REMOTE_OBSTABLE_LOCATION = "s_set_remote_obstacle_location"
    S_SET_OWN_BALL_LOCATION = "s_set_own_ball_location"
    S_SET_OWN_FRONT_CAMERA_BALL_LOCATION = "s_set_own_front_camera_ball_location"
    S_SET_REMOTE_BALL_LOCATION = "s_set_remote_ball_location"
    S_GET_BALL_POSSESSION = "s_get_ball_possession"
    S_SET_REMOTE_BALL_POSSESSION = "s_set_remote_ball_possession"
    S_GET_BALL_LOCATION = "s_get_ball_location"
    S_GET_OWN_BALL_LOCATION = "s_get_own_ball_location"
    S_CLAIM_BALL_POSSESSION = "s_claim_ball_possession"
    S_RELEASE_BALL_POSSESSION = "s_release_ball_possession"
    S_GET_ACTIVE_ROBOTS = "s_get_active_robots"
    S_SET_OWN_ENCODER_DISPLACEMENT = "s_set_own_encoder_displacement"
    S_SET_OWN_ROBOT_STATUS = "s_set_own_robot_status"
    S_GET_LAST_KNOWN_BALL_LOCATION = "s_get_last_known_ball_location"
    S_FORCE_RECALCULATION = "s_force_recalculation"