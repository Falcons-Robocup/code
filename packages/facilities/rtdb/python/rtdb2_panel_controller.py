""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 class RtDBPanelController():
    def __init__(self, max_vpos = None, max_hpos = None):
        self.vpos = 0
        self.hpos = 0
        self.max_vpos = max_vpos
        self.max_hpos = max_hpos

    def reset(self):
        self.vpos = 0
        self.hpos = 0

    def increment_vertical(self, page = 1):
        if self.max_vpos is not None:
            self.vpos = self.vpos + page if self.vpos + page < self.max_vpos else self.max_vpos - 1
    def decrement_vertical(self, page = 1):
        self.vpos = 0 if self.vpos - page < 0 else self.vpos - page
    def increment_horizontal(self, page = 1):
        if self.max_hpos is not None:
            self.hpos = self.hpos + page if self.hpos + page < self.max_hpos else self.max_hpos - 1
    def decrement_horizontal(self, page = 1):
        self.hpos = 0 if self.hpos - page < 0 else self.hpos - page
