""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import time, os
import Gnuplot, Gnuplot.funcutils
import subprocess

if __name__ == "__main__":

   tracedir_cmd = "ls -ltr /var/tmp/ | grep falcons_control_201 | awk '{print $9}' | tail -n 1"
   tracedir = subprocess.check_output(tracedir_cmd, shell=True).strip()
   tracedir = "/var/tmp/" + tracedir
   
   pidplot_entries_cmd = "grep PIDPLOT %s/*" % tracedir
   pidplot_entries = subprocess.check_output(pidplot_entries_cmd, shell=True).strip().split('\n')

   with open('pidplot.dat', 'w') as f:
      i = 1
      for pidplot_entry in pidplot_entries:
         # Replace all double spaces by a single space
         while '  ' in pidplot_entry:
            pidplot_entry = pidplot_entry.replace('  ', ' ')
         pidplot_entry_vals = pidplot_entry.split(' ')

         # Group together only the values we need for the plot
         pidplot_entry = str(i) + " " + ' '.join(pidplot_entry_vals[5:]) + "\n"
         f.write(pidplot_entry)

         i += 1


   # Determine which columns to plot.
   columns = {
         #2: 'P',
         #3: 'I',
         #4: 'D',
         #5: 'Px',
         #6: 'Py',
         #7: 'Pphi',
         #8: 'Ix',
         #9: 'Iy',
         #10: 'Iphi',
         #11: 'Dx',
         #12: 'Dy',
         #13: 'Dphi',
         14: 'targVelX',
         #15: 'targVelY',
         #16: 'targVelPhi',
         17: 'curPosX',
         #18: 'curPosY',
         #19: 'curPosPhi',
         20: 'targPosX',
         #21: 'targPosY',
         #22: 'targPosPhi',
         23: 'curVelX',
         #24: 'curVelY',
         #25: 'curVelPhi'
         }

   # Determine range in y for final plot:
   # min = min(columns) - 1
   # max = max(columns) + 1
   # This is used to automatically scale the plot to the plot values
   minval = 0.0
   maxval = 0.0
   added_range = 0.5
   with open('pidplot.dat', 'r') as f:
      while (True):
         line = f.readline().strip()
         if line == '':
            break

         line_vals = line.split(' ')
         #Update min and max
         for col in columns.keys():
            colIdx = col-1 # starts at 1
            if (float(line_vals[colIdx]) - added_range) < minval:
               minval = float(line_vals[colIdx]) - added_range
            elif (float(line_vals[colIdx]) + added_range) > maxval:
               maxval = float(line_vals[colIdx]) + added_range

   gp = Gnuplot.Gnuplot()
   gp.clear()
   gp.title('PID response')
   gp.set_range('yrange', (minval, maxval))
   gp.xlabel("program cycle number")

   #kpval = 0.5
   #kival = 0.0
   #kdval = 0.0
   #PIDplot(kpval, kival, kdval)
   for col in columns.keys():
      gp.replot( Gnuplot.File('pidplot.dat', using=(1,col), with_="lines", title = columns[col]) )
       
   raw_input("Press any key to close the graph.")
