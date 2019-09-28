 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the OpenCV Foundation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "precomp.hpp"
#include "debug.hpp"
#include "opencv2/core/core_c.h"
#include <iostream>
//#include <stdio.h>

using namespace std;
namespace cv{namespace optim{
    class DownhillSolverImpl : public DownhillSolver
    {
    public:
        void getInitStep(OutputArray step) const;
        void setInitStep(InputArray step);
        Ptr<Function> getFunction() const;
        void setFunction(const Ptr<Function>& f);
        TermCriteria getTermCriteria() const;
        DownhillSolverImpl();
        void setTermCriteria(const TermCriteria& termcrit);
        double minimize(InputOutputArray x);
        void setMinConstraint(double constraint[], int length);
        void setMaxConstraint(double constraint[], int length);
        int numberOfTries() { return numTries; }

    protected:
        Ptr<Solver::Function> _Function;
        TermCriteria _termcrit;
        Mat _step;
        Mat_<double> buf_x;

    private:
        inline void createInitialSimplex(Mat_<double>& simplex,Mat& step);
        inline double innerDownhillSimplex(cv::Mat_<double>& p,double MinRange,double MinError,int& nfunk,
                const Ptr<Solver::Function>& f,int nmax);
        inline double tryNewPoint(Mat_<double>& p,Mat_<double>& y,Mat_<double>& coord_sum,const Ptr<Solver::Function>& f,int ihi,
                double fac,Mat_<double>& ptry);

        double minConstraint[3];
        double maxConstraint[3];
        int numTries;
    };

    double DownhillSolverImpl::tryNewPoint(
        Mat_<double>& p,
        Mat_<double>& y,
        Mat_<double>&  coord_sum,
        const Ptr<Solver::Function>& f,
        int      ihi,
        double   fac,
        Mat_<double>& ptry
        )
    {
        int ndim=p.cols;
        int j;
        double fac1,fac2,ytry;

        fac1=(1.0-fac)/ndim;
        fac2=fac1-fac;
        for (j=0;j<ndim;j++)
        {
        	double try_value = coord_sum(j)*fac1-p(ihi,j)*fac2;
        	if (try_value < minConstraint[j]){
				ptry(j) =  minConstraint[j];
				//cout << "min constraint: " << minConstraint[j]<< "\n";
			}
			else{
				if (try_value > maxConstraint[j]){
				     ptry(j) =  maxConstraint[j];
				     //cout << "max constraint " << maxConstraint[j] << "\n";
				}
				else{
					ptry(j)=try_value;
				}
			}



        }
        ytry=f->calc((double*)ptry.data);
        if (ytry < y(ihi))
        {
            y(ihi)=ytry;
            for (j=0;j<ndim;j++)
            {
                coord_sum(j) += ptry(j)-p(ihi,j);
                p(ihi,j)=ptry(j);
            }
        }
        return ytry;
    }

    /*
    Performs the actual minimization of Solver::Function f (after the initialization was done)

    The matrix p[ndim+1][1..ndim] represents ndim+1 vertices that
    form a simplex - each row is an ndim vector.
    On output, nfunk gives the number of function evaluations taken.
    */
    double DownhillSolverImpl::innerDownhillSimplex(
        cv::Mat_<double>&   p,
        double     MinRange,
        double     MinError,
        int&       nfunk,
        const Ptr<Solver::Function>& f,
        int nmax
        )
    {
        int ndim=p.cols;
        double res;
        int i,ihi,ilo,inhi,j,mpts=ndim+1;
        double error, range,ysave,ytry;
        Mat_<double> coord_sum(1,ndim,0.0),buf(1,ndim,0.0),y(1,ndim,0.0);

        nfunk = 0;

        for(i=0;i<ndim+1;++i)
        {
            y(i) = f->calc(p[i]);
            //std::cout << *p[i] << " " << y(i) << "\n";
        }

        nfunk = ndim+1;

        reduce(p,coord_sum,0,CV_REDUCE_SUM);

        for (;;)
        {
            ilo=0;
            /*  find highest (worst), next-to-worst, and lowest
                (best) points by going through all of them. */
            ihi = y(0)>y(1) ? (inhi=1,0) : (inhi=0,1);
            for (i=0;i<mpts;i++)
            {
                if (y(i) <= y(ilo))
                    ilo=i;
                if (y(i) > y(ihi))
                {
                    inhi=ihi;
                    ihi=i;
                }
                else if (y(i) > y(inhi) && i != ihi)
                    inhi=i;
            }

            /* check stop criterion */
            error=fabs(y(ihi)-y(ilo));
            range=0;
            for(i=0;i<ndim;++i)
            {
                double min = p(0,i);
                double max = p(0,i);
                double d;
                for(j=1;j<=ndim;++j)
                {
                    if( min > p(j,i) ) min = p(j,i);
                    if( max < p(j,i) ) max = p(j,i);
                }
                d = fabs(max-min);
                if(range < d) range = d;
            }

            if(range <= MinRange || error <= MinError)
            { /* Put best point and value in first slot. */
                std::swap(y(0),y(ilo));
                for (i=0;i<ndim;i++)
                {
                    std::swap(p(0,i),p(ilo,i));
                }
                break;
            }

            if (nfunk >= nmax){
                //dprintf(("nmax exceeded\n"));
                return y(ilo);
            }
            nfunk += 2;
            /*Begin a new iteration. First, reflect the worst point about the centroid of others */
            //cout << "Iteration: " << nfunk << "\n";
            // Here goes to 0.0
           // cout << "Here goes to 0.0 \n";
            ytry = tryNewPoint(p,y,coord_sum,f,ihi,-1.0,buf);
            if (ytry <= y(ilo))
            { /*If that's better than the best point, go twice as far in that direction*/
                ytry = tryNewPoint(p,y,coord_sum,f,ihi,2.0,buf);
            }
            else if (ytry >= y(inhi))
            {   /* The new point is worse than the second-highest, but better
                  than the worst so do not go so far in that direction */
                ysave = y(ihi);
                ytry = tryNewPoint(p,y,coord_sum,f,ihi,0.5,buf);
                if (ytry >= ysave)
                { /* Can't seem to improve things. Contract the simplex to good point
               in hope to find a simplex landscape. */
                    for (i=0;i<mpts;i++)
                    {
                        if (i != ilo)
                        {
                            for (j=0;j<ndim;j++)
                            {
                                p(i,j) = coord_sum(j) = 0.5*(p(i,j)+p(ilo,j));
                            }
                            y(i)=f->calc((double*)coord_sum.data);
                        }
                    }
                    nfunk += ndim;
                    reduce(p,coord_sum,0,CV_REDUCE_SUM);
                }
            } else --(nfunk); /* correct nfunk */
            //dprintf(("this is simplex on iteration %d\n",nfunk));
        } /* go to next iteration. */
        res = y(0);

        return res;
    }

    void DownhillSolverImpl::createInitialSimplex(Mat_<double>& simplex,Mat& step){
#ifndef NONO
		for(int i=1;i<=step.cols;++i)
		{
			simplex.row(0).copyTo(simplex.row(i));
			simplex(i,i-1)+= 0.5*step.at<double>(0,i-1);
			//cout << "step.at<double>(0,i-1): " << step.at<double>(0,i-1) << "\n";

		}
		simplex.row(0) -= 0.5*step;
		// APOX: so now we have one "lower" point where x,y and rz are all low, and three "higher" points where only
		// one dimension is high.
#else
		// APOX: below does not work very well !
    	Mat_<double> center;
		simplex.row(0).copyTo(center); // now we have the center point or start point in the 3d space
		// create 4 points, where the x,y and rz only can be value-step or value+step so the center point
		// is still the center of the 4 created points in the 3d space
		simplex(0,0) = center.at<double>(0,0) + 0.5*step.at<double>(0,0);
		simplex(0,1) = center.at<double>(0,1) - 0.5*step.at<double>(0,1);
		simplex(0,2) = center.at<double>(0,2) - 0.5*step.at<double>(0,2);

		simplex(1,0) = center.at<double>(0,0) - 0.5*step.at<double>(0,0);
		simplex(1,1) = center.at<double>(0,1) + 0.5*step.at<double>(0,1);
		simplex(1,2) = center.at<double>(0,2) + 0.5*step.at<double>(0,2);

		simplex(2,0) = center.at<double>(0,0) - 0.5*step.at<double>(0,0);
		simplex(2,1) = center.at<double>(0,1) - 0.5*step.at<double>(0,1);
		simplex(2,2) = center.at<double>(0,2) + 0.5*step.at<double>(0,2);

		simplex(3,0) = center.at<double>(0,0) + 0.5*step.at<double>(0,0);
		simplex(3,1) = center.at<double>(0,1) + 0.5*step.at<double>(0,1);
		simplex(3,2) = center.at<double>(0,2) - 0.5*step.at<double>(0,2);
#endif

		// min and max the new created points so they are all inside the field
		for(int jj=0;jj<simplex.rows;++jj)
		{
			for(int ii=0;ii<simplex.cols;++ii)
			{
				if( simplex(jj,ii) < minConstraint[ii] ) { simplex(jj,ii) = minConstraint[ii]; }
				else if( simplex(jj,ii) > maxConstraint[ii] ) { simplex(jj,ii) = maxConstraint[ii]; }
			}
		}
		//dprintf(("this is simplex\n"));
		// cout << "simplex " << endl << simplex << endl;
        }

    double DownhillSolverImpl::minimize(InputOutputArray x){
       // dprintf(("hi from minimize\n"));
        //CV_Assert(_Function.empty()==false);
        //dprintf(("termcrit:\n\ttype: %d\n\tmaxCount: %d\n\tEPS: %g\n",_termcrit.type,_termcrit.maxCount,_termcrit.epsilon));
        //dprintf(("step\n"));

        Mat x_mat=x.getMat(); // APOX: x is already the correct Mat type, so the function just returns the input pointer and x_mat is the same as x.
       // CV_Assert(MIN(x_mat.rows,x_mat.cols)==1);
       // CV_Assert(MAX(x_mat.rows,x_mat.cols)==_step.cols);
       // CV_Assert(x_mat.type()==CV_64FC1);

        Mat_<double> proxy_x; // APOX: intermediate variable to set the first row with the center point x in the simplex 3 dimensional space

        if(x_mat.rows>1){
            buf_x.create(1,_step.cols);
            Mat_<double> proxy(_step.cols,1,(double*)buf_x.data);
            x_mat.copyTo(proxy);
            proxy_x=buf_x;
            cout << "not used in RoboCup" << endl;
        }else{
            proxy_x=x_mat; // APOX: so proxy_x is pointing to x_mat, this is the starting point of the search
        }

        numTries=0;
        int ndim=_step.cols; // APOX: this one is set via the step init and has 3 cols, so ndim is 3
        Mat_<double> simplex= Mat_<double>(ndim+1,ndim, 0.0); // APOX: create a matrix that contains area where to start from, it has 4 points in the 3 dimensional space

        proxy_x.copyTo(simplex.row(0)); // APOX: set the first row with the center point x in the the simplex 3 dimensional space, not row (point) is used with the step to calculate the 4 points in the 3 dimensionals space, the point itself is overwritten (not used)
        createInitialSimplex(simplex,_step); // APOX: now the 4 (outer) points in the 3 dimensional space have been created
        // APOX: now start the simplex algorithm with these 4 points
        //  the minRange (epsilon) to exit, probably the distance of the points
        //  the minError (epsilon) to exit, probably the difference between the scores of the 4 points
        //  return amount of tries
        //  the cost table function (calculate)
        //  and the maximal numTries before exit
        double res = innerDownhillSimplex(simplex, _termcrit.epsilon, _termcrit.epsilon, numTries, _Function, _termcrit.maxCount);
        simplex.row(0).copyTo(proxy_x); // APOX: not used

        if(x_mat.rows>1){
            Mat(x_mat.rows, 1, CV_64F, (double*)proxy_x.data).copyTo(x);
            cout << "not used in RoboCup" << endl;
        }
        return res;
    }
    DownhillSolverImpl::DownhillSolverImpl(){
        _Function=Ptr<Function>();
        _step=Mat_<double>();
    }
    Ptr<Solver::Function> DownhillSolverImpl::getFunction()const{
        return _Function;
    }
    void DownhillSolverImpl::setFunction(const Ptr<Function>& f){
        _Function=f;
    }
    TermCriteria DownhillSolverImpl::getTermCriteria()const{
        return _termcrit;
    }
    void DownhillSolverImpl::setTermCriteria(const TermCriteria& termcrit){
       // CV_Assert(termcrit.type==(TermCriteria::MAX_ITER+TermCriteria::EPS) && termcrit.epsilon>0 && termcrit.maxCount>0);
        _termcrit=termcrit;
    }
    // both minRange & minError are specified by termcrit.epsilon; In addition, user may specify the number of iterations that the algorithm does.
    Ptr<DownhillSolver> createDownhillSolver(const Ptr<Solver::Function>& f, InputArray initStep, TermCriteria termcrit){
        DownhillSolver *DS=new DownhillSolverImpl();
        DS->setFunction(f);
        DS->setInitStep(initStep);
        DS->setTermCriteria(termcrit);
        return Ptr<DownhillSolver>(DS);
    }
    void DownhillSolverImpl::getInitStep(OutputArray step)const{
        _step.copyTo(step);
    }
    void DownhillSolverImpl::setInitStep(InputArray step){
        //set dimensionality and make a deep copy of step
        Mat m=step.getMat();
       // dprintf(("m.cols=%d\nm.rows=%d\n",m.cols,m.rows));
      //  CV_Assert(MIN(m.cols,m.rows)==1 && m.type()==CV_64FC1);
        if(m.rows==1){
            m.copyTo(_step);
        }else{
            transpose(m,_step);
        }
    }
    void DownhillSolverImpl::setMinConstraint(double constraint[], int length){

    	for(int i(0); i < length; ++i)
    	    {
    			minConstraint[i] = constraint[i];
    	    }

    }

    void DownhillSolverImpl::setMaxConstraint(double constraint[], int length){

    	for(int i(0); i < length; ++i)
    	    {
    	        //std::cout << data[i] << ' ';
    			maxConstraint[i] = constraint[i];
    	    }
    }

}}
