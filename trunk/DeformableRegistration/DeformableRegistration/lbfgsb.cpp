/*************************************************************************
NEOS, November 1994. (Latest revision June 1996.)
Optimization Technology Center.
Argonne National Laboratory and Northwestern University.

Written by Ciyou Zhu in collaboration with
R.H. Byrd, P. Lu-Chen and J. Nocedal.

Contributors:
    * Sergey Bochkanov (ALGLIB project). Translation from FORTRAN to
      pseudocode.
      
This software is freely available, but we  expect  that  all  publications
describing  work using this software, or all commercial products using it,
quote at least one of the references given below:
    * R. H. Byrd, P. Lu and J. Nocedal.  A Limited  Memory  Algorithm  for
      Bound Constrained Optimization, (1995), SIAM Journal  on  Scientific
      and Statistical Computing , 16, 5, pp. 1190-1208.
    * C. Zhu, R.H. Byrd and J. Nocedal. L-BFGS-B: Algorithm 778: L-BFGS-B,
      FORTRAN routines for  large  scale  bound  constrained  optimization
      (1997), ACM Transactions on Mathematical Software,  Vol 23,  Num. 4,
      pp. 550 - 560.
*************************************************************************/

// #include <stdafx.h>
// #include "stdafx.h"
#include "lbfgsb.h"
#include "DeformationGraph.h"
#include "OpenMeshWrapper.h"
#include "Viewer.h"

using namespace  std;

// Assuming A = [x1 x4 x7] b = [x10] u = [x13] w = [x15]
//              [x2 x5 x8]     [x11]     [x14]
//              [x3 x6 x9]     [x12]
//
// total 15*n variables

void funcgrad(const ap::real_1d_array& x, double& f, ap::real_1d_array& g,
	DeformationGraph& graph,
	CTriMesh& mesh,
	std::vector<double>& alph,
	const std::vector<std::vector<double>>& target_dmap,
	const std::vector<std::vector<double>>& pdx,
	const std::vector<std::vector<double>>& pdy,
	std::vector<std::vector<std::vector<double>>>& coef_map
	)
{
	// initialize
	for(int i = g.getlowbound(); i <= g.gethighbound(); ++i) g(i) = 0;
	f = 0.0;

	int y_res = target_dmap.size();
	int x_res = target_dmap[0].size();

	// energy terms
	double E_rigid = 0.0;
	double E_smooth = 0.0;
	double E_conf = 0.0;
	double E_fit = 0.0;

	// weight values of energy
	double a_rigid = alph[0];
	double a_smooth = alph[1];
	double a_fit = alph[2];
	double a_conf = alph[3];

	// the number of graph nodes
	int n = (((x.gethighbound() - x.getlowbound() + 1))/15 == graph.nodes.size()) ? graph.nodes.size() : 0;
	for(int i = 0; i < n; ++i)
	{
		//////////////////////////////////////////////////////////////////////////
		// E_rigid
		//////////////////////////////////////////////////////////////////////////
		double c1c1 = x(15*i+1)*x(15*i+1) + x(15*i+2)*x(15*i+2) + x(15*i+3)*x(15*i+3);
		double c2c2 = x(15*i+4)*x(15*i+4) + x(15*i+5)*x(15*i+5) + x(15*i+6)*x(15*i+6);
		double c3c3 = x(15*i+7)*x(15*i+7) + x(15*i+8)*x(15*i+8) + x(15*i+9)*x(15*i+9);
		double c1c2 = x(15*i+1)*x(15*i+4) + x(15*i+2)*x(15*i+5) + x(15*i+3)*x(15*i+6);
		double c1c3 = x(15*i+1)*x(15*i+7) + x(15*i+2)*x(15*i+8) + x(15*i+3)*x(15*i+9);
		double c2c3 = x(15*i+4)*x(15*i+7) + x(15*i+5)*x(15*i+8) + x(15*i+6)*x(15*i+9);

// 		E_rigid += c1c2*c1c2 + c1c3*c1c3 + c2c3*c2c3 + (c1c1-1)*(c1c1-1) + (c2c2-1)*(c2c2-1) + (c3c3-1)*(c3c3-1);
// 
// 		g(15*i+1) += a_rigid*(2*c1c2*x(15*i+4) + 2*c1c3*x(15*i+7) + 4*(c1c1-1)*x(15*i+1));
// 		g(15*i+2) += a_rigid*(2*c1c2*x(15*i+5) + 2*c1c3*x(15*i+8) + 4*(c1c1-1)*x(15*i+2));
// 		g(15*i+3) += a_rigid*(2*c1c2*x(15*i+6) + 2*c1c3*x(15*i+9) + 4*(c1c1-1)*x(15*i+3));
// 		g(15*i+4) += a_rigid*(2*c1c2*x(15*i+1) + 2*c2c3*x(15*i+7) + 4*(c2c2-1)*x(15*i+4));
// 		g(15*i+5) += a_rigid*(2*c1c2*x(15*i+2) + 2*c2c3*x(15*i+8) + 4*(c2c2-1)*x(15*i+5));
// 		g(15*i+6) += a_rigid*(2*c1c2*x(15*i+3) + 2*c2c3*x(15*i+9) + 4*(c2c2-1)*x(15*i+6));
// 		g(15*i+7) += a_rigid*(2*c1c3*x(15*i+1) + 2*c2c3*x(15*i+4) + 4*(c3c3-1)*x(15*i+7));
// 		g(15*i+8) += a_rigid*(2*c1c3*x(15*i+2) + 2*c2c3*x(15*i+5) + 4*(c3c3-1)*x(15*i+8));
// 		g(15*i+9) += a_rigid*(2*c1c3*x(15*i+3) + 2*c2c3*x(15*i+6) + 4*(c3c3-1)*x(15*i+9));

		//////////////////////////////////////////////////////////////////////////
		// E_smooth
		//////////////////////////////////////////////////////////////////////////
		std::vector<int> nei;	graph.GetNeighbors(i, nei);
		double xi[3] = {graph.nodes[i][0], graph.nodes[i][1], graph.nodes[i][2]};
		double sig_reg[3];
		for(size_t j = 0; j < nei.size(); j++)
		{
			double xj[3] = {graph.nodes[nei[j]][0], graph.nodes[nei[j]][1], graph.nodes[nei[j]][2]};

			// sig_reg = Ai(xj-xi) + xi + bi - (xj + bj)
			sig_reg[0] = x(15*i+1)*(xj[0]-xi[0]) + x(15*i+4)*(xj[1] - xi[1]) + x(15*i+7)*(xj[2] - xi[2]);		// sig_reg = Rj(xj-xi)
			sig_reg[1] = x(15*i+2)*(xj[0]-xi[0]) + x(15*i+5)*(xj[1] - xi[1]) + x(15*i+8)*(xj[2] - xi[2]);
			sig_reg[2] = x(15*i+3)*(xj[0]-xi[0]) + x(15*i+6)*(xj[1] - xi[1]) + x(15*i+9)*(xj[2] - xi[2]);
			sig_reg[0] += xi[0] + x(15*i+10) - xj[0] - x(15*nei[j]+10);											// sig_reg += xi + bi - (xj + bj)
			sig_reg[1] += xi[1] + x(15*i+11) - xj[1] - x(15*nei[j]+11);
			sig_reg[2] += xi[2] + x(15*i+12) - xj[2] - x(15*nei[j]+12);

// 			E_smooth += sig_reg[0]*sig_reg[0] + sig_reg[1]*sig_reg[1] + sig_reg[2]*sig_reg[2];				// use alpha_jk = 1.0
// 
// 			g(15*i+1) += a_smooth*2*sig_reg[0]*(xj[0]-xi[0]);
// 			g(15*i+2) += a_smooth*2*sig_reg[1]*(xj[0]-xi[0]);
// 			g(15*i+3) += a_smooth*2*sig_reg[2]*(xj[0]-xi[0]);
// 			g(15*i+4) += a_smooth*2*sig_reg[0]*(xj[1]-xi[1]);
// 			g(15*i+5) += a_smooth*2*sig_reg[1]*(xj[1]-xi[1]);
// 			g(15*i+6) += a_smooth*2*sig_reg[2]*(xj[1]-xi[1]);
// 			g(15*i+7) += a_smooth*2*sig_reg[0]*(xj[2]-xi[2]);
// 			g(15*i+8) += a_smooth*2*sig_reg[1]*(xj[2]-xi[2]);
// 			g(15*i+9) += a_smooth*2*sig_reg[2]*(xj[2]-xi[2]);
// 
// 			g(15*i+10) += a_smooth*2*sig_reg[0];
// 			g(15*i+11) += a_smooth*2*sig_reg[1];
// 			g(15*i+12) += a_smooth*2*sig_reg[2];
// 			g(15*nei[j]+10) -= a_smooth*2*sig_reg[0];
// 			g(15*nei[j]+11) -= a_smooth*2*sig_reg[1];
// 			g(15*nei[j]+12) -= a_smooth*2*sig_reg[2];
		}


		//////////////////////////////////////////////////////////////////////////
		// E_fit
		//////////////////////////////////////////////////////////////////////////
		double u = x(i*15+13);
		double v = x(i*15+14);

		// bilinear interpolation
		//////////////////////////////////////////////////////////////////////////
		double d1, d2, d, d_du, d_dv, dp, au, av;
		au = u-int(u);
		av = v-int(v);
		if (u+1>=x_res-1 || v+1>=y_res-1)
		{
// 			cout<< "opps..";
			d = target_dmap[v][u];
			d_du = pdx[v][u];
			d_dv = pdy[v][u];
		}
		else
		{
			d1 = (1-au)*target_dmap[int(v)][int(u)] + au*target_dmap[int(v)][int(u)+1];
			d2 = (1-au)*target_dmap[int(v)+1][int(u)] + au*target_dmap[int(v)+1][int(u)+1];
			d = (1-av)*d1+av*d2;

			d1 = (1-au)*pdx[int(v)][int(u)] + au*pdx[int(v)][int(u)+1];
			d2 = (1-au)*pdx[int(v)+1][int(u)] + au*pdx[int(v)+1][int(u)+1];
			d_du = (1-av)*d1+av*d2;

			d1 = (1-au)*pdy[int(v)][int(u)] + au*pdy[int(v)][int(u)+1];
			d2 = (1-au)*pdy[int(v)+1][int(u)] + au*pdy[int(v)+1][int(u)+1];
			d_dv = (1-av)*d1+av*d2;
		}
		
		double c_hat[3] = {u, v, d};

		double x_hat[3];
		for (int j = 0; j<3; ++j)
			x_hat[j] = graph.nodes[i][j] + x(15*i+10+j);

		double xh_ch[3];
		for (int j = 0; j<3; ++j)
			xh_ch[j] = x_hat[j] - c_hat[j];

		double xh_ch_norm2 = xh_ch[0]*xh_ch[0] + xh_ch[1]*xh_ch[1] + xh_ch[2]*xh_ch[2];	// ||x_hat - c_hat||^2
		//double w_sqr = x(15*i+15)*x(15*i+15);
		double w_sqr = 1;

		E_fit += w_sqr*xh_ch_norm2;

		// my new
		// x_hat - c_hat
		// {x1+b1-u}
		// {x2+b2-v}
		// {x3+b3-d}
		g(15*i+10) += a_fit*w_sqr*2*xh_ch[0];
		g(15*i+11) += a_fit*w_sqr*2*xh_ch[1];
		g(15*i+12) += a_fit*w_sqr*2*xh_ch[2];

		g(15*i+13) -= a_fit*w_sqr*2*(xh_ch[0] + xh_ch[2]*d_du);
		g(15*i+14) -= a_fit*w_sqr*2*(xh_ch[1] + xh_ch[2]*d_dv);

		//g(15*i+15) += a_fit*2*x(15*i+15)*xh_ch_norm2;

		//////////////////////////////////////////////////////////////////////////
		// E_conf
		//////////////////////////////////////////////////////////////////////////
		double conf_tem = 1-w_sqr;
// 		E_conf += conf_tem*conf_tem;
// 		g(15*i+15) -= a_conf*4*x(15*i+15)*conf_tem;
	}

	cout<<g(15*100+10)<<endl;
	cout<<g(15*100+11)<<endl;
	cout<<g(15*100+12)<<endl;


	f = a_rigid*E_rigid + a_smooth*E_smooth + a_fit*E_fit + a_conf*E_conf;
	cout<<endl<<endl;
	cout<<"E_rigid  : "<<E_rigid<<endl;
	cout<<"E_smooth : "<<E_smooth<<endl;
	cout<<"E_fit    : "<<E_fit<<endl;
	cout<<"E_conf   : "<<E_conf<<endl;
	cout<<"a*E_rigid  : "<<a_rigid*E_rigid<<endl;
	cout<<"a*E_smooth : "<<a_smooth*E_smooth<<endl;
	cout<<"a*E_fit    : "<<a_fit*E_fit<<endl;
	cout<<"a*E_conf   : "<<a_conf*E_conf<<endl;
	cout<<endl<<endl;
}

static void lbfgsbactive(const int& n,
     const ap::real_1d_array& l,
     const ap::real_1d_array& u,
     const ap::integer_1d_array& nbd,
     ap::real_1d_array& x,
     ap::integer_1d_array& iwhere,
     bool& prjctd,
     bool& cnstnd,
     bool& boxed);
static void lbfgsbbmv(const int& m,
     const ap::real_2d_array& sy,
     ap::real_2d_array& wt,
     const int& col,
     const ap::real_1d_array& v,
     ap::real_1d_array& p,
     int& info,
     ap::real_1d_array& workvec);
static void lbfgsbcauchy(const int& n,
     const ap::real_1d_array& x,
     const ap::real_1d_array& l,
     const ap::real_1d_array& u,
     const ap::integer_1d_array& nbd,
     const ap::real_1d_array& g,
     ap::integer_1d_array& iorder,
     ap::integer_1d_array& iwhere,
     ap::real_1d_array& t,
     ap::real_1d_array& d,
     ap::real_1d_array& xcp,
     const int& m,
     const ap::real_2d_array& wy,
     const ap::real_2d_array& ws,
     const ap::real_2d_array& sy,
     ap::real_2d_array& wt,
     const double& theta,
     const int& col,
     const int& head,
     ap::real_1d_array& p,
     ap::real_1d_array& c,
     ap::real_1d_array& wbp,
     ap::real_1d_array& v,
     int& nint,
     const ap::real_1d_array& sg,
     const ap::real_1d_array& yg,
     const double& sbgnrm,
     int& info,
     ap::real_1d_array& workvec);
static void lbfgsbcmprlb(const int& n,
     const int& m,
     const ap::real_1d_array& x,
     const ap::real_1d_array& g,
     const ap::real_2d_array& ws,
     const ap::real_2d_array& wy,
     const ap::real_2d_array& sy,
     ap::real_2d_array& wt,
     const ap::real_1d_array& z,
     ap::real_1d_array& r,
     ap::real_1d_array& wa,
     const ap::integer_1d_array& index,
     const double& theta,
     const int& col,
     const int& head,
     const int& nfree,
     const bool& cnstnd,
     int& info,
     ap::real_1d_array& workvec,
     ap::real_1d_array& workvec2);
static void lbfgsberrclb(const int& n,
     const int& m,
     const double& factr,
     const ap::real_1d_array& l,
     const ap::real_1d_array& u,
     const ap::integer_1d_array& nbd,
     int& task,
     int& info,
     int& k);
static void lbfgsbformk(const int& n,
     const int& nsub,
     const ap::integer_1d_array& ind,
     const int& nenter,
     const int& ileave,
     const ap::integer_1d_array& indx2,
     const int& iupdat,
     const bool& updatd,
     ap::real_2d_array& wn,
     ap::real_2d_array& wn1,
     const int& m,
     const ap::real_2d_array& ws,
     const ap::real_2d_array& wy,
     const ap::real_2d_array& sy,
     const double& theta,
     const int& col,
     const int& head,
     int& info,
     ap::real_1d_array& workvec,
     ap::real_2d_array& workmat);
static void lbfgsbformt(const int& m,
     ap::real_2d_array& wt,
     const ap::real_2d_array& sy,
     const ap::real_2d_array& ss,
     const int& col,
     const double& theta,
     int& info);
static void lbfgsbfreev(const int& n,
     int& nfree,
     ap::integer_1d_array& index,
     int& nenter,
     int& ileave,
     ap::integer_1d_array& indx2,
     const ap::integer_1d_array& iwhere,
     bool& wrk,
     const bool& updatd,
     const bool& cnstnd,
     const int& iter);
static void lbfgsbhpsolb(const int& n,
     ap::real_1d_array& t,
     ap::integer_1d_array& iorder,
     const int& iheap);
static void lbfgsblnsrlb(const int& n,
     const ap::real_1d_array& l,
     const ap::real_1d_array& u,
     const ap::integer_1d_array& nbd,
     ap::real_1d_array& x,
     const double& f,
     double& fold,
     double& gd,
     double& gdold,
     const ap::real_1d_array& g,
     const ap::real_1d_array& d,
     ap::real_1d_array& r,
     ap::real_1d_array& t,
     const ap::real_1d_array& z,
     double& stp,
     double& dnrm,
     double& dtd,
     double& xstep,
     double& stpmx,
     const int& iter,
     int& ifun,
     int& iback,
     int& nfgv,
     int& info,
     int& task,
     const bool& boxed,
     const bool& cnstnd,
     int& csave,
     ap::integer_1d_array& isave,
     ap::real_1d_array& dsave);
static void lbfgsbmatupd(const int& n,
     const int& m,
     ap::real_2d_array& ws,
     ap::real_2d_array& wy,
     ap::real_2d_array& sy,
     ap::real_2d_array& ss,
     const ap::real_1d_array& d,
     const ap::real_1d_array& r,
     int& itail,
     const int& iupdat,
     int& col,
     int& head,
     double& theta,
     const double& rr,
     const double& dr,
     const double& stp,
     const double& dtd);
static void lbfgsbprojgr(const int& n,
     const ap::real_1d_array& l,
     const ap::real_1d_array& u,
     const ap::integer_1d_array& nbd,
     const ap::real_1d_array& x,
     const ap::real_1d_array& g,
     double& sbgnrm);
static void lbfgsbsubsm(const int& n,
     const int& m,
     const int& nsub,
     const ap::integer_1d_array& ind,
     const ap::real_1d_array& l,
     const ap::real_1d_array& u,
     const ap::integer_1d_array& nbd,
     ap::real_1d_array& x,
     ap::real_1d_array& d,
     const ap::real_2d_array& ws,
     const ap::real_2d_array& wy,
     const double& theta,
     const int& col,
     const int& head,
     int& iword,
     ap::real_1d_array& wv,
     ap::real_2d_array& wn,
     int& info);
static void lbfgsbdcsrch(const double& f,
     const double& g,
     double& stp,
     const double& ftol,
     const double& gtol,
     const double& xtol,
     const double& stpmin,
     const double& stpmax,
     int& task,
     ap::integer_1d_array& isave,
     ap::real_1d_array& dsave,
     int& addinfo);
static void lbfgsbdcstep(double& stx,
     double& fx,
     double& dx,
     double& sty,
     double& fy,
     double& dy,
     double& stp,
     const double& fp,
     const double& dp,
     bool& brackt,
     const double& stpmin,
     const double& stpmax);
static bool additionallbfgsbstoppingcriterion(int iter,
     const ap::real_1d_array& x,
     double f,
     const ap::real_1d_array& g);
static bool lbfgsbdpofa(ap::real_2d_array& a, const int& n);
static void lbfgsbdtrsl(ap::real_2d_array& t,
     const int& n,
     ap::real_1d_array& b,
     const int& job,
     int& info);
static void lbfgsbnewiteration(const ap::real_1d_array& x,
     double f,
     const ap::real_1d_array& g);

/*************************************************************************
The  subroutine  minimizes  the  function  F(x) of N arguments with simple
constraints using a quasi-Newton method (LBFGS scheme) which is  optimized
to use a minimum amount of memory.

The subroutine generates the approximation of an inverse Hessian matrix by
using information about the last M steps of the algorithm (instead  of N).
It lessens a required amount of memory from a value  of  order  N^2  to  a
value of order 2*N*M.

This subroutine uses the FuncGrad subroutine which calculates the value of
the function F and gradient G in point X. The programmer should define the
FuncGrad subroutine by himself.  It should be noted  that  the  subroutine
doesn't need to waste  time for memory allocation of array G, because  the
memory is allocated in calling the  subroutine.  Setting  a  dimension  of
array G each time when calling a subroutine will excessively slow down  an
algorithm.

The programmer could also redefine the LBFGSNewIteration subroutine  which
is called on each new step. The current point X, the function value F  and
the gradient G are passed  into  this  subroutine.  It  is  reasonable  to
redefine the subroutine for better debugging, for  example,  to  visualize
the solution process.

Input parameters:
    N       -   problem dimension. N>0
    M       -   number of  corrections  in  the  BFGS  scheme  of  Hessian
                approximation  update.  Recommended value:  3<=M<=7.   The
                smaller value causes worse convergence,  the  bigger  will
                not  cause  a  considerably  better  convergence, but will
                cause a fall in the performance. M<=N.
    X       -   initial solution approximation.
                Array whose index ranges from 1 to N.
    EpsG    -   positive number which defines a precision of  search.  The
                subroutine finishes its work if the condition ||G|| < EpsG
                is satisfied, where ||.|| means Euclidian norm, G - gradient
                projection onto a feasible set, X - current approximation.
    EpsF    -   positive number which defines a precision of  search.  The
                subroutine  finishes  its  work if on iteration number k+1
                the condition |F(k+1)-F(k)| <= EpsF*max{|F(k)|, |F(k+1)|, 1}
                is satisfied.
    EpsX    -   positive number which defines a precision of  search.  The
                subroutine  finishes  its  work if on iteration number k+1
                the condition |X(k+1)-X(k)| <= EpsX is satisfied.
    MaxIts  -   maximum number of iterations.
                If MaxIts=0, the number of iterations is unlimited.
    NBD     -   constraint type. If NBD(i) is equal to:
                * 0, X(i) has no constraints,
                * 1, X(i) has only lower boundary,
                * 2, X(i) has both lower and upper boundaries,
                * 3, X(i) has only upper boundary,
                Array whose index ranges from 1 to N.
    L       -   lower boundaries of X(i) variables.
                Array whose index ranges from 1 to N.
    U       -   upper boundaries of X(i) variables.
                Array whose index ranges from 1 to N.

Output parameters:
    X       -   solution approximation.
Array whose index ranges from 1 to N.
    Info    -   a return code:
                    * -2 unknown internal error,
                    * -1 wrong parameters were specified,
                    * 0 interrupted by user,
                    * 1 relative function decreasing is less or equal to EpsF,
                    * 2 step is less or equal to EpsX,
                    * 4 gradient norm is less or equal to EpsG,
                    * 5 number of iterations exceeds MaxIts.

FuncGrad routine description. User-defined.
Input parameters:
    X   -   array whose index ranges from 1 to N.
Output parameters:
    F   -   function value at X.
    G   -   function gradient.
            Array whose index ranges from 1 to N.
The memory for array G has already been allocated in the calling subroutine,
and it isn't necessary to allocate it in the FuncGrad subroutine.

    NEOS, November 1994. (Latest revision June 1996.)
    Optimization Technology Center.
    Argonne National Laboratory and Northwestern University.

    Written by Ciyou Zhu in collaboration with
    R.H. Byrd, P. Lu-Chen and J. Nocedal.
*************************************************************************/
void lbfgsbminimize(const int& n,
     const int& m,
	 ap::real_1d_array& x,
	 DeformationGraph& graph,
	 CTriMesh& mesh,
	 std::vector<double>& alph,
	 const std::vector<std::vector<double>>& dmap,
	 const std::vector<std::vector<double>>& pdx,
	 const std::vector<std::vector<double>>& pdy,
	 std::vector<std::vector<std::vector<double>>>& coef_map,
	 Viewer* viewer,
	 const double& epsg,
     const double& epsf,
     const double& epsx,
     const int& maxits,
     const ap::integer_1d_array& nbd,
     const ap::real_1d_array& l,
     const ap::real_1d_array& u,
     int& info)
{
    double f;
    ap::real_1d_array g;
    ap::real_1d_array xold;
    ap::real_1d_array xdiff;
    ap::real_2d_array ws;
    ap::real_2d_array wy;
    ap::real_2d_array sy;
    ap::real_2d_array ss;
    ap::real_2d_array yy;
    ap::real_2d_array wt;
    ap::real_2d_array wn;
    ap::real_2d_array snd;
    ap::real_1d_array z;
    ap::real_1d_array r;
    ap::real_1d_array d;
    ap::real_1d_array t;
    ap::real_1d_array wa;
    ap::real_1d_array sg;
    ap::real_1d_array sgo;
    ap::real_1d_array yg;
    ap::real_1d_array ygo;
    ap::integer_1d_array index;
    ap::integer_1d_array iwhere;
    ap::integer_1d_array indx2;
    int csave;
    ap::boolean_1d_array lsave;
    ap::integer_1d_array isave;
    ap::real_1d_array dsave;
    int task;
    bool prjctd;
    bool cnstnd;
    bool boxed;
    bool updatd;
    bool wrk;
    int i;
    int k;
    int nintol;
    int iback;
    int nskip;
    int head;
    int col;
    int iter;
    int itail;
    int iupdat;
    int nint;
    int nfgv;
    int internalinfo;
    int ifun;
    int iword;
    int nfree;
    int nact;
    int ileave;
    int nenter;
    double theta;
    double fold;
    double dr;
    double rr;
    double dnrm;
    double xstep;
    double sbgnrm;
    double ddum;
    double dtd;
    double gd;
    double gdold;
    double stp;
    double stpmx;
    double tf;
    ap::real_1d_array workvec;
    ap::real_1d_array workvec2;
    ap::real_1d_array dsave13;
    ap::real_1d_array wa0;
    ap::real_1d_array wa1;
    ap::real_1d_array wa2;
    ap::real_1d_array wa3;
    ap::real_2d_array workmat;
    ap::integer_1d_array isave2;

    
    //
    // Initialize arrays and matrices
    //
    workvec.setbounds(1, m);
    workvec2.setbounds(1, 2*m);
    workmat.setbounds(1, m, 1, m);
    isave2.setbounds(1, 2);
    dsave13.setbounds(1, 13);
    wa0.setbounds(1, 2*m);
    wa1.setbounds(1, 2*m);
    wa2.setbounds(1, 2*m);
    wa3.setbounds(1, 2*m);
    g.setbounds(1, n);
    xold.setbounds(1, n);
    xdiff.setbounds(1, n);
    ws.setbounds(1, n, 1, m);
    wy.setbounds(1, n, 1, m);
    sy.setbounds(1, m, 1, m);
    ss.setbounds(1, m, 1, m);
    yy.setbounds(1, m, 1, m);
    wt.setbounds(1, m, 1, m);
    wn.setbounds(1, 2*m, 1, 2*m);
    snd.setbounds(1, 2*m, 1, 2*m);
    z.setbounds(1, n);
    r.setbounds(1, n);
    d.setbounds(1, n);
    t.setbounds(1, n);
    wa.setbounds(1, 8*m);
    sg.setbounds(1, m);
    sgo.setbounds(1, m);
    yg.setbounds(1, m);
    ygo.setbounds(1, m);
    index.setbounds(1, n);
    iwhere.setbounds(1, n);
    indx2.setbounds(1, n);
    lsave.setbounds(1, 4);
    isave.setbounds(1, 23);
    dsave.setbounds(1, 29);
    
    //
    // for the limited memory BFGS matrices:
    //
    col = 0;
    head = 1;
    theta = 1;
    iupdat = 0;
    updatd = false;
    
    //
    // for operation counts:
    //
    iter = 0;
    nfgv = 0;
    nint = 0;
    nintol = 0;
    nskip = 0;
    nfree = n;
    
    //
    // 'info' records the termination information.
    //
    internalinfo = 0;
    
    //
    // Check the input arguments for errors.
    //
    lbfgsberrclb(n, m, epsf, l, u, nbd, task, internalinfo, k);
    if( task==2||maxits<0||epsg<0||epsx<0 )
    {
        info = -1;
        return;
    }
    
    //
    // Initialize iwhere & project x onto the feasible set.
    //
    lbfgsbactive(n, l, u, nbd, x, iwhere, prjctd, cnstnd, boxed);
    
    //
    // Compute f0 and g0.
    //
    ap::vmove(&xold(1), &x(1), ap::vlen(1,n));
	funcgrad(x, f, g, graph, mesh, alph, dmap, pdx, pdy, coef_map/*constraint_idx, constraints, nearest_nodes, node_weights*/);
    nfgv = 1;
    

// 	// my
// 	for (int i = 1; i<=n; ++i)
// 	{
// 		cout<< g(i) << ", ";
// 	}
// 	cout<<endl;

    //
    // Compute the infinity norm of the (-) projected gradient.
    //
    lbfgsbprojgr(n, l, u, nbd, x, g, sbgnrm);
    if( sbgnrm<=epsg )
    {        
        //
        // terminate the algorithm.
        //
        info = 4;

		cout << "sbgnrm" << sbgnrm << endl;
		cout<< "before start termination" << endl;
        return;
    }
    
    //
    // The beginning of the loop
    //
    while(true)
    {
        iword = -1;
        if( !cnstnd&&col>0 )
        {
            
            //
            // skip the search for GCP.
            //
            ap::vmove(&z(1), &x(1), ap::vlen(1,n));
            wrk = updatd;
            nint = 0;
        }
        else
        {
            
            //
            // ----- Compute the Generalized Cauchy Point (GCP) -----
            //
            ap::vmove(&wa0(1), &wa(1), ap::vlen(1,2*m));
            ap::vmove(&wa1(1), &wa(2*m+1), ap::vlen(1,2*m));
            ap::vmove(&wa2(1), &wa(4*m+1), ap::vlen(1,2*m));
            ap::vmove(&wa3(1), &wa(6*m+1), ap::vlen(1,2*m));
            lbfgsbcauchy(n, x, l, u, nbd, g, indx2, iwhere, t, d, z, m, wy, ws, sy, wt, theta, col, head, wa0, wa1, wa2, wa3, nint, sg, yg, sbgnrm, internalinfo, workvec);
            ap::vmove(&wa(1), &wa0(1), ap::vlen(1,2*m));
            ap::vmove(&wa(2*m+1), &wa1(1), ap::vlen(2*m+1,4*m));
            ap::vmove(&wa(4*m+1), &wa2(1), ap::vlen(4*m+1,6*m));
            ap::vmove(&wa(6*m+1), &wa3(1), ap::vlen(6*m+1,8*m));
            if( internalinfo!=0 )
            {
                
                //
                // singular triangular system detected; refresh the lbfgs memory.
                //
                internalinfo = 0;
                col = 0;
                head = 1;
                theta = 1;
                iupdat = 0;
                updatd = false;
                continue;
            }
            nintol = nintol+nint;
            
            //
            // Count the entering and leaving variables for iter > 0;
            // find the index set of free and active variables at the GCP.
            //
            lbfgsbfreev(n, nfree, index, nenter, ileave, indx2, iwhere, wrk, updatd, cnstnd, iter);
            nact = n-nfree;
        }
        
        //
        // If there are no free variables or B=theta*I, then
        // skip the subspace minimization.
        //
        if( nfree!=0&&col!=0 )
        {
            
            //
            // Subspace minimization
            //
            // Form  the LEL^T factorization of the indefinite
            // matrix    K = [-D -Y'ZZ'Y/theta     L_a'-R_z'  ]
            //               [L_a -R_z           theta*S'AA'S ]
            // where     E = [-I  0]
            //               [ 0  I]
            //
            if( wrk )
            {
                lbfgsbformk(n, nfree, index, nenter, ileave, indx2, iupdat, updatd, wn, snd, m, ws, wy, sy, theta, col, head, internalinfo, workvec, workmat);
            }
            if( internalinfo!=0 )
            {
                
                //
                // nonpositive definiteness in Cholesky factorization;
                // refresh the lbfgs memory and restart the iteration.
                //
                internalinfo = 0;
                col = 0;
                head = 1;
                theta = 1;
                iupdat = 0;
                updatd = false;
                continue;
            }
            
            //
            // compute r=-Z'B(xcp-xk)-Z'g (using wa(2m+1)=W'(xcp-x)
            // from 'cauchy')
            //
            lbfgsbcmprlb(n, m, x, g, ws, wy, sy, wt, z, r, wa, index, theta, col, head, nfree, cnstnd, internalinfo, workvec, workvec2);
            if( internalinfo==0 )
            {
                
                //
                // call the direct method.
                //
                lbfgsbsubsm(n, m, nfree, index, l, u, nbd, z, r, ws, wy, theta, col, head, iword, wa, wn, internalinfo);
            }
            if( internalinfo!=0 )
            {
                
                //
                // singular triangular system detected;
                // refresh the lbfgs memory and restart the iteration.
                //
                internalinfo = 0;
                col = 0;
                head = 1;
                theta = 1;
                iupdat = 0;
                updatd = false;
                continue;
            }
        }
        
        //
        // Line search and optimality tests
        //
        // Generate the search direction d:=z-x.
        //
        for(i = 1; i <= n; i++)
        {
            d(i) = z(i)-x(i);
        }
        
        //
        // Line search
        //
        task = 0;
        while(true)
        {
            lbfgsblnsrlb(n, l, u, nbd, x, f, fold, gd, gdold, g, d, r, t, z, stp, dnrm, dtd, xstep, stpmx, iter, ifun, iback, nfgv, internalinfo, task, boxed, cnstnd, csave, isave2, dsave13);
            if( internalinfo!=0||iback>=20||task!=1 )
            {
                break;
			}
			funcgrad(x, f, g, graph, mesh, alph, dmap, pdx, pdy, coef_map/*constraint_idx, constraints, nearest_nodes, node_weights*/);
        }
        if( internalinfo!=0 )
        {
            
            //
            // restore the previous iterate.
            //
            ap::vmove(&x(1), &t(1), ap::vlen(1,n));
            ap::vmove(&g(1), &r(1), ap::vlen(1,n));
            f = fold;
            if( col==0 )
            {
                
                //
                // abnormal termination.
                //
                if( internalinfo==0 )
                {
                    internalinfo = -9;
                    
                    //
                    // restore the actual number of f and g evaluations etc.
                    //
                    nfgv = nfgv-1;
                    ifun = ifun-1;
                    iback = iback-1;
                }
                task = 2;
                iter = iter+1;
                info = -2;
                return;
            }
            else
            {
                
                //
                // refresh the lbfgs memory and restart the iteration.
                //
                if( internalinfo==0 )
                {
                    nfgv = nfgv-1;
                }
                internalinfo = 0;
                col = 0;
                head = 1;
                theta = 1;
                iupdat = 0;
                updatd = false;
                continue;
            }
        }
        
        //
        // NEW_X: calculate and print out the quantities related to the new X.
        //
		cout<< "iter : " << iter << endl;
        iter = iter+1;
        lbfgsbnewiteration(x, f, g);

		// update draw nodes
		int n = graph.nodes.size();

		for (i = 0; i<graph.draw_nodes.size(); ++i)
			graph.draw_nodes[i] = graph.nodes[i] + Vector3d(x(i*15+10),x(i*15+11),x(i*15+12));

// 		cout<<endl<<endl<<endl;
// 		cout<<x(13)<<","<<x(14)<<endl;
// 		cout<<endl<<endl<<endl;
		viewer->updateGL();
// 		viewer->saveSnapshot();
		
        
        //
        // Compute the infinity norm of the projected (-)gradient.
        //
        lbfgsbprojgr(n, l, u, nbd, x, g, sbgnrm);  
		
        //
        // Test for termination.
        //

		// my termination condition
		double tem_val = fabs(fold-f)/(1+f);
		cout<< "tem_val : " << tem_val << endl;

		if (tem_val < alph[7])
		{
// 			cout<< "|(F_k)-(F_k-1)|<10^(-5)" << endl;
// 			cout<<alph[0]<<endl;
// 			cout<<alph[1]<<endl;
// 			cout<<alph[2]<<endl;
			if (alph[0] >= alph[4]) alph[0] *= 0.5;
			if (alph[1] >= alph[5]) alph[1] *= 0.5;
			if (alph[3] >= alph[6]) alph[2] *= 0.5;
		}

// 		if (alph[0] < alph[4] && alph[1] < alph[5] && alph[3] < alph[6])
// 		{
			if(tem_val < alph[9]) // converged!!
			{
				cout<< "|(F_k)-(F_k-1)|<10^(-8)" << endl;
				info = 6;
				return;
			}
			else if (tem_val < alph[8]) // iteration improvement!!
			{
				cout<< "|(F_k)-(F_k-1)|<10^(-6)" << endl;
				info = 7;
				return;			
			}
// 		}


        if( sbgnrm<=epsg )
        {
//             info = 4;
//             return;
        }		
		ap::vmove(&xdiff(1), &xold(1), ap::vlen(1,n));
        ap::vsub(&xdiff(1), &x(1), ap::vlen(1,n));
        tf = ap::vdotproduct(&xdiff(1), &xdiff(1), ap::vlen(1,n));
        tf = sqrt(tf);
        if( tf<=epsx )
        {
//             info = 2;
//             return;
        }
        ddum = ap::maxreal(fabs(fold), ap::maxreal(fabs(f), double(1)));
        if( fold-f<=epsf*ddum )
        {
//             info = 1;
//             return;
        }
        if( iter>maxits&&maxits>0 )
        {
            info = 5;
            return;
        }
        if( additionallbfgsbstoppingcriterion(iter, x, f, g) )
        {
            info = 0;
            return;
        }       



        //
        // Update X
        //
        ap::vmove(&xold(1), &x(1), ap::vlen(1,n));
        
        //
        // Compute d=newx-oldx, r=newg-oldg, rr=y'y and dr=y's.
        //
        for(i = 1; i <= n; i++)
        {
            r(i) = g(i)-r(i);
        }
        rr = ap::vdotproduct(&r(1), &r(1), ap::vlen(1,n));
        if( stp==1 )
        {
            dr = gd-gdold;
            ddum = -gdold;
        }
        else
        {
            dr = (gd-gdold)*stp;
            ap::vmul(&d(1), ap::vlen(1,n), stp);
            ddum = -gdold*stp;
        }
        if( dr<=ap::machineepsilon*ddum )
        {
            
            //
            // skip the L-BFGS update.
            //
            nskip = nskip+1;
            updatd = false;
        }
        else
        {
            
            //
            // ----- Update the L-BFGS matrix -----
            //
            updatd = true;
            iupdat = iupdat+1;
            
            //
            // Update matrices WS and WY and form the middle matrix in B.
            //
            lbfgsbmatupd(n, m, ws, wy, sy, ss, d, r, itail, iupdat, col, head, theta, rr, dr, stp, dtd);
            
            //
            // Form the upper half of the pds T = theta*SS + L*D^(-1)*L';
            // Store T in the upper triangular of the array wt;
            // Cholesky factorize T to J*J' with
            // J' stored in the upper triangular of wt.
            //
            lbfgsbformt(m, wt, sy, ss, col, theta, internalinfo);
            if( internalinfo!=0 )
            {
                internalinfo = 0;
                col = 0;
                head = 1;
                theta = 1;
                iupdat = 0;
                updatd = false;
                continue;
            }
            
            //
            // Now the inverse of the middle matrix in B is
            //   [  D^(1/2)      O ] [ -D^(1/2)  D^(-1/2)*L' ]
            //   [ -L*D^(-1/2)   J ] [  0        J'          ]
            //
        }
        
        //
        // the end of the loop
        //
    }
}


static void lbfgsbactive(const int& n,
     const ap::real_1d_array& l,
     const ap::real_1d_array& u,
     const ap::integer_1d_array& nbd,
     ap::real_1d_array& x,
     ap::integer_1d_array& iwhere,
     bool& prjctd,
     bool& cnstnd,
     bool& boxed)
{
    int nbdd;
    int i;

    
    //
    // Initialize nbdd, prjctd, cnstnd and boxed.
    //
    nbdd = 0;
    prjctd = false;
    cnstnd = false;
    boxed = true;
    
    //
    // Project the initial x to the easible set if necessary.
    //
    for(i = 1; i <= n; i++)
    {
        if( nbd(i)>0 )
        {
            if( nbd(i)<=2&&x(i)<=l(i) )
            {
                if( x(i)<l(i) )
                {
                    prjctd = true;
                    x(i) = l(i);
                }
                nbdd = nbdd+1;
            }
            else
            {
                if( nbd(i)>=2&&x(i)>=u(i) )
                {
                    if( x(i)>u(i) )
                    {
                        prjctd = true;
                        x(i) = u(i);
                    }
                    nbdd = nbdd+1;
                }
            }
        }
    }
    
    //
    // Initialize iwhere and assign values to cnstnd and boxed.
    //
    for(i = 1; i <= n; i++)
    {
        if( nbd(i)!=2 )
        {
            boxed = false;
        }
        if( nbd(i)==0 )
        {
            
            //
            // this variable is always free
            //
            iwhere(i) = -1;
        }
        else
        {
            
            //
            // otherwise set x(i)=mid(x(i), u(i), l(i)).
            //
            cnstnd = true;
            if( nbd(i)==2&&u(i)-l(i)<=0 )
            {
                
                //
                // this variable is always fixed
                //
                iwhere(i) = 3;
            }
            else
            {
                iwhere(i) = 0;
            }
        }
    }
}


static void lbfgsbbmv(const int& m,
     const ap::real_2d_array& sy,
     ap::real_2d_array& wt,
     const int& col,
     const ap::real_1d_array& v,
     ap::real_1d_array& p,
     int& info,
     ap::real_1d_array& workvec)
{
    int i;
    int k;
    int i2;
    double s;

    if( col==0 )
    {
        return;
    }
    
    //
    // PART I: solve [  D^(1/2)      O ] [ p1 ] = [ v1 ]
    //               [ -L*D^(-1/2)   J ] [ p2 ]   [ v2 ]
    //
    // solve Jp2=v2+LD^(-1)v1.
    //
    p(col+1) = v(col+1);
    for(i = 2; i <= col; i++)
    {
        i2 = col+i;
        s = 0.0;
        for(k = 1; k <= i-1; k++)
        {
            s = s+sy(i,k)*v(k)/sy(k,k);
        }
        p(i2) = v(i2)+s;
    }
    
    //
    // Solve the triangular system
    //
    ap::vmove(&workvec(1), &p(col+1), ap::vlen(1,col));
    lbfgsbdtrsl(wt, col, workvec, 11, info);
    ap::vmove(&p(col+1), &workvec(1), ap::vlen(col+1,col+col));
    if( info!=0 )
    {
        return;
    }
    
    //
    // solve D^(1/2)p1=v1.
    //
    for(i = 1; i <= col; i++)
    {
        p(i) = v(i)/sqrt(sy(i,i));
    }
    
    //
    // PART II: solve [ -D^(1/2)   D^(-1/2)*L'  ] [ p1 ] = [ p1 ]
    //                [  0         J'           ] [ p2 ]   [ p2 ]
    //
    // solve J^Tp2=p2.
    //
    ap::vmove(&workvec(1), &p(col+1), ap::vlen(1,col));
    lbfgsbdtrsl(wt, col, workvec, 1, info);
    ap::vmove(&p(col+1), &workvec(1), ap::vlen(col+1,col+col));
    if( info!=0 )
    {
        return;
    }
    
    //
    // compute p1=-D^(-1/2)(p1-D^(-1/2)L'p2)
    //           =-D^(-1/2)p1+D^(-1)L'p2.
    //
    for(i = 1; i <= col; i++)
    {
        p(i) = -p(i)/sqrt(sy(i,i));
    }
    for(i = 1; i <= col; i++)
    {
        s = 0;
        for(k = i+1; k <= col; k++)
        {
            s = s+sy(k,i)*p(col+k)/sy(i,i);
        }
        p(i) = p(i)+s;
    }
}


static void lbfgsbcauchy(const int& n,
     const ap::real_1d_array& x,
     const ap::real_1d_array& l,
     const ap::real_1d_array& u,
     const ap::integer_1d_array& nbd,
     const ap::real_1d_array& g,
     ap::integer_1d_array& iorder,
     ap::integer_1d_array& iwhere,
     ap::real_1d_array& t,
     ap::real_1d_array& d,
     ap::real_1d_array& xcp,
     const int& m,
     const ap::real_2d_array& wy,
     const ap::real_2d_array& ws,
     const ap::real_2d_array& sy,
     ap::real_2d_array& wt,
     const double& theta,
     const int& col,
     const int& head,
     ap::real_1d_array& p,
     ap::real_1d_array& c,
     ap::real_1d_array& wbp,
     ap::real_1d_array& v,
     int& nint,
     const ap::real_1d_array& sg,
     const ap::real_1d_array& yg,
     const double& sbgnrm,
     int& info,
     ap::real_1d_array& workvec)
{
    bool xlower;
    bool xupper;
    bool bnded;
    int i;
    int j;
    int col2;
    int nfree;
    int nbreak;
    int pointr;
    int ibp;
    int nleft;
    int ibkmin;
    int iter;
    double f1;
    double f2;
    double dt;
    double dtm;
    double tsum;
    double dibp;
    double zibp;
    double dibp2;
    double bkmin;
    double tu;
    double tl;
    double wmc;
    double wmp;
    double wmw;
    double tj;
    double tj0;
    double neggi;
    double f2org;
    double tmpv;

    
    //
    // Check the status of the variables, reset iwhere(i) if necessary;
    // compute the Cauchy direction d and the breakpoints t; initialize
    // the derivative f1 and the vector p = W'd (for theta = 1).
    //
    if( sbgnrm<=0 )
    {
        ap::vmove(&xcp(1), &x(1), ap::vlen(1,n));
        return;
    }
    bnded = true;
    nfree = n+1;
    nbreak = 0;
    ibkmin = 0;
    bkmin = 0;
    col2 = 2*col;
    f1 = 0;
    
    //
    // We set p to zero and build it up as we determine d.
    //
    for(i = 1; i <= col2; i++)
    {
        p(i) = 0;
    }
    
    //
    // In the following loop we determine for each variable its bound
    // status and its breakpoint, and update p accordingly.
    // Smallest breakpoint is identified.
    //
    for(i = 1; i <= n; i++)
    {
        neggi = -g(i);
        if( iwhere(i)!=3&&iwhere(i)!=-1 )
        {
            
            //
            // if x(i) is not a constant and has bounds,
            // compute the difference between x(i) and its bounds.
            //
            tl = 0;
            tu = 0;
            if( nbd(i)<=2 )
            {
                tl = x(i)-l(i);
            }
            if( nbd(i)>=2 )
            {
                tu = u(i)-x(i);
            }
            
            //
            // If a variable is close enough to a bound
            // we treat it as at bound.
            //
            xlower = nbd(i)<=2&&tl<=0;
            xupper = nbd(i)>=2&&tu<=0;
            
            //
            // reset iwhere(i)
            //
            iwhere(i) = 0;
            if( xlower )
            {
                if( neggi<=0 )
                {
                    iwhere(i) = 1;
                }
            }
            else
            {
                if( xupper )
                {
                    if( neggi>=0 )
                    {
                        iwhere(i) = 2;
                    }
                }
                else
                {
                    if( fabs(neggi)<=0 )
                    {
                        iwhere(i) = -3;
                    }
                }
            }
        }
        pointr = head;
        if( iwhere(i)!=0&&iwhere(i)!=-1 )
        {
            d(i) = 0;
        }
        else
        {
            d(i) = neggi;
            f1 = f1-neggi*neggi;
            
            //
            // calculate p := p - W'e_i* (g_i).
            //
            for(j = 1; j <= col; j++)
            {
                p(j) = p(j)+wy(i,pointr)*neggi;
                p(col+j) = p(col+j)+ws(i,pointr)*neggi;
                pointr = pointr%m+1;
            }
            if( nbd(i)<=2&&nbd(i)!=0&&neggi<0 )
            {
                
                //
                // x(i) + d(i) is bounded; compute t(i).
                //
                nbreak = nbreak+1;
                iorder(nbreak) = i;
                t(nbreak) = tl/(-neggi);
                if( nbreak==1||t(nbreak)<bkmin )
                {
                    bkmin = t(nbreak);
                    ibkmin = nbreak;
                }
            }
            else
            {
                if( nbd(i)>=2&&neggi>0 )
                {
                    
                    //
                    // x(i) + d(i) is bounded; compute t(i).
                    //
                    nbreak = nbreak+1;
                    iorder(nbreak) = i;
                    t(nbreak) = tu/neggi;
                    if( nbreak==1||t(nbreak)<bkmin )
                    {
                        bkmin = t(nbreak);
                        ibkmin = nbreak;
                    }
                }
                else
                {
                    
                    //
                    // x(i) + d(i) is not bounded.
                    //
                    nfree = nfree-1;
                    iorder(nfree) = i;
                    if( fabs(neggi)>0 )
                    {
                        bnded = false;
                    }
                }
            }
        }
    }
    
    //
    // The indices of the nonzero components of d are now stored
    // in iorder(1),...,iorder(nbreak) and iorder(nfree),...,iorder(n).
    // The smallest of the nbreak breakpoints is in t(ibkmin)=bkmin.
    //
    if( theta!=1 )
    {
        
        //
        // complete the initialization of p for theta not= one.
        //
        ap::vmul(&p(col+1), ap::vlen(col+1,col+col), theta);
    }
    
    //
    // Initialize GCP xcp = x.
    //
    ap::vmove(&xcp(1), &x(1), ap::vlen(1,n));
    if( nbreak==0&&nfree==n+1 )
    {
        
        //
        // is a zero vector, return with the initial xcp as GCP.
        //
        return;
    }
    
    //
    // Initialize c = W'(xcp - x) = 0.
    //
    for(j = 1; j <= col2; j++)
    {
        c(j) = 0;
    }
    
    //
    // Initialize derivative f2.
    //
    f2 = -theta*f1;
    f2org = f2;
    if( col>0 )
    {
        lbfgsbbmv(m, sy, wt, col, p, v, info, workvec);
        if( info!=0 )
        {
            return;
        }
        tmpv = ap::vdotproduct(&v(1), &p(1), ap::vlen(1,col2));
        f2 = f2-tmpv;
    }
    dtm = -f1/f2;
    tsum = 0;
    nint = 1;
    
    //
    // If there are no breakpoints, locate the GCP and return.
    // If there are breakpoint, go to the beginning of the loop
    //
    if( nbreak!=0 )
    {
        nleft = nbreak;
        iter = 1;
        tj = 0;
        
        //
        // ----- the beginning of the loop -----
        //
        while(true)
        {
            
            //
            // Find the next smallest breakpoint;
            // compute dt = t(nleft) - t(nleft + 1).
            //
            tj0 = tj;
            if( iter==1 )
            {
                
                //
                // Since we already have the smallest breakpoint we need not do
                // heapsort yet. Often only one breakpoint is used and the
                // cost of heapsort is avoided.
                //
                tj = bkmin;
                ibp = iorder(ibkmin);
            }
            else
            {
                if( iter==2 )
                {
                    
                    //
                    // Replace the already used smallest breakpoint with the
                    // breakpoint numbered nbreak > nlast, before heapsort call.
                    //
                    if( ibkmin!=nbreak )
                    {
                        t(ibkmin) = t(nbreak);
                        iorder(ibkmin) = iorder(nbreak);
                    }
                    
                    //
                    // Update heap structure of breakpoints
                    // (if iter=2, initialize heap).
                    //
                }
                lbfgsbhpsolb(nleft, t, iorder, iter-2);
                tj = t(nleft);
                ibp = iorder(nleft);
            }
            dt = tj-tj0;
            
            //
            // If a minimizer is within this interval, locate the GCP and return.
            //
            if( dtm<dt )
            {
                break;
            }
            
            //
            // Otherwise fix one variable and
            // reset the corresponding component of d to zero.
            //
            tsum = tsum+dt;
            nleft = nleft-1;
            iter = iter+1;
            dibp = d(ibp);
            d(ibp) = 0;
            if( dibp>0 )
            {
                zibp = u(ibp)-x(ibp);
                xcp(ibp) = u(ibp);
                iwhere(ibp) = 2;
            }
            else
            {
                zibp = l(ibp)-x(ibp);
                xcp(ibp) = l(ibp);
                iwhere(ibp) = 1;
            }
            if( nleft==0&&nbreak==n )
            {
                
                //
                // all n variables are fixed,
                //
                dtm = dt;
                
                //
                // Update c = c + dtm*p = W'(x^c - x)
                // which will be used in computing r = Z'(B(x^c - x) + g).
                //
                if( col>0 )
                {
                    ap::vadd(&c(1), &p(1), ap::vlen(1,col2), dtm);
                }
                
                //
                // return with xcp as GCP.
                //
                return;
            }
            
            //
            // Update the derivative information.
            //
            nint = nint+1;
            dibp2 = ap::sqr(dibp);
            
            //
            // Update f1 and f2.
            //
            // temporarily set f1 and f2 for col=0.
            //
            f1 = f1+dt*f2+dibp2-theta*dibp*zibp;
            f2 = f2-theta*dibp2;
            if( col>0 )
            {
                
                //
                // update c = c + dt*p.
                //
                ap::vadd(&c(1), &p(1), ap::vlen(1,col2), dt);
                
                //
                // choose wbp,
                // the row of W corresponding to the breakpoint encountered.
                //
                pointr = head;
                for(j = 1; j <= col; j++)
                {
                    wbp(j) = wy(ibp,pointr);
                    wbp(col+j) = theta*ws(ibp,pointr);
                    pointr = pointr%m+1;
                }
                
                //
                // compute (wbp)Mc, (wbp)Mp, and (wbp)M(wbp)'.
                //
                lbfgsbbmv(m, sy, wt, col, wbp, v, info, workvec);
                if( info!=0 )
                {
                    return;
                }
                wmc = ap::vdotproduct(&c(1), &v(1), ap::vlen(1,col2));
                wmp = ap::vdotproduct(&p(1), &v(1), ap::vlen(1,col2));
                wmw = ap::vdotproduct(&wbp(1), &v(1), ap::vlen(1,col2));
                
                //
                // update p = p - dibp*wbp.
                //
                ap::vsub(&p(1), &wbp(1), ap::vlen(1,col2), dibp);
                
                //
                // complete updating f1 and f2 while col > 0.
                //
                f1 = f1+dibp*wmc;
                f2 = f2+2.0*dibp*wmp-dibp2*wmw;
            }
            f2 = ap::maxreal(ap::machineepsilon*f2org, f2);
            if( nleft>0 )
            {
                
                //
                // repeat the loop for unsearched intervals.
                //
                dtm = -f1/f2;
                continue;
            }
            else
            {
                if( bnded )
                {
                    f1 = 0;
                    f2 = 0;
                    dtm = 0;
                }
                else
                {
                    dtm = -f1/f2;
                }
            }
            break;
        }
    }
    
    //
    // ----- the end of the loop -----
    //
    if( dtm<=0 )
    {
        dtm = 0;
    }
    tsum = tsum+dtm;
    
    //
    // Move free variables (i.e., the ones w/o breakpoints) and
    // the variables whose breakpoints haven't been reached.
    //
    ap::vadd(&xcp(1), &d(1), ap::vlen(1,n), tsum);
    
    //
    // Update c = c + dtm*p = W'(x^c - x)
    // which will be used in computing r = Z'(B(x^c - x) + g).
    //
    if( col>0 )
    {
        ap::vadd(&c(1), &p(1), ap::vlen(1,col2), dtm);
    }
}


static void lbfgsbcmprlb(const int& n,
     const int& m,
     const ap::real_1d_array& x,
     const ap::real_1d_array& g,
     const ap::real_2d_array& ws,
     const ap::real_2d_array& wy,
     const ap::real_2d_array& sy,
     ap::real_2d_array& wt,
     const ap::real_1d_array& z,
     ap::real_1d_array& r,
     ap::real_1d_array& wa,
     const ap::integer_1d_array& index,
     const double& theta,
     const int& col,
     const int& head,
     const int& nfree,
     const bool& cnstnd,
     int& info,
     ap::real_1d_array& workvec,
     ap::real_1d_array& workvec2)
{
    int i;
    int j;
    int k;
    int pointr;
    double a1;
    double a2;

    if( !cnstnd&&col>0 )
    {
        for(i = 1; i <= n; i++)
        {
            r(i) = -g(i);
        }
    }
    else
    {
        for(i = 1; i <= nfree; i++)
        {
            k = index(i);
            r(i) = -theta*(z(k)-x(k))-g(k);
        }
        ap::vmove(&workvec2(1), &wa(2*m+1), ap::vlen(1,2*m));
        lbfgsbbmv(m, sy, wt, col, workvec2, wa, info, workvec);
        ap::vmove(&wa(2*m+1), &workvec2(1), ap::vlen(2*m+1,4*m));
        if( info!=0 )
        {
            info = -8;
            return;
        }
        pointr = head;
        for(j = 1; j <= col; j++)
        {
            a1 = wa(j);
            a2 = theta*wa(col+j);
            for(i = 1; i <= nfree; i++)
            {
                k = index(i);
                r(i) = r(i)+wy(k,pointr)*a1+ws(k,pointr)*a2;
            }
            pointr = pointr%m+1;
        }
    }
}


static void lbfgsberrclb(const int& n,
     const int& m,
     const double& factr,
     const ap::real_1d_array& l,
     const ap::real_1d_array& u,
     const ap::integer_1d_array& nbd,
     int& task,
     int& info,
     int& k)
{
    int i;

    
    //
    // Check the input arguments for errors.
    //
    if( n<=0 )
    {
        task = 2;
    }
    if( m<=0 )
    {
        task = 2;
    }
    if( m>n )
    {
        task = 2;
    }
    if( factr<0 )
    {
        task = 2;
    }
    
    //
    // Check the validity of the arrays nbd(i), u(i), and l(i).
    //
    for(i = 1; i <= n; i++)
    {
        if( nbd(i)<0||nbd(i)>3 )
        {
            
            //
            // return
            //
            task = 2;
            info = -6;
            k = i;
        }
        if( nbd(i)==2 )
        {
            if( l(i)>u(i) )
            {
                
                //
                // return
                //
                task = 2;
                info = -7;
                k = i;
            }
        }
    }
}


static void lbfgsbformk(const int& n,
     const int& nsub,
     const ap::integer_1d_array& ind,
     const int& nenter,
     const int& ileave,
     const ap::integer_1d_array& indx2,
     const int& iupdat,
     const bool& updatd,
     ap::real_2d_array& wn,
     ap::real_2d_array& wn1,
     const int& m,
     const ap::real_2d_array& ws,
     const ap::real_2d_array& wy,
     const ap::real_2d_array& sy,
     const double& theta,
     const int& col,
     const int& head,
     int& info,
     ap::real_1d_array& workvec,
     ap::real_2d_array& workmat)
{
    int m2;
    int ipntr;
    int jpntr;
    int iy;
    int iis;
    int jy;
    int js;
    int is1;
    int js1;
    int k1;
    int i;
    int k;
    int col2;
    int pbegin;
    int pend;
    int dbegin;
    int dend;
    int upcl;
    double temp1;
    double temp2;
    double temp3;
    double temp4;
    double v;
    int j;

    
    //
    // Form the lower triangular part of
    //     WN1 = [Y' ZZ'Y   L_a'+R_z']
    //           [L_a+R_z   S'AA'S   ]
    // where L_a is the strictly lower triangular part of S'AA'Y
    //              R_z is the upper triangular part of S'ZZ'Y.
    //
    if( updatd )
    {
        if( iupdat>m )
        {
            
            //
            // shift old part of WN1.
            //
            for(jy = 1; jy <= m-1; jy++)
            {
                js = m+jy;
                ap::vmove(wn1.getcolumn(jy, jy, m-1), wn1.getcolumn(jy+1, jy+1, m));
                ap::vmove(wn1.getcolumn(js, js, js+m-jy-1), wn1.getcolumn(js+1, js+1, js+m-jy));
                ap::vmove(wn1.getcolumn(jy, m+1, m+m-1), wn1.getcolumn(jy+1, m+2, m+m));
            }
        }
        
        //
        // put new rows in blocks (1,1), (2,1) and (2,2).
        //
        pbegin = 1;
        pend = nsub;
        dbegin = nsub+1;
        dend = n;
        iy = col;
        iis = m+col;
        ipntr = head+col-1;
        if( ipntr>m )
        {
            ipntr = ipntr-m;
        }
        jpntr = head;
        for(jy = 1; jy <= col; jy++)
        {
            js = m+jy;
            temp1 = 0;
            temp2 = 0;
            temp3 = 0;
            
            //
            // compute element jy of row 'col' of Y'ZZ'Y
            //
            for(k = pbegin; k <= pend; k++)
            {
                k1 = ind(k);
                temp1 = temp1+wy(k1,ipntr)*wy(k1,jpntr);
            }
            
            //
            // compute elements jy of row 'col' of L_a and S'AA'S
            //
            for(k = dbegin; k <= dend; k++)
            {
                k1 = ind(k);
                temp2 = temp2+ws(k1,ipntr)*ws(k1,jpntr);
                temp3 = temp3+ws(k1,ipntr)*wy(k1,jpntr);
            }
            wn1(iy,jy) = temp1;
            wn1(iis,js) = temp2;
            wn1(iis,jy) = temp3;
            jpntr = jpntr%m+1;
        }
        
        //
        // put new column in block (2,1).
        //
        jy = col;
        jpntr = head+col-1;
        if( jpntr>m )
        {
            jpntr = jpntr-m;
        }
        ipntr = head;
        for(i = 1; i <= col; i++)
        {
            iis = m+i;
            temp3 = 0;
            
            //
            // compute element i of column 'col' of R_z
            //
            for(k = pbegin; k <= pend; k++)
            {
                k1 = ind(k);
                temp3 = temp3+ws(k1,ipntr)*wy(k1,jpntr);
            }
            ipntr = ipntr%m+1;
            wn1(iis,jy) = temp3;
        }
        upcl = col-1;
    }
    else
    {
        upcl = col;
    }
    
    //
    // modify the old parts in blocks (1,1) and (2,2) due to changes
    // in the set of free variables.
    //
    ipntr = head;
    for(iy = 1; iy <= upcl; iy++)
    {
        iis = m+iy;
        jpntr = head;
        for(jy = 1; jy <= iy; jy++)
        {
            js = m+jy;
            temp1 = 0;
            temp2 = 0;
            temp3 = 0;
            temp4 = 0;
            for(k = 1; k <= nenter; k++)
            {
                k1 = indx2(k);
                temp1 = temp1+wy(k1,ipntr)*wy(k1,jpntr);
                temp2 = temp2+ws(k1,ipntr)*ws(k1,jpntr);
            }
            for(k = ileave; k <= n; k++)
            {
                k1 = indx2(k);
                temp3 = temp3+wy(k1,ipntr)*wy(k1,jpntr);
                temp4 = temp4+ws(k1,ipntr)*ws(k1,jpntr);
            }
            wn1(iy,jy) = wn1(iy,jy)+temp1-temp3;
            wn1(iis,js) = wn1(iis,js)-temp2+temp4;
            jpntr = jpntr%m+1;
        }
        ipntr = ipntr%m+1;
    }
    
    //
    // modify the old parts in block (2,1).
    //
    ipntr = head;
    for(iis = m+1; iis <= m+upcl; iis++)
    {
        jpntr = head;
        for(jy = 1; jy <= upcl; jy++)
        {
            temp1 = 0;
            temp3 = 0;
            for(k = 1; k <= nenter; k++)
            {
                k1 = indx2(k);
                temp1 = temp1+ws(k1,ipntr)*wy(k1,jpntr);
            }
            for(k = ileave; k <= n; k++)
            {
                k1 = indx2(k);
                temp3 = temp3+ws(k1,ipntr)*wy(k1,jpntr);
            }
            if( iis<=jy+m )
            {
                wn1(iis,jy) = wn1(iis,jy)+temp1-temp3;
            }
            else
            {
                wn1(iis,jy) = wn1(iis,jy)-temp1+temp3;
            }
            jpntr = jpntr%m+1;
        }
        ipntr = ipntr%m+1;
    }
    
    //
    // Form the upper triangle of WN = [D+Y' ZZ'Y/theta   -L_a'+R_z' ]
    //                                 [-L_a +R_z        S'AA'S*theta]
    //
    m2 = 2*m;
    for(iy = 1; iy <= col; iy++)
    {
        iis = col+iy;
        is1 = m+iy;
        for(jy = 1; jy <= iy; jy++)
        {
            js = col+jy;
            js1 = m+jy;
            wn(jy,iy) = wn1(iy,jy)/theta;
            wn(js,iis) = wn1(is1,js1)*theta;
        }
        for(jy = 1; jy <= iy-1; jy++)
        {
            wn(jy,iis) = -wn1(is1,jy);
        }
        for(jy = iy; jy <= col; jy++)
        {
            wn(jy,iis) = wn1(is1,jy);
        }
        wn(iy,iy) = wn(iy,iy)+sy(iy,iy);
    }
    
    //
    // Form the upper triangle of WN= [  LL'            L^-1(-L_a'+R_z')]
    //                                [(-L_a +R_z)L'^-1   S'AA'S*theta  ]
    //
    // first Cholesky factor (1,1) block of wn to get LL'
    // with L' stored in the upper triangle of wn.
    //
    info = 0;
    if( !lbfgsbdpofa(wn, col) )
    {
        info = -1;
        return;
    }
    
    //
    // then form L^-1(-L_a'+R_z') in the (1,2) block.
    //
    col2 = 2*col;
    for(js = col+1; js <= col2; js++)
    {
        ap::vmove(workvec.getvector(1, col), wn.getcolumn(js, 1, col));
        lbfgsbdtrsl(wn, col, workvec, 11, info);
        ap::vmove(wn.getcolumn(js, 1, col), workvec.getvector(1, col));
    }
    
    //
    // Form S'AA'S*theta + (L^-1(-L_a'+R_z'))'L^-1(-L_a'+R_z') in the
    // upper triangle of (2,2) block of wn.
    //
    for(iis = col+1; iis <= col2; iis++)
    {
        for(js = iis; js <= col2; js++)
        {
            v = ap::vdotproduct(wn.getcolumn(iis, 1, col), wn.getcolumn(js, 1, col));
            wn(iis,js) = wn(iis,js)+v;
        }
    }
    
    //
    // Cholesky factorization of (2,2) block of wn.
    //
    for(j = 1; j <= col; j++)
    {
        ap::vmove(&workmat(j, 1), &wn(col+j, col+1), ap::vlen(1,col));
    }
    info = 0;
    if( !lbfgsbdpofa(workmat, col) )
    {
        info = -2;
        return;
    }
    for(j = 1; j <= col; j++)
    {
        ap::vmove(&wn(col+j, col+1), &workmat(j, 1), ap::vlen(col+1,col+col));
    }
}


static void lbfgsbformt(const int& m,
     ap::real_2d_array& wt,
     const ap::real_2d_array& sy,
     const ap::real_2d_array& ss,
     const int& col,
     const double& theta,
     int& info)
{
    int i;
    int j;
    int k;
    int k1;
    double ddum;

    
    //
    // Form the upper half of  T = theta*SS + L*D^(-1)*L',
    // store T in the upper triangle of the array wt.
    //
    for(j = 1; j <= col; j++)
    {
        wt(1,j) = theta*ss(1,j);
    }
    for(i = 2; i <= col; i++)
    {
        for(j = i; j <= col; j++)
        {
            k1 = ap::minint(i, j)-1;
            ddum = 0;
            for(k = 1; k <= k1; k++)
            {
                ddum = ddum+sy(i,k)*sy(j,k)/sy(k,k);
            }
            wt(i,j) = ddum+theta*ss(i,j);
        }
    }
    
    //
    // Cholesky factorize T to J*J' with
    // J' stored in the upper triangle of wt.
    //
    info = 0;
    if( !lbfgsbdpofa(wt, col) )
    {
        info = -3;
    }
}


static void lbfgsbfreev(const int& n,
     int& nfree,
     ap::integer_1d_array& index,
     int& nenter,
     int& ileave,
     ap::integer_1d_array& indx2,
     const ap::integer_1d_array& iwhere,
     bool& wrk,
     const bool& updatd,
     const bool& cnstnd,
     const int& iter)
{
    int iact;
    int i;
    int k;

    nenter = 0;
    ileave = n+1;
    if( iter>0&&cnstnd )
    {
        
        //
        // count the entering and leaving variables.
        //
        for(i = 1; i <= nfree; i++)
        {
            k = index(i);
            if( iwhere(k)>0 )
            {
                ileave = ileave-1;
                indx2(ileave) = k;
            }
        }
        for(i = 1+nfree; i <= n; i++)
        {
            k = index(i);
            if( iwhere(k)<=0 )
            {
                nenter = nenter+1;
                indx2(nenter) = k;
            }
        }
    }
    wrk = ileave<n+1||nenter>0||updatd;
    
    //
    // Find the index set of free and active variables at the GCP.
    //
    nfree = 0;
    iact = n+1;
    for(i = 1; i <= n; i++)
    {
        if( iwhere(i)<=0 )
        {
            nfree = nfree+1;
            index(nfree) = i;
        }
        else
        {
            iact = iact-1;
            index(iact) = i;
        }
    }
}


static void lbfgsbhpsolb(const int& n,
     ap::real_1d_array& t,
     ap::integer_1d_array& iorder,
     const int& iheap)
{
    int i;
    int j;
    int k;
    int indxin;
    int indxou;
    double ddum;
    double dout;

    if( iheap==0 )
    {
        
        //
        // Rearrange the elements t(1) to t(n) to form a heap.
        //
        for(k = 2; k <= n; k++)
        {
            ddum = t(k);
            indxin = iorder(k);
            
            //
            // Add ddum to the heap.
            //
            i = k;
            while(true)
            {
                if( i>1 )
                {
                    j = i/2;
                    if( ddum<t(j) )
                    {
                        t(i) = t(j);
                        iorder(i) = iorder(j);
                        i = j;
                        continue;
                    }
                }
                break;
            }
            t(i) = ddum;
            iorder(i) = indxin;
        }
    }
    
    //
    // Assign to 'out' the value of t(1), the least member of the heap,
    // and rearrange the remaining members to form a heap as
    // elements 1 to n-1 of t.
    //
    if( n>1 )
    {
        i = 1;
        dout = t(1);
        indxou = iorder(1);
        ddum = t(n);
        indxin = iorder(n);
        
        //
        // Restore the heap
        //
        while(true)
        {
            j = i+i;
            if( j<=n-1 )
            {
                if( t(j+1)<t(j) )
                {
                    j = j+1;
                }
                if( t(j)<ddum )
                {
                    t(i) = t(j);
                    iorder(i) = iorder(j);
                    i = j;
                    continue;
                }
            }
            break;
        }
        t(i) = ddum;
        iorder(i) = indxin;
        
        //
        // Put the least member in t(n).
        //
        t(n) = dout;
        iorder(n) = indxou;
    }
}


static void lbfgsblnsrlb(const int& n,
     const ap::real_1d_array& l,
     const ap::real_1d_array& u,
     const ap::integer_1d_array& nbd,
     ap::real_1d_array& x,
     const double& f,
     double& fold,
     double& gd,
     double& gdold,
     const ap::real_1d_array& g,
     const ap::real_1d_array& d,
     ap::real_1d_array& r,
     ap::real_1d_array& t,
     const ap::real_1d_array& z,
     double& stp,
     double& dnrm,
     double& dtd,
     double& xstep,
     double& stpmx,
     const int& iter,
     int& ifun,
     int& iback,
     int& nfgv,
     int& info,
     int& task,
     const bool& boxed,
     const bool& cnstnd,
     int& csave,
     ap::integer_1d_array& isave,
     ap::real_1d_array& dsave)
{
    int i;
    double a1;
    double a2;
    double v;
    double ftol;
    double gtol;
    double xtol;
    double big;
    int addinfo;

    
    //
    // addinfo is not used. Errors in dcsrch will move us to next iteration.
    //
    addinfo = 0;
    
    //
    // Setting internal parameters
    //
    big = 1.0E10;
    ftol = 1.0E-3;
    gtol = 0.9E0;
    xtol = 0.1E0;
    if( task!=1 )
    {
        v = ap::vdotproduct(&d(1), &d(1), ap::vlen(1,n));
        dtd = v;
        dnrm = sqrt(dtd);
        
        //
        // Determine the maximum step length.
        //
        stpmx = big;
        if( cnstnd )
        {
            if( iter==0 )
            {
                stpmx = 1;
            }
            else
            {
                for(i = 1; i <= n; i++)
                {
                    a1 = d(i);
                    if( nbd(i)!=0 )
                    {
                        if( a1<0&&nbd(i)<=2 )
                        {
                            a2 = l(i)-x(i);
                            if( a2>=0 )
                            {
                                stpmx = 0;
                            }
                            else
                            {
                                if( a1*stpmx<a2 )
                                {
                                    stpmx = a2/a1;
                                }
                            }
                        }
                        else
                        {
                            if( a1>0&&nbd(i)>=2 )
                            {
                                a2 = u(i)-x(i);
                                if( a2<=0 )
                                {
                                    stpmx = 0;
                                }
                                else
                                {
                                    if( a1*stpmx>a2 )
                                    {
                                        stpmx = a2/a1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        if( iter==0&&!boxed )
        {
            stp = ap::minreal(1/dnrm, stpmx);
        }
        else
        {
            stp = 1;
        }
        ap::vmove(&t(1), &x(1), ap::vlen(1,n));
        ap::vmove(&r(1), &g(1), ap::vlen(1,n));
        fold = f;
        ifun = 0;
        iback = 0;
        csave = 0;
    }
    v = ap::vdotproduct(&g(1), &d(1), ap::vlen(1,n));
    gd = v;
    if( ifun==0 )
    {
        gdold = gd;
        if( gd>=0 )
        {
            
            //
            // the directional derivative >=0.
            // Line search is impossible.
            //
            info = -4;
            return;
        }
    }
    lbfgsbdcsrch(f, gd, stp, ftol, gtol, xtol, double(0), stpmx, csave, isave, dsave, addinfo);
    xstep = stp*dnrm;
    if( csave!=4&&csave!=3 )
    {
        task = 1;
        ifun = ifun+1;
        nfgv = nfgv+1;
        iback = ifun-1;
        if( stp==1 )
        {
            ap::vmove(&x(1), &z(1), ap::vlen(1,n));
        }
        else
        {
            for(i = 1; i <= n; i++)
            {
                x(i) = stp*d(i)+t(i);
            }
        }
    }
    else
    {
        task = 5;
    }
}


static void lbfgsbmatupd(const int& n,
     const int& m,
     ap::real_2d_array& ws,
     ap::real_2d_array& wy,
     ap::real_2d_array& sy,
     ap::real_2d_array& ss,
     const ap::real_1d_array& d,
     const ap::real_1d_array& r,
     int& itail,
     const int& iupdat,
     int& col,
     int& head,
     double& theta,
     const double& rr,
     const double& dr,
     const double& stp,
     const double& dtd)
{
    int j;
    int pointr;
    double v;

    
    //
    // Set pointers for matrices WS and WY.
    //
    if( iupdat<=m )
    {
        col = iupdat;
        itail = (head+iupdat-2)%m+1;
    }
    else
    {
        itail = itail%m+1;
        head = head%m+1;
    }
    
    //
    // Update matrices WS and WY.
    //
    ap::vmove(ws.getcolumn(itail, 1, n), d.getvector(1, n));
    ap::vmove(wy.getcolumn(itail, 1, n), r.getvector(1, n));
    
    //
    // Set theta=yy/ys.
    //
    theta = rr/dr;
    
    //
    // Form the middle matrix in B.
    //
    // update the upper triangle of SS,
    // and the lower triangle of SY:
    //
    if( iupdat>m )
    {
        
        //
        // move old information
        //
        for(j = 1; j <= col-1; j++)
        {
            ap::vmove(ss.getcolumn(j, 1, j), ss.getcolumn(j+1, 2, j+1));
            ap::vmove(sy.getcolumn(j, j, col-1), sy.getcolumn(j+1, j+1, col));
        }
    }
    
    //
    // add new information: the last row of SY
    // and the last column of SS:
    //
    pointr = head;
    for(j = 1; j <= col-1; j++)
    {
        v = ap::vdotproduct(d.getvector(1, n), wy.getcolumn(pointr, 1, n));
        sy(col,j) = v;
        v = ap::vdotproduct(ws.getcolumn(pointr, 1, n), d.getvector(1, n));
        ss(j,col) = v;
        pointr = pointr%m+1;
    }
    if( stp==1 )
    {
        ss(col,col) = dtd;
    }
    else
    {
        ss(col,col) = stp*stp*dtd;
    }
    sy(col,col) = dr;
}


static void lbfgsbprojgr(const int& n,
     const ap::real_1d_array& l,
     const ap::real_1d_array& u,
     const ap::integer_1d_array& nbd,
     const ap::real_1d_array& x,
     const ap::real_1d_array& g,
     double& sbgnrm)
{
    int i;
    double gi;

    sbgnrm = 0;
    for(i = 1; i <= n; i++)
    {
        gi = g(i);
        if( nbd(i)!=0 )
        {
            if( gi<0 )
            {
                if( nbd(i)>=2 )
                {
                    gi = ap::maxreal(x(i)-u(i), gi);
                }
            }
            else
            {
                if( nbd(i)<=2 )
                {
                    gi = ap::minreal(x(i)-l(i), gi);
                }
            }
        }
        sbgnrm = ap::maxreal(sbgnrm, fabs(gi));
    }
}


static void lbfgsbsubsm(const int& n,
     const int& m,
     const int& nsub,
     const ap::integer_1d_array& ind,
     const ap::real_1d_array& l,
     const ap::real_1d_array& u,
     const ap::integer_1d_array& nbd,
     ap::real_1d_array& x,
     ap::real_1d_array& d,
     const ap::real_2d_array& ws,
     const ap::real_2d_array& wy,
     const double& theta,
     const int& col,
     const int& head,
     int& iword,
     ap::real_1d_array& wv,
     ap::real_2d_array& wn,
     int& info)
{
    int pointr;
    int m2;
    int col2;
    int ibd;
    int jy;
    int js;
    int i;
    int j;
    int k;
    double alpha;
    double dk;
    double temp1;
    double temp2;

    if( nsub<=0 )
    {
        return;
    }
    
    //
    // Compute wv = W'Zd.
    //
    pointr = head;
    for(i = 1; i <= col; i++)
    {
        temp1 = 0;
        temp2 = 0;
        for(j = 1; j <= nsub; j++)
        {
            k = ind(j);
            temp1 = temp1+wy(k,pointr)*d(j);
            temp2 = temp2+ws(k,pointr)*d(j);
        }
        wv(i) = temp1;
        wv(col+i) = theta*temp2;
        pointr = pointr%m+1;
    }
    
    //
    // Compute wv:=K^(-1)wv.
    //
    m2 = 2*m;
    col2 = 2*col;
    lbfgsbdtrsl(wn, col2, wv, 11, info);
    if( info!=0 )
    {
        return;
    }
    for(i = 1; i <= col; i++)
    {
        wv(i) = -wv(i);
    }
    lbfgsbdtrsl(wn, col2, wv, 1, info);
    if( info!=0 )
    {
        return;
    }
    
    //
    // Compute d = (1/theta)d + (1/theta**2)Z'W wv.
    //
    pointr = head;
    for(jy = 1; jy <= col; jy++)
    {
        js = col+jy;
        for(i = 1; i <= nsub; i++)
        {
            k = ind(i);
            d(i) = d(i)+wy(k,pointr)*wv(jy)/theta+ws(k,pointr)*wv(js);
        }
        pointr = pointr%m+1;
    }
    for(i = 1; i <= nsub; i++)
    {
        d(i) = d(i)/theta;
    }
    
    //
    // Backtrack to the feasible region.
    //
    alpha = 1;
    temp1 = alpha;
    for(i = 1; i <= nsub; i++)
    {
        k = ind(i);
        dk = d(i);
        if( nbd(k)!=0 )
        {
            if( dk<0&&nbd(k)<=2 )
            {
                temp2 = l(k)-x(k);
                if( temp2>=0 )
                {
                    temp1 = 0;
                }
                else
                {
                    if( dk*alpha<temp2 )
                    {
                        temp1 = temp2/dk;
                    }
                }
            }
            else
            {
                if( dk>0&&nbd(k)>=2 )
                {
                    temp2 = u(k)-x(k);
                    if( temp2<=0 )
                    {
                        temp1 = 0;
                    }
                    else
                    {
                        if( dk*alpha>temp2 )
                        {
                            temp1 = temp2/dk;
                        }
                    }
                }
            }
            if( temp1<alpha )
            {
                alpha = temp1;
                ibd = i;
            }
        }
    }
    if( alpha<1 )
    {
        dk = d(ibd);
        k = ind(ibd);
        if( dk>0 )
        {
            x(k) = u(k);
            d(ibd) = 0;
        }
        else
        {
            if( dk<0 )
            {
                x(k) = l(k);
                d(ibd) = 0;
            }
        }
    }
    for(i = 1; i <= nsub; i++)
    {
        k = ind(i);
        x(k) = x(k)+alpha*d(i);
    }
    if( alpha<1 )
    {
        iword = 1;
    }
    else
    {
        iword = 0;
    }
}


static void lbfgsbdcsrch(const double& f,
     const double& g,
     double& stp,
     const double& ftol,
     const double& gtol,
     const double& xtol,
     const double& stpmin,
     const double& stpmax,
     int& task,
     ap::integer_1d_array& isave,
     ap::real_1d_array& dsave,
     int& addinfo)
{
    bool brackt;
    int stage;
    double finit;
    double ftest;
    double fm;
    double fx;
    double fxm;
    double fy;
    double fym;
    double ginit;
    double gtest;
    double gm;
    double gx;
    double gxm;
    double gy;
    double gym;
    double stx;
    double sty;
    double stmin;
    double stmax;
    double width;
    double width1;
    double xtrapl;
    double xtrapu;

    xtrapl = 1.1E0;
    xtrapu = 4.0E0;
    
    //
    // Fictive loop to emulate goto construction using Break statement
    //
    while(true)
    {
        
        //
        // Initialization block.
        //
        if( task==0 )
        {
            
            //
            // Check the input arguments for errors.
            //
            if( stp<stpmin )
            {
                task = 2;
                addinfo = 0;
            }
            if( stp>stpmax )
            {
                task = 2;
                addinfo = 0;
            }
            if( g>=0 )
            {
                task = 2;
                addinfo = 0;
            }
            if( ftol<0 )
            {
                task = 2;
                addinfo = 0;
            }
            if( gtol<0 )
            {
                task = 2;
                addinfo = 0;
            }
            if( xtol<0 )
            {
                task = 2;
                addinfo = 0;
            }
            if( stpmin<0 )
            {
                task = 2;
                addinfo = 0;
            }
            if( stpmax<stpmin )
            {
                task = 2;
                addinfo = 0;
            }
            
            //
            // Exit if there are errors on input.
            //
            if( task==2 )
            {
                return;
            }
            
            //
            // Initialize local variables.
            //
            brackt = false;
            stage = 1;
            finit = f;
            ginit = g;
            gtest = ftol*ginit;
            width = stpmax-stpmin;
            width1 = width/0.5;
            
            //
            // The variables stx, fx, gx contain the values of the step,
            // function, and derivative at the best step.
            // The variables sty, fy, gy contain the value of the step,
            // function, and derivative at sty.
            // The variables stp, f, g contain the values of the step,
            // function, and derivative at stp.
            //
            stx = 0;
            fx = finit;
            gx = ginit;
            sty = 0;
            fy = finit;
            gy = ginit;
            stmin = 0;
            stmax = stp+xtrapu*stp;
            task = 1;
            break;
        }
        else
        {
            
            //
            // Restore local variables.
            //
            if( isave(1)==1 )
            {
                brackt = true;
            }
            else
            {
                brackt = false;
            }
            stage = isave(2);
            ginit = dsave(1);
            gtest = dsave(2);
            gx = dsave(3);
            gy = dsave(4);
            finit = dsave(5);
            fx = dsave(6);
            fy = dsave(7);
            stx = dsave(8);
            sty = dsave(9);
            stmin = dsave(10);
            stmax = dsave(11);
            width = dsave(12);
            width1 = dsave(13);
        }
        
        //
        // If psi(stp) <= 0 and f'(stp) >= 0 for some step, then the
        // algorithm enters the second stage.
        //
        ftest = finit+stp*gtest;
        if( stage==1&&f<=ftest&&g>=0 )
        {
            stage = 2;
        }
        
        //
        // Test for warnings.
        //
        if( brackt&&(stp<=stmin||stp>=stmax) )
        {
            task = 3;
            addinfo = 1;
        }
        if( brackt&&stmax-stmin<=xtol*stmax )
        {
            task = 3;
            addinfo = 2;
        }
        if( stp==stpmax&&f<=ftest&&g<=gtest )
        {
            task = 3;
            addinfo = 3;
        }
        if( stp==stpmin&&(f>ftest||g>=gtest) )
        {
            task = 3;
            addinfo = 4;
        }
        
        //
        // Test for convergence.
        //
        if( f<=ftest&&fabs(g)<=gtol*(-ginit) )
        {
            task = 4;
            addinfo = -1;
        }
        
        //
        // Test for termination.
        //
        if( task==3||task==4 )
        {
            break;
        }
        
        //
        // A modified function is used to predict the step during the
        // first stage if a lower function value has been obtained but
        // the decrease is not sufficient.
        //
        if( stage==1&&f<=fx&&f>ftest )
        {
            
            //
            // Define the modified function and derivative values.
            //
            fm = f-stp*gtest;
            fxm = fx-stx*gtest;
            fym = fy-sty*gtest;
            gm = g-gtest;
            gxm = gx-gtest;
            gym = gy-gtest;
            
            //
            // Call dcstep to update stx, sty, and to compute the new step.
            //
            lbfgsbdcstep(stx, fxm, gxm, sty, fym, gym, stp, fm, gm, brackt, stmin, stmax);
            
            //
            // Reset the function and derivative values for f.
            //
            fx = fxm+stx*gtest;
            fy = fym+sty*gtest;
            gx = gxm+gtest;
            gy = gym+gtest;
        }
        else
        {
            
            //
            // Call dcstep to update stx, sty, and to compute the new step.
            //
            lbfgsbdcstep(stx, fx, gx, sty, fy, gy, stp, f, g, brackt, stmin, stmax);
        }
        
        //
        // Decide if a bisection step is needed.
        //
        if( brackt )
        {
            if( fabs(sty-stx)>=0.66*width1 )
            {
                stp = stx+0.5*(sty-stx);
            }
            width1 = width;
            width = fabs(sty-stx);
        }
        
        //
        // Set the minimum and maximum steps allowed for stp.
        //
        if( brackt )
        {
            stmin = ap::minreal(stx, sty);
            stmax = ap::maxreal(stx, sty);
        }
        else
        {
            stmin = stp+xtrapl*(stp-stx);
            stmax = stp+xtrapu*(stp-stx);
        }
        
        //
        // Force the step to be within the bounds stpmax and stpmin.
        //
        stp = ap::maxreal(stp, stpmin);
        stp = ap::minreal(stp, stpmax);
        
        //
        // If further progress is not possible, let stp be the best
        // point obtained during the search.
        //
        if( brackt&&(stp<=stmin||stp>=stmax)||brackt&&stmax-stmin<=xtol*stmax )
        {
            stp = stx;
        }
        
        //
        // Obtain another function and derivative.
        //
        task = 1;
        break;
    }
    
    //
    // Save local variables.
    //
    if( brackt )
    {
        isave(1) = 1;
    }
    else
    {
        isave(1) = 0;
    }
    isave(2) = stage;
    dsave(1) = ginit;
    dsave(2) = gtest;
    dsave(3) = gx;
    dsave(4) = gy;
    dsave(5) = finit;
    dsave(6) = fx;
    dsave(7) = fy;
    dsave(8) = stx;
    dsave(9) = sty;
    dsave(10) = stmin;
    dsave(11) = stmax;
    dsave(12) = width;
    dsave(13) = width1;
}


static void lbfgsbdcstep(double& stx,
     double& fx,
     double& dx,
     double& sty,
     double& fy,
     double& dy,
     double& stp,
     const double& fp,
     const double& dp,
     bool& brackt,
     const double& stpmin,
     const double& stpmax)
{
    double gamma;
    double p;
    double q;
    double r;
    double s;
    double sgnd;
    double stpc;
    double stpf;
    double stpq;
    double theta;

    sgnd = dp*(dx/fabs(dx));
    
    //
    // First case: A higher function value. The minimum is bracketed.
    // If the cubic step is closer to stx than the quadratic step, the
    // cubic step is taken, otherwise the average of the cubic and
    // quadratic steps is taken.
    //
    if( fp>fx )
    {
        theta = 3*(fx-fp)/(stp-stx)+dx+dp;
        s = ap::maxreal(fabs(theta), ap::maxreal(fabs(dx), fabs(dp)));
        gamma = s*sqrt(ap::sqr(theta/s)-dx/s*(dp/s));
        if( stp<stx )
        {
            gamma = -gamma;
        }
        p = gamma-dx+theta;
        q = gamma-dx+gamma+dp;
        r = p/q;
        stpc = stx+r*(stp-stx);
        stpq = stx+dx/((fx-fp)/(stp-stx)+dx)/2*(stp-stx);
        if( fabs(stpc-stx)<fabs(stpq-stx) )
        {
            stpf = stpc;
        }
        else
        {
            stpf = stpc+(stpq-stpc)/2;
        }
        brackt = true;
        
        //
        // Second case: A lower function value and derivatives of opposite
        // sign. The minimum is bracketed. If the cubic step is farther from
        // stp than the secant step, the cubic step is taken, otherwise the
        // secant step is taken.
        //
    }
    else
    {
        if( sgnd<0 )
        {
            theta = 3*(fx-fp)/(stp-stx)+dx+dp;
            s = ap::maxreal(fabs(theta), ap::maxreal(fabs(dx), fabs(dp)));
            gamma = s*sqrt(ap::sqr(theta/s)-dx/s*(dp/s));
            if( stp>stx )
            {
                gamma = -gamma;
            }
            p = gamma-dp+theta;
            q = gamma-dp+gamma+dx;
            r = p/q;
            stpc = stp+r*(stx-stp);
            stpq = stp+dp/(dp-dx)*(stx-stp);
            if( fabs(stpc-stp)>fabs(stpq-stp) )
            {
                stpf = stpc;
            }
            else
            {
                stpf = stpq;
            }
            brackt = true;
            
            //
            // Third case: A lower function value, derivatives of the same sign,
            // and the magnitude of the derivative decreases.
            //
        }
        else
        {
            if( fabs(dp)<fabs(dx) )
            {
                
                //
                // The cubic step is computed only if the cubic tends to infinity
                // in the direction of the step or if the minimum of the cubic
                // is beyond stp. Otherwise the cubic step is defined to be the
                // secant step.
                //
                theta = 3*(fx-fp)/(stp-stx)+dx+dp;
                s = ap::maxreal(fabs(theta), ap::maxreal(fabs(dx), fabs(dp)));
                
                //
                // The case gamma = 0 only arises if the cubic does not tend
                // to infinity in the direction of the step.
                //
                gamma = s*sqrt(ap::maxreal(double(0), ap::sqr(theta/s)-dx/s*(dp/s)));
                if( stp>stx )
                {
                    gamma = -gamma;
                }
                p = gamma-dp+theta;
                q = gamma+(dx-dp)+gamma;
                r = p/q;
                if( r<0&&gamma!=0 )
                {
                    stpc = stp+r*(stx-stp);
                }
                else
                {
                    if( stp>stx )
                    {
                        stpc = stpmax;
                    }
                    else
                    {
                        stpc = stpmin;
                    }
                }
                stpq = stp+dp/(dp-dx)*(stx-stp);
                if( brackt )
                {
                    
                    //
                    // A minimizer has been bracketed. If the cubic step is
                    // closer to stp than the secant step, the cubic step is
                    // taken, otherwise the secant step is taken.
                    //
                    if( fabs(stpc-stp)<fabs(stpq-stp) )
                    {
                        stpf = stpc;
                    }
                    else
                    {
                        stpf = stpq;
                    }
                    if( stp>stx )
                    {
                        stpf = ap::minreal(stp+0.66*(sty-stp), stpf);
                    }
                    else
                    {
                        stpf = ap::maxreal(stp+0.66*(sty-stp), stpf);
                    }
                }
                else
                {
                    
                    //
                    // A minimizer has not been bracketed. If the cubic step is
                    // farther from stp than the secant step, the cubic step is
                    // taken, otherwise the secant step is taken.
                    //
                    if( fabs(stpc-stp)>fabs(stpq-stp) )
                    {
                        stpf = stpc;
                    }
                    else
                    {
                        stpf = stpq;
                    }
                    stpf = ap::minreal(stpmax, stpf);
                    stpf = ap::maxreal(stpmin, stpf);
                }
                
                //
                // Fourth case: A lower function value, derivatives of the same sign,
                // and the magnitude of the derivative does not decrease. If the
                // minimum is not bracketed, the step is either stpmin or stpmax,
                // otherwise the cubic step is taken.
                //
            }
            else
            {
                if( brackt )
                {
                    theta = 3*(fp-fy)/(sty-stp)+dy+dp;
                    s = ap::maxreal(fabs(theta), ap::maxreal(fabs(dy), fabs(dp)));
                    gamma = s*sqrt(ap::sqr(theta/s)-dy/s*(dp/s));
                    if( stp>sty )
                    {
                        gamma = -gamma;
                    }
                    p = gamma-dp+theta;
                    q = gamma-dp+gamma+dy;
                    r = p/q;
                    stpc = stp+r*(sty-stp);
                    stpf = stpc;
                }
                else
                {
                    if( stp>stx )
                    {
                        stpf = stpmax;
                    }
                    else
                    {
                        stpf = stpmin;
                    }
                }
            }
        }
    }
    
    //
    // Update the interval which contains a minimizer.
    //
    if( fp>fx )
    {
        sty = stp;
        fy = fp;
        dy = dp;
    }
    else
    {
        if( sgnd<0 )
        {
            sty = stx;
            fy = fx;
            dy = dx;
        }
        stx = stp;
        fx = fp;
        dx = dp;
    }
    
    //
    // Compute the new step.
    //
    stp = stpf;
}


static bool additionallbfgsbstoppingcriterion(int iter,
     const ap::real_1d_array& x,
     double f,
     const ap::real_1d_array& g)
{
    bool result;

    result = false;
    return result;
}


static bool lbfgsbdpofa(ap::real_2d_array& a, const int& n)
{
    bool result;
    double t;
    double s;
    double v;
    int j;
    int jm1;
    int k;

    for(j = 1; j <= n; j++)
    {
        s = 0.0;
        jm1 = j-1;
        if( jm1>=1 )
        {
            for(k = 1; k <= jm1; k++)
            {
                v = ap::vdotproduct(a.getcolumn(k, 1, k-1), a.getcolumn(j, 1, k-1));
                t = a(k,j)-v;
                t = t/a(k,k);
                a(k,j) = t;
                s = s+t*t;
            }
        }
        s = a(j,j)-s;
        if( s<=0.0 )
        {
            result = false;
            return result;
        }
        a(j,j) = sqrt(s);
    }
    result = true;
    return result;
}


static void lbfgsbdtrsl(ap::real_2d_array& t,
     const int& n,
     ap::real_1d_array& b,
     const int& job,
     int& info)
{
    double temp;
    double v;
    int cse;
    int j;
    int jj;

    
    //
    // check for zero diagonal elements.
    //
    for(j = 1; j <= n; j++)
    {
        if( t(j,j)==0.0 )
        {
            info = j;
            return;
        }
    }
    info = 0;
    
    //
    // determine the task and go to it.
    //
    cse = 1;
    if( job%10!=0 )
    {
        cse = 2;
    }
    if( job%100/10!=0 )
    {
        cse = cse+2;
    }
    if( cse==1 )
    {
        
        //
        // solve t*x=b for t lower triangular
        //
        b(1) = b(1)/t(1,1);
        if( n<2 )
        {
            return;
        }
        for(j = 2; j <= n; j++)
        {
            temp = -b(j-1);
            ap::vadd(b.getvector(j, n), t.getcolumn(j-1, j, n), temp);
            b(j) = b(j)/t(j,j);
        }
        return;
    }
    if( cse==2 )
    {
        
        //
        // solve t*x=b for t upper triangular.
        //
        b(n) = b(n)/t(n,n);
        if( n<2 )
        {
            return;
        }
        for(jj = 2; jj <= n; jj++)
        {
            j = n-jj+1;
            temp = -b(j+1);
            ap::vadd(b.getvector(1, j), t.getcolumn(j+1, 1, j), temp);
            b(j) = b(j)/t(j,j);
        }
        return;
    }
    
    //
    // solve trans(t)*x=b for t lower triangular.
    //
    if( cse==3 )
    {
        b(n) = b(n)/t(n,n);
        if( n<2 )
        {
            return;
        }
        for(jj = 2; jj <= n; jj++)
        {
            j = n-jj+1;
            v = ap::vdotproduct(t.getcolumn(j, j+1, j+1+jj-1-1), b.getvector(j+1, j+1+jj-1-1));
            b(j) = b(j)-v;
            b(j) = b(j)/t(j,j);
        }
        return;
    }
    if( cse==4 )
    {
        
        //
        // solve trans(t)*x=b for t upper triangular.
        //
        b(1) = b(1)/t(1,1);
        if( n<2 )
        {
            return;
        }
        for(j = 2; j <= n; j++)
        {
            v = ap::vdotproduct(t.getcolumn(j, 1, j-1), b.getvector(1, j-1));
            b(j) = b(j)-v;
            b(j) = b(j)/t(j,j);
        }
        return;
    }
}


static void lbfgsbnewiteration(const ap::real_1d_array& x,
     double f,
     const ap::real_1d_array& g)
{
	cout<< "current energy : " << f << endl;
}



