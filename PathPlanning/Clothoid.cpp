#include "Clothoid.h"
#include "BaseAngle.h"

double fns[] = { 0.49999988085884732562,
1.3511177791210715095,
1.3175407836168659241,
1.1861149300293854992,
0.7709627298888346769,
0.4173874338787963957,
0.19044202705272903923,
0.06655998896627697537,
0.022789258616785717418,
0.0040116689358507943804,
0.0012192036851249883877 };

double fds[] = { 1.0,
2.7022305772400260215,
4.2059268151438492767,
4.5221882840107715516,
3.7240352281630359588,
2.4589286254678152943,
1.3125491629443702962,
0.5997685720120932908,
0.20907680750378849485,
0.07159621634657901433,
0.012602969513793714191,
0.0038302423512931250065 };

double gns[] = { 0.50000014392706344801,
0.032346434925349128728,
0.17619325157863254363,
0.038606273170706486252,
0.023693692309257725361,
0.007092018516845033662,
0.0012492123212412087428,
0.00044023040894778468486,
-8.80266827476172521e-6,
-1.4033554916580018648e-8,
2.3509221782155474353e-10 };

double gds[] = { 1.0,
2.0646987497019598937,
2.9109311766948031235,
2.6561936751333032911,
2.0195563983177268073,
1.1167891129189363902,
0.57267874755973172715,
0.19408481169593070798,
0.07634808341431248904,
0.011573247407207865977,
0.0044099273693067311209,
-0.00009070958410429993314 };

void Clothoid::FresnelCS(double *FresnelC, double *FresnelS, double y)
{
	double t, twofn, fact, denterm, numterm, sum, ratio, term, sumn, sumd, f, g, U, SinU, CosU, oldterm, eps10, absterm;

	double x = abs(y);
	if (x < 1.0)
	{
		t = -((PI / 2)*x*x)*((PI / 2)*x*x);
		//	% Cosine integral series
		twofn = 0.0;
		fact = 1.0;
		denterm = 1.0;
		numterm = 1.0;
		sum = 1.0;
		ratio = 10.0;

		while (ratio > eps1)
		{
			twofn = twofn + 2.0;
			fact = fact*twofn*(twofn - 1.0);
			denterm = denterm + 4.0;
			numterm = numterm*t;
			term = numterm / (fact*denterm);
			sum = sum + term;
			ratio = abs(term / sum);
		}
		FresnelC[0] = x*sum;

		//% Sine integral series
		twofn = 1.0;
		fact = 1.0;
		denterm = 3.0;
		numterm = 1.0;
		sum = 1.0 / 3.0;
		ratio = 10.0;

		while (ratio > eps1)
		{
			twofn = twofn + 2.0;
			fact = fact*twofn*(twofn - 1.0);
			denterm = denterm + 4.0;
			numterm = numterm*t;
			term = numterm / (fact*denterm);
			sum = sum + term;
			ratio = abs(term / sum);
		}
		FresnelS[0] = (PI / 2)*sum*x*x*x;
	}
	else if (x < 6.0)		//	% Rational approximation for f
	{
		sumn = 0.0;
		sumd = fds[11];
		for (int k = 11; k >= 1; k--)
		{
			sumn = fns[k - 1] + x*sumn;
			sumd = fds[k - 1] + x*sumd;
		}

		f = sumn / sumd;
		//% Rational approximation for  g
		sumn = 0.0;
		sumd = gds[11];
		for (int k = 11; k >= 1; k--)
		{
			sumn = gns[k - 1] + x*sumn;
			sumd = gds[k - 1] + x*sumd;
		}

		g = sumn / sumd;
		U = (PI / 2)*x*x;
		SinU = sin(U);
		CosU = cos(U);
		FresnelC[0] = 0.5 + f*SinU - g*CosU;
		FresnelS[0] = 0.5 - f*CosU - g*SinU;
	}
	else
	{
		//	% x >= 6; asymptotic expansions for  f  and  g
		t = -1 / ((PI*x*x)*(PI*x*x));
		//	% Expansion for  f
		numterm = -1.0;
		term = 1.0;
		sum = 1.0;
		oldterm = 1.0;
		ratio = 10.0;
		eps10 = 0.1*eps1;

		while (ratio > eps10)
		{
			numterm = numterm + 4.0;
			term = term*numterm*(numterm - 2.0)*t;
			sum = sum + term;
			absterm = abs(term);
			ratio = abs(term / sum);
			if (oldterm < absterm)
			{
				//disp('\n\n !!In FresnelCS f not converged to eps');
				ratio = eps10;
			}

			oldterm = absterm;
		}
		f = sum / (PI*x);
		//% Expansion for  g
		numterm = -1.0;
		term = 1.0;
		sum = 1.0;
		oldterm = 1.0;
		ratio = 10.0;
		eps10 = 0.1*eps1;

		while (ratio > eps10)
		{
			numterm = numterm + 4.0;
			term = term*numterm*(numterm + 2.0)*t;
			sum = sum + term;
			absterm = abs(term);
			ratio = abs(term / sum);
			if (oldterm < absterm)
			{
				//disp('\n\n!!In FresnelCS g not converged to eps');
				ratio = eps10;
			}
			oldterm = absterm;

		}
		g = sum / ((PI*x)*(PI*x)*x);
		U = (PI / 2)*x*x;
		SinU = sin(U);
		CosU = cos(U);
		FresnelC[0] = 0.5 + f*SinU - g*CosU;
		FresnelS[0] = 0.5 - f*CosU - g*SinU;
	}
	if (y < 0)
	{
		FresnelC[0] = -FresnelC[0];
		FresnelS[0] = -FresnelS[0];
	}
}
void Clothoid::intXY(double *X, double *Y, int nk, double a, double b, double c)
{
	double absa = abs(a);
	double epsi = 0.01;// % best thresold

	if (absa < epsi)//% case `a` small
	{
		evalXYaSmall(X, Y, nk, a, b, 5);
	}
	else
	{
		evalXYaLarge(X, Y, nk, a, b);
	}


	double cc = cos(c);
	double ss = sin(c);

	for (int k = 0; k < nk; k++)
	{
		double xx = X[k];
		double yy = Y[k];
		X[k] = xx*cc - yy*ss;
		Y[k] = xx*ss + yy*cc;
	}

}
void Clothoid::FresnelCSk(int nk, double t, double *FresnelC, double*FresnelS)
{
	double C[3];
	double S[3];
	FresnelCS(C, S, t);
	FresnelC[0] = C[0];
	FresnelS[0] = S[0];
	if (nk > 1)
	{
		double tt = PI / 2 * t*t;
		double ss = sin(tt);
		double cc = cos(tt);
		FresnelC[1] = ss / PI;
		FresnelS[1] = (1 - cc) / PI;
		if (nk > 2)
		{
			FresnelC[2] = (t*ss - FresnelS[0]) / PI;
			FresnelS[2] = (FresnelC[0] - t*cc) / PI;
		}
	}

}

void Clothoid::evalXYaLarge(double* X, double* Y, int nk, double a, double b)
{
	double	s = 0;
	if (a > 0)s = 1;
	else if (a == 0)s = 0;
	else if (a < 0)s = -1;
	//else s=0;
	double	z = sqrt(abs(a) / PI);
	double	ell = s*b / sqrt(abs(a)*PI);
	double	g = -0.5*s*b*b / abs(a);
	double Cl[3];
	double Sl[3];
	double Cz[3];
	double Sz[3];

	FresnelCSk(nk, ell, Cl, Sl);
	FresnelCSk(nk, ell + z, Cz, Sz);
	double* dC = new double[nk]; dC[0] = Cz[0] - Cl[0];
	double* dS = new double[nk]; dS[0] = Sz[0] - Sl[0];
	double cg = cos(g) / z;
	double sg = sin(g) / z;
	X[0] = cg * dC[0] - s * sg * dS[0];
	Y[0] = sg * dC[0] + s * cg * dS[0];
	if (nk > 1)
	{
		dC[1] = Cz[1] - Cl[1];
		dC[2] = Cz[2] - Cl[2];
		dS[1] = Sz[1] - Sl[1];
		dS[2] = Sz[2] - Sl[2];
		cg = cg / z;
		sg = sg / z;
		double DC = dC[1] - ell*dC[0];
		double DS = dS[1] - ell*dS[0];
		X[1] = cg * DC - s * sg * DS;
		Y[1] = sg * DC + s * cg * DS;
		if (nk > 2)
		{
			cg = cg / z;
			sg = sg / z;
			DC = dC[2] + ell*(ell*dC[0] - 2 * dC[1]);
			DS = dS[2] + ell*(ell*dS[0] - 2 * dS[1]);
			X[2] = cg * DC - s * sg * DS;
			Y[2] = sg * DC + s * cg * DS;
		}
	}
	delete dC;
	delete dS;
}
double Clothoid::S(double mu, double nu, double b)
{

	double tmp = 1 / ((mu + nu + 1)*(mu - nu + 1));
	double res = tmp;
	for (int n = 1; n <= 100; n++)
	{
		tmp = tmp * (-b / (2 * n + mu - nu + 1)) * (b / (2 * n + mu + nu + 1));
		res = res + tmp;
		if (abs(tmp) < abs(res) *1e-50)
			break;

	}
	return res;
}
void Clothoid::evalXYazero(double *X, double *Y, int nk, double b)
{
	double sb = sin(b);
	double cb = cos(b);
	double b2 = b*b;
	if (abs(b) < 1e-3)
	{
		X[0] = 1 - b2 / 6 * (1 - b2 / 20 * (1 - b2 / 42));
		Y[0] = (b / 2)*(1 - b2 / 12 * (1 - b2 / 30));
	}
	else
	{
		X[0] = sb / b;
		Y[0] = (1 - cb) / b;
	}
	double A = b*sb;
	double D = sb - b*cb;
	double B = b*D;
	double C = -b2*sb;
	for (int k = 1; k < nk; k++)
	{
		X[k] = (k*A*S(k + 0.5, 1.5, b) + B*S(k + 1.5, 0.5, b) + cb) / (1 + k);
		Y[k] = (C*S(k + 1.5, 1.5, b) + sb) / (2 + k) + D*S(k + 0.5, 0.5, b);
	}
}
void Clothoid::evalXYaSmall(double *X, double *Y, int nk, double a, double b, double p)
{

	double* X0 = new double[nk + 4 * (int)p + 2];
	double* Y0 = new double[nk + 4 * (int)p + 2];
	memset(X0, 0, (nk + 4 * (int)p + 2)*sizeof(double));
	memset(Y0, 0, (nk + 4 * (int)p + 2)*sizeof(double));
	evalXYazero(X0, Y0, nk + 4 * p + 2, b);

	for (int j = 0; j < nk; j++)
	{
		X[j] = X0[j] - a*Y0[j + 2] / 2;
		Y[j] = Y0[j] + a*X0[j + 2] / 2;
	}


	int 	t = 1;
	double aa = -(a / 4)*(a / 4);
	for (int n = 1; n <= p; n++)
	{
		t = t*aa / (n*(2 * n - 1));
		double bf = a / (4 * n + 2);
		for (int j = 0; j < nk; j++)
		{
			X[j] = X[j] + t*(X0[4 * n + j] - bf*Y0[4 * n + j + 2]);
			Y[j] = Y[j] + t*(Y0[4 * n + j] + bf*X0[4 * n + j + 2]);

		}
	}

	delete X0;
	delete Y0;
}

Clothoid::Clothoid(const double& x0, const double &y0, const double& theta0, const double& x1, const double &y1, const double& theta1):
_x0(x0), _y0(y0), _theta0(theta0), _x1(x1), _y1(y1), _theta1(theta1), _init(true)
{
	BuildClothoid();
}

void Clothoid::SetPoint(const double& x0, const double &y0, const double& theta0, const double& x1, const double &y1, const double& theta1)
{
	_x0 = x0;
	_y0 = y0;
	_theta0 = theta0;
	_x1 = x1;
	_y1 = y1;
	_theta1 = theta1;

	_init = true;
}

void Clothoid::BuildClothoid(double tol /*=0.05*/)
{
	double dx = _x1 - _x0;
	double dy = _y1 - _y0;
	double r = sqrt(dx*dx + dy*dy);
	double phi = atan2(dy, dx);
	double phi0 = RadAngle::Normalize(_theta0 - phi);
	double phi1 = RadAngle::Normalize(_theta1 - phi);

	double delta = phi1 - phi0;

	// % initial point
	double	 Aguess = guessA(phi0, phi1);
	double A;
	// % Newton iteration
	findA(A, Aguess, delta, phi0, tol);

	double* h = new double;
	double* g = new double;
	memset(h, 0, sizeof(double));
	memset(g, 0, sizeof(double));
	//% final operation
	intXY(h, g, 1, 2 * A, delta - A, phi0);
	_L = r / h[0];

	if (_L > 0)
	{
		_k = (delta - A) / _L;
		_dk = 2 * A / _L / _L;
	}
	else
	{
		_k = infcon;
		_dk = infcon;
		_L = infcon;
	}

	delete h;
	delete g;
}

void Clothoid::findA(double &A, double Aguess, double delta, double phi0, double tol)
{
	double f, df;
	double intC[3];
	double intS[3];

	A = Aguess;
	for (int iter = 1; iter <= 100; iter++)
	{
		intXY(intC, intS, 3, 2 * A, delta - A, phi0);
		f = intS[0];
		df = intC[2] - intC[1];
		A = A - f / df;
		if (abs(f) < tol)
			break;
	}
}


double Clothoid::guessA(double phi0, double phi1)
{
	double CF[6] = { 2.989696028701907,
		0.716228953608281,
		-0.458969738821509,
		-0.502821153340377,
		0.261062141752652,
		-0.045854475238709 };
	double X = phi0 / PI;
	double 	Y = phi1 / PI;
	double	xy = X*Y;
	double A = (phi0 + phi1) * (CF[0] + xy * (CF[1] + xy * CF[2]) +
		(CF[3] + xy * CF[4]) * (X*X + Y*Y) +
		CF[5] * (X*X*X*X + Y*Y*Y*Y));
	return A;
}

//XY为保存Clothoid上点的数组，npts为点的个数
void Clothoid::PointsOnClothoid(RoadPoint XY[], int npts)
{
	double C[1];
	double S[1];
	int j = 0;

	for (double t = 0; t <= _L;)
	{
		intXY(C, S, 1, _dk * t * t, _k * t, _theta0);

		XY[j].x = _x0 + t * C[0];
		XY[j].y = _y0 + t * S[0];
		XY[j].angle = _theta0 + t * _k + 0.5 * t * t * _dk;
		t = t + _L / (double)npts;
		j++;
	}
}