#ifndef GAUSSPROJECTION_H
#define GAUSSPROJECTION_H

#include <cmath>

const double RHO = 206265;
const double PI = 3.1415926535897932384626;

class unitSwitch
{
    public:
    static double degreeToRad(int deg, int min, double sec);
    static double degreeToRad(double deg);
    static double degreeToSec(int deg,int min, double sec);
    static double degreeToSec(double deg);
    static double secToRad(double sec);

    static double degreeToDeg(int deg, int min, double sec);
};

class LBtoxy
{
public:
    LBtoxy();
    LBtoxy(double L, double B);
    LBtoxy(double L, double B, int distinguish);

    void setOrder();
    void setL0();
    void setl();
    void setN();
    void seta0();
    void seta4();
    void seta6();
    void seta3();
    void seta5();
    void setCosBandSinB();

    void setGaussCoordinate();

    void calculateAll();

    int getOrder() const;
    double getl() const;

    double getx() const;
    double gety() const;

private:
    double myL,
           myB,
           myx,
           myy;

    double myl,
           myN,
           mya0,
           mya4,
           mya6,
           mya3,
           mya5,
           myCosB,
           myCosBSquare,
           mySinB,
           mySinBSquare;

    int myOrder,
        myL0,
        myDistinguish;
};


class xyToLB
{
public:
    xyToLB();
    xyToLB(double x, double y, int order);
    xyToLB(double x, double y, int order, int distinguish);

    void setBeta();
    void setCosAndSinBeta();
    void setBf();
    void setCosAndSinBf();
    void setNf();
    void setZ();
    void setb2();
    void setb3();
    void setb4();
    void setb5();

    void setB();
    void setl();
    void setL();
    void setAll();

    double getL() const;
    double getB() const;

private:
    double myx,
           myy,
           myl,
           myB,
           myL0,
           myL;
    int myOrder,
        myDistinguish;

    double myBeta,
           myBf,
           myCosBf,
           myCosBfSquare,
           mySinBf,
           myCosBeta,
           myCosBetaSquare,
           mySinBeta,
           myNf,
           myZ,
           myb2,
           myb3,
           myb4,
           myb5;

};

inline double unitSwitch::degreeToRad(int deg, int min, double sec)
{
    return PI /180 * ((double)deg + (double)min /60 + sec /3600);
}

inline double unitSwitch::degreeToSec(int deg, int min, double sec)
{
    return deg * 3600 + min * 60 + sec;
}

inline double unitSwitch::degreeToSec(double deg)
{
    return deg * 3600;
}

inline double unitSwitch::degreeToRad(double deg)
{
    return PI /180 * deg;
}

inline double unitSwitch::degreeToDeg(int deg, int min, double sec)
{
    return (double)deg + (double)min / 60 + sec /3600;
}

inline double unitSwitch::secToRad(double sec)
{
    return sec / 648000 * PI;
}



inline LBtoxy::LBtoxy()
{
    myL = myB = 0;
}

inline LBtoxy::LBtoxy(double L, double B)
{
    myL = L;
    myB = B;
    myDistinguish = 6;
}

inline LBtoxy::LBtoxy(double L, double B, int distinguish)
{
    myL = L;
    myB = B;
    myDistinguish = distinguish;
}

inline void LBtoxy::setOrder()
{
    if (myDistinguish == 6)
        myOrder = int(myL / 6) +1;
    else
        myOrder = int(myL /3 +0.5);
}


inline int LBtoxy::getOrder() const
{
    return myOrder;
}

inline double LBtoxy::getl() const
{
    return myl;
}

inline double LBtoxy::getx() const
{
    return myx;
}

inline double LBtoxy::gety() const
{
    return myy;
}

inline xyToLB::xyToLB()
{
    myx = myy = 0;
    myOrder = 1;
}

inline xyToLB::xyToLB(double x, double y, int order)
{
    myx = x;
    myy = y;
    myOrder = order;
    myDistinguish = 6;
}

inline xyToLB::xyToLB(double x, double y, int order, int distinguish)
{
    myx = x;
    myy = y;
    myOrder = order;
    myDistinguish = distinguish;
}

inline double xyToLB::getL() const
{
    return myL;
}

inline double xyToLB::getB() const
{
    return myB;
}
#endif // GAUSSPROJECTION_H
