#include "GaussProjection.h"


void LBtoxy::setL0()
{
    if (myDistinguish == 6)
        myL0 = 6* myOrder - 3;
    else
        myL0 = 3* myOrder;
}

void LBtoxy::setCosBandSinB()
{
    myCosB = cos(unitSwitch::degreeToRad(myB));
    mySinB = sin(unitSwitch::degreeToRad(myB));

    myCosBSquare = myCosB * myCosB;
    mySinBSquare = mySinB * mySinB;
}

void LBtoxy::setl()
{
    myl = unitSwitch::degreeToSec(myL - myL0) / RHO;
}

void LBtoxy::setN()
{
    myN = 6399698.902 - (21562.267 - (108.973 - 0.612 * myCosBSquare) * myCosBSquare)* myCosBSquare;
}

void LBtoxy::seta0()
{
    mya0 = 32140.404 - (135.3302 - (0.7092 - 0.0040 * myCosBSquare)* myCosBSquare)* myCosBSquare;
}

void LBtoxy::seta4()
{
    mya4 = (0.25 + 0.00252 * myCosBSquare) * myCosBSquare - 0.04166;
}

void LBtoxy::seta6()
{
    mya6  = (0.166 * myCosBSquare - 0.084) * myCosBSquare;
}

void LBtoxy::seta3()
{
    mya3 = (0.3333333 + 0.001123 * myCosBSquare)* myCosBSquare - 0.1666667;
}

void LBtoxy::seta5()
{
    mya5 = 0.0083 - (0.1667 -(0.1968 +0.0040 * myCosBSquare)* myCosBSquare) * myCosBSquare;
}


void LBtoxy::setGaussCoordinate()
{
    myx = 6367558.4969 * unitSwitch::degreeToSec(myB) / RHO - (mya0 - (0.5 + (mya4 + mya6 * myl * myl) * myl * myl)
        * myl * myl * myN)* mySinB * myCosB;
    myy = (1 + (mya3 + mya5 * myl *myl)* myl *myl)* myl * myN * myCosB;
}

void LBtoxy::calculateAll()
{
    setOrder();
    setL0();
    setCosBandSinB();
    setl();
    setN();
    seta0();
    seta4();
    seta6();
    seta3();
    seta5();
    setGaussCoordinate();
}


void xyToLB::setBeta()
{
    myBeta = myx / 6367558.4969 * RHO;
}

void xyToLB::setCosAndSinBeta()
{
    myCosBeta = cos(unitSwitch::secToRad(myBeta));
    myCosBetaSquare = myCosBeta * myCosBeta;
    mySinBeta = sin(unitSwitch::secToRad(myBeta));
}

void xyToLB::setBf()
{
     myBf = myBeta + (50221746 + (293622 + (2350 + 22 * myCosBetaSquare)* myCosBetaSquare)* myCosBetaSquare) * 1e-10 * mySinBeta * myCosBeta * RHO;
}

void xyToLB::setCosAndSinBf()
{
    myCosBf = cos(unitSwitch::secToRad(myBf));
    myCosBfSquare = myCosBf * myCosBf;
    mySinBf = sin(unitSwitch::secToRad(myBf));
}

void xyToLB::setNf()
{
    myNf = 6399698.902 - (21562.267 - (108.973 - 0.612 * myCosBfSquare)* myCosBfSquare)* myCosBfSquare;
}

void xyToLB::setZ()
{
    myZ = myy / (myNf * myCosBf);
}

void xyToLB::setb2()
{
    myb2 = (0.5 + 0.003369 * myCosBfSquare) * mySinBf * myCosBf;
}

void xyToLB::setb3()
{
    myb3 = 0.333333 - (0.166667 - 0.001123 * myCosBfSquare)* myCosBfSquare;
}

void xyToLB::setb4()
{
    myb4 = 0.25 + (0.16161 + 0.00562 * myCosBfSquare)* myCosBfSquare;
}

void xyToLB::setb5()
{
    myb5 = 0.2 - (0.1667 - 0.0088 * myCosBfSquare)* myCosBfSquare;
}

void xyToLB::setl()
{
    myl = (1 - (myb3 - myb5 * myZ * myZ)* myZ * myZ) * myZ * RHO;
}

void xyToLB::setL()
{
    if (myDistinguish == 6)
        myL0 = 6 * myOrder -3;
    else
        myL0 = 3 * myOrder;

    myL = unitSwitch::degreeToSec(myL0) + myl;
}

void xyToLB::setB()
{
    myB = myBf - (1 - (myb4 - 0.12 * myZ * myZ)* myZ * myZ)* myZ *myZ * myb2 * RHO;
}

void xyToLB::setAll()
{
    setBeta();
    setCosAndSinBeta();
    setBf();
    setCosAndSinBf();
    setNf();
    setZ();
    setb2();
    setb3();
    setb4();
    setb5();

    setB();
    setl();
    setL();
}
