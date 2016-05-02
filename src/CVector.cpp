#include "CVector.h"

CVector::CVector(double x, double y )//originally had both x and y = 0... why?
{
    xComponent = x;
    yComponent = y;
}

CVector& CVector::operator= (const CVector& other)
{
    xComponent = other.xComponent;
    yComponent = other.yComponent;
    return *this;
}

CVector CVector::operator+ (const CVector& other)
{
    return CVector(xComponent+other.xComponent, yComponent+other.yComponent);
}

CVector CVector::operator- (const CVector& other)
{
    return CVector(xComponent-other.xComponent, yComponent-other.yComponent);
}

double CVector::operator* (const CVector& other)
{
    return (xComponent*other.xComponent + yComponent*other.yComponent);
}

CVector CVector::operator* (double scalar)
{
    return CVector(xComponent*scalar, yComponent*scalar);
}

double CVector::CVector::mod ()
{
    return std::sqrt(xComponent*xComponent + yComponent*yComponent);
}

void CVector::printData ()
{
    std::cout << xComponent <<"," << yComponent << std::endl;
}
