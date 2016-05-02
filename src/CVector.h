#ifndef CVECTOR_H
#define CVECTOR_H

#include <cmath>
#include <iostream>

//=====================================================
// General Description:
//
// This is a class for storing basic 2 dimentional
// vectors in the linear algebra sense.  Note this is
// NOT a C++ vector (as in a list).  This class can
// add, subtract, multiply by scalars, find magnitude,
// and dot product of two vectors.
//=====================================================

class CVector
{
public:
    //class variables
    double xComponent;
    double yComponent;


    CVector(double x = 0.0, double y = 0.0); //overload constructor

    //operations
    CVector& operator= (const CVector& other);
    CVector operator+ (const CVector& other);
    CVector operator- (const CVector& other);
    double operator* (const CVector& other);
    CVector operator* (double scalar);

    //
    //member functions
    //
    //simple modulus of the vector
    double mod();

    //prints the vector data to the command line
    void printData();


};

#endif // CVECTOR_H
