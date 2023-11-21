#include "BezierCurve.h"
#include "math.h"
#include "mbed.h"

extern Serial pc;
int factorial(int k) {
    int f = 1;
    for (int j = 2 ; j<= k ; j++)
        f*=j; 
    return f;   
}

BezierCurve::BezierCurve(int dim, int order) :_dim(dim), _order(order) {
    _pts = new float*[_order+1];
    _nck = new int[_order+1];
    _nck_deriv = new int[_order];
    _nck_deriv2 = new int[_order - 1];
    int facn = factorial(_order);
    int facn2= factorial(_order-1);
    int facn3= factorial(_order-2);
    for(int i = 0 ; i <= _order ; i++) {
        _pts[i] = new float[_dim];
        _nck[i] = facn / factorial(i) / factorial(_order-i);
        if (i< _order) {
            _nck_deriv[i] = facn2 / factorial(i) / factorial(_order-1-i);
        }
        if (i< _order - 1) {
            _nck_deriv2[i] = facn3 / factorial(i) / factorial(_order-2-i);
        }
    }
}

BezierCurve::~BezierCurve() {
    for(int i = 0 ; i < _dim ; i++) {
        delete _pts[i];    
    }    
    delete _pts;
}

void BezierCurve::setPoints(float pts[] ) {
    pc.printf("Setting Points\n");
    float * p = pts;
    for(int i = 0 ; i<=_order ; i++) {
        pc.printf("\n\r\tPt. %d:",i);
        for( int j = 0 ; j < _dim ; j++) {
            _pts[i][j] = *p;
            p++;
            pc.printf("\t\t%f",_pts[i][j]);
        }    
    }
}

void BezierCurve::evaluate(float time, float point[]) {
    //float *_point = new float[_dim];
    
    for(int i=0; i< _dim ; i++) {
        point[i] = 0;
    }        
    for(int i=0; i<=_order ; i++) {
        float mi = pow(time,i)*pow(1-time,_order-i) * _nck[i];
        for(int j=0 ; j < _dim ; j++) {
            point[j] += _pts[i][j] * mi;
        }    
    }
    //for(int i=0; i< _dim ; i++) {
    //    point[i] = _point[i];
    //} 
    //delete _point;
}

void BezierCurve::evaluateDerivative(float time, float point[]) {
    //double *_point = new double[_dim];
    for(int i=0; i< _dim ; i++) {
        point[i] = 0;
    }    
    //double dtime = time;    
    for(int i=0; i<=_order-1 ; i++) {
        float mi = pow(time,i)*pow(1-time,_order-1-i) * _nck_deriv[i] * _order;
        for(int j=0 ; j < _dim ; j++) {
            point[j] += (_pts[i+1][j] - _pts[i][j] ) * mi;
        }    
    }
    //for(int i=0; i< _dim ; i++) {
    //    point[i] = _point[i];
    //} 
    //delete _point;
}

void BezierCurve::evaluateDerivative2(float time, float point[]) {
    //double *_point = new double[_dim];
    for(int i=0; i< _dim ; i++) {
        point[i] = 0;
    }    
    //double dtime = time;    
    for(int i=0; i<=_order-2 ; i++) {
        float mi = pow(time,i)*pow(1-time,_order-2-i) * _nck_deriv2[i] * _order * (_order - 1);
        for(int j=0 ; j < _dim ; j++) {
            point[j] += (_pts[i+2][j] - 2*_pts[i + 1][j] + _pts[i][j]) * mi;
        }    
    }
    //for(int i=0; i< _dim ; i++) {
    //    point[i] = _point[i];
    //} 
    //delete _point;
}