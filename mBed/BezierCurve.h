class BezierCurve
{
public:
    BezierCurve(int dim, int order);
    ~BezierCurve();
    void setPoints(float pts[]); 
    void evaluate(float time, float point[]);
    void evaluateDerivative(float time, float point[]);
    void evaluateDerivative2(float time, float point[]);
private:
    const int _dim;
    const int _order;
    float ** _pts;
    int * _nck;
    int * _nck_deriv;
    int * _nck_deriv2;
};