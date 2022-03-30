#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <new>
#include <cstring>
#include <malloc.h>

using namespace std;



int datax[30] = {};
int datay[30] = {};
float n = 0;

float dis(float x, float y, float a, float b)
{
    return abs(a * x - y + b) / sqrt(a * a + b * b);
}

float f(float a, float b)
{
    float sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        sum += dis(datax[i], datay[i], a, b);
    }
    return sum / n;
}

float dfabda(float a, float b, float da)
{
    return (f(a + da, b) - f(a, b)) / da;
}

float dfabdb(float a, float b, float db)
{
    return (f(a, b - db) - f(a, b)) / db;
}

int main() {

    for (int i = 0; i < 20; i++) {
        cout << datax[i] - 0.5 << datay[i] + 0.5;
        cout << datax[i] << " " << datay[i] << " " << endl;
    }
    float a0 = 0, b0 = 0;
    float da = 0.01;
    float db = 0.01;
    float psi = 0.1;
    float a, a1 = 1; //임의로 찍어주는 기울기
    float b, b1 = 1;

    for (int i = 0; i < 50; i++) {
        a0 = a1;
        b0 = b1;
        a1 -= psi * dfabda(a0, b0, da);
        b1 -= psi * dfabdb(a0, b0, db);

    }
    cout << " y = " << a1 << "x + " << b1 << endl;
    return 0;
}
