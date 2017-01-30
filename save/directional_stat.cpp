#include <iostream>
#include <stdio.h>
#include <math.h>
using namespace std;
int main (){

	int a [20] = {10, 3, 16, 11, 9, 18, 16, 13, 7, 4, 9, 11, 17, 14, 19, 7, 15, 20, 31};
	float X=0; float Y=0; int n=20; double theta;

	for (int i=0; i<=20; i++)
	{
		Y = sin (a[i])/n;
		X = cos (a[i])/n;
		float r = sqrt (X*X + Y*Y);
		double cos_x = X/r;
		double sin_y = Y/r;
		theta = atan(sin_y/cos_x);
		cout << "result\n" <<"\n"<< theta <<"\n"; 
	}
      
}