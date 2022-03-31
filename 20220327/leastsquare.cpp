#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

void leastSquare(int arr1[], int arr2[], int n);
float mean(int arr[], int n);

int main(void){
  
  int arr1[4] = {10,20,30,40};
  int arr2[4] = {71,45,24,8};
  
  leastSquare(arr1,arr2,4);
  
}

void leastSquare(int arr1[], int arr2[], int n){
   
   float x_bar = mean(arr1,n);
   float y_bar = mean(arr2,n);
   
   float a,b;
   float sum1=0,sum2 = 0;
   
   for(int i=0; i<n; i++){
      sum1 += (arr1[i] - x_bar) * (arr2[i] - y_bar);
      sum2 += pow((arr1[i] - x_bar), 2);
   }
   
   a = sum1 / sum2;
   
   b = y_bar - a * x_bar;
   
   cout << "" << a << endl;
   cout << "" << b << endl;
   
}

float mean(int arr[] , int n){
  float sum = 0;
  float mean =0;
  
  for(int i=0; i< n; i++){
     sum += arr[i];
  }
  
  mean = sum / n;
  
  return mean;
}
