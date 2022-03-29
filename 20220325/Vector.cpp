#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

vector<vector<int>> solution(vector<vector<int>> arr1, vector<vector<int>> arr2){
	   vector<vector<int>> ans(arr1.size(), vector<int>(arr2[0].size(),0));
	   
	   
	   for(int i=0; i<arr1.size(); i++){
		    for(int j=0; j<arr2[0].size(); j++){
				   int sum=0;
				   for(int k=0; k<arr2.size(); k++){
					   
					      sum += arr1[i][k] * arr2[k][j];
					   }
					   ans[i][j] = sum;
				}
		   }
		   return ans;
		   
		  
	}
	
int main(){
	   ios_base::sync_with_stdio(false);
	   cin.tie(NULL);
	   
	   vector<vector<int>> arr1{{1,4},{3,2},{4,1}};
	   vector<vector<int>> arr2{{1,3},{4,1},{2,4}};
	   
	   
	   vector<vector<int>> ans=solution(arr1,arr2);
	     for(vector<int> a: ans){
		      for(int b:a){
				            cout << b << " ";
				  }
				  cout <<"\n";
		   }
	   
	    
	   return 0;
	}
