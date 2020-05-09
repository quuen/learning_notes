#include <iostream>
#include <vector>
#include <queue>
using namespace std;
double calibration_value=0;
queue<double> throttle_que;

//throttle_que.push(calibration_value);
int main()
{
   throttle_que.push(calibration_value);
   if(throttle_que.size()>5)
   {
       
       throttle_que.pop();
   }
}


 