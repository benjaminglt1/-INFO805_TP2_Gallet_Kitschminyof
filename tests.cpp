#include <iostream>
#include "PointVector.h"

using namespace std;
using namespace rt;

bool testPointVecteur()
{
  Point3 p = { 1.0, 0.0, 0.0 };
  cout << "p=" << p << endl;
  Vector3 w = { 0.5, 3.0, 2.0 };
  cout << "w=" << w << endl;
  cout << "p+w=" << p+w << endl;
  cout << "p-w=" << p-w << endl;
  cout << "||w||^2=" << w.dot(w) << endl;
  return true;
}
/*
int main( int argc, char* argv[] )
{
  bool ok = testPointVecteur();
  return ok;
}*/
