#include</anaconda/include/boost/timer/timer.hpp>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <ctime>
//#include <boost/timer/timer.hpp>
//#include </Users/Moti/Downloads/boost_1_67_0/boost/mpi/timer.hpp>
//#include <timer>
#include</anaconda/include/boost/timer/timer.hpp>
#include <numeric>
#include <iostream>
#include <fstream>
#include "CityMap.hpp"
#include "KdTrip.hpp"
#include "Cab.hpp"
using namespace std ;
int main(){
	boost::iostreams::mapped_file mfile("/../Data/nytaxi_500k.bin",
                                      boost::iostreams::mapped_file::priv);
	int nTrip = mfile.size()/sizeof(KdTrip::Trip);
	cout<<"Hey:"<<nTrip<<endl ;
	//mfile.const_data();
}
