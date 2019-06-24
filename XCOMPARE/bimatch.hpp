#ifndef BipartedMatching_HPP
#define BipartedMatching_HPP

#include <vector>
#include <limits>
#include <iostream>
#include <queue>
#include <functional>
#include <stdio.h> 
#include "Cab.hpp"
#include <string.h>
#include <map>
using namespace std ; 
class bimatch
{
public:
	std::vector<int> cabs ;
	std::vector<int> pointers;
	std::vector <int> Sol_Cabnum;
	bimatch(std::vector<int> p, std::vector<int> s, std::vector<int> c){
		this->pointers = p ;
		this->Sol_Cabnum = s ;
		this->cabs = c ;
	}
	bool bpm(int u,std::map<int, bool> &seen,  std::map<int,int> &matchR) ;
	void maxBPM(std::map<int, int> &matchR) ;
};
	inline bool bimatch::bpm( int u, std::map<int,bool> &seen, std::map<int,int> &matchR) { 
	    int count ; 
	    if(u!=this->pointers.size()-1){
	    	count = this->pointers.at(u+1) - this->pointers.at(u) ;
		}
	    else{
	    	count = this->Sol_Cabnum.size() - this->pointers.at(u) ;
		}
	    
	    for (int v = 0; v < count; v++) { 
	        int cab_num = this->Sol_Cabnum.at(this->pointers.at(u)+v) ;
	        if (!seen[cab_num]){ 
	            seen[cab_num] = true;  
	            if (matchR[cab_num] < 0 || this->bpm(matchR[cab_num], seen, matchR)){ 
	                matchR[cab_num] = u ; 
	                return true; 
	            } 
	        } 
	    }
	    return false; 
	} 

	inline void bimatch::maxBPM(std::map<int, int> &matchR){   
        int temp ;
	    for(int i=0 ;i<this->cabs.size();i++){
	    	matchR.insert(pair<int,int>(this->cabs.at(i),-1)) ; 
	    }
	    int result = 0;  
	    for (int u = 0 ; u < this->pointers.size() ; u++){ 
	        std::map<int, bool> seen ;
	        for(int i=0 ;i<this->cabs.size();i++) {
	        	seen.insert(pair <int, bool> (this->cabs.at(i),false)) ; 
	        }
	        if (this->bpm( u, seen, matchR)) 
	            result++; 
	    } 
	}

#endif
