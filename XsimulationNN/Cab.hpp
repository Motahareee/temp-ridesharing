#ifndef CAB_HPP
#define CAB_HPP

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <deque>
#include "CityMap.hpp"
#include "CabTrip.hpp"
#include <stack>



// default speed per second
#define DEFAULT_SPEED (17398400/3600)
#define MAX_DELAY_DISTANCE 1449866 //5 min
#define MAX_EXTRA_DISTANCE 1449866 //5 min
#define debug -1

#define DROPOFF_DELAY 0
#define PICKUP_DELAY 0


/*
 *Class for a cab
 */
class Cab
{
public:
  // A Stop is either a pickup, a dropoff or a destination that a taxi needs to go to
  struct Stop {
    int       stop_wait;
    int       intersection; // location of the stop
    int       passengers;   // the number of passenger - possitive: pickup, negative: dropoff, zero: just a waypoint
    int       distance;     // distance from the last stop to this stop
    int       max_extra;    // the maximum extra distance incurred for from all the stops after this stop
    int       max_delay;    // the maximum distance delayed to pick up after this stop
    int       dist_acc;     // the accumulate distance from this stop to the final stop
    int       spring_gap;
    int       occ;          // occupancy at this stop
    CabTrip * trip;         // pointing to the trip associated with this stop, or NULL if it's just a random destination
    Stop(): stop_wait(0),intersection(-1), passengers(0), distance(0), max_extra(0), max_delay(0), dist_acc(0), occ(0), trip(0) {}
    Stop(int w, int i, int p, int d, int o, CabTrip *t): stop_wait(w), intersection(i), passengers(p), distance(d), max_extra(0), max_delay(0), 
						  dist_acc(0), spring_gap(0), occ(o), trip(t) {}
  };
  typedef  std::list<Stop>  StopList;
  StopList stops;        // Stops being served by the cab
  int      id;           // This is the (unique) id of the cab (medallion?)
  int      max_share;    // The maximum number of shared trips
  int      capacity;     // Capacity of the cab
  int      occupancy;    // Current occupancy of the cab, counting all of its destination
  int      cur_pos;      // The current position of the cab
  int      speed;        // Current speed
  int      odometer;     // The total distance travel for the cab throughout the simulation
  int      dist_revenue; // The total distance that the taxi traveled with some passengers
  int      dist_driven;  // The distance traveled from the previous stop (not necessary the same as cur_pos) towards the first stop
  int      last_update;  // The time of the last update_cab() call
  int      state;
  int      prev_stop ;

  std::vector<int> pathIntersection; // intersections of the path towards the first stop, in reverse
  std::vector<int> pathDistance;     // accumalated distance of the path intersection, in reverse
  int              cur_index;        // the current index along the path given this traveled distance

  // Caching computation for finding shareable stops
  bool                     shareability_changed; // Whether the shareable stop should be recomputed
  std::pair<StopList::iterator, int> saved_stop; // The last saved stop

  struct ServingOrder
  {
    int pick;
    int drop;
    ServingOrder(int p=-1, int d=-1): pick(p), drop(d) {}
    void reset(int p=-1, int d=-1) { this->pick = p; this->drop = d; }
    bool isValid() { return this->pick>=0; }
  };
  
public:
  Cab(int num_share, int cap=4, int id=-1);
  void  set_max_share(int num_share);

public:
  virtual ~Cab() {}
  virtual int   update_cab(CityMap *city, int time, stack<int> &rebalancing_set);
  virtual int   cost_to_share(CityMap *city, const CabTrip *trip, ServingOrder *so,
                      int pick_loc, int drop_loc, int walk_wait, int &pick_wait, int &cab_wait);
  virtual void  add_trip(CityMap *city, CabTrip *trip, int pick_loc, int drop_loc,
                      int &cab_wait, const ServingOrder &order);
  virtual void  gen_stop_idle(CityMap *city);

protected:
  int   find_shareable_stop(StopList::iterator &skip_it, int passengers);
  void  advance_odometer(int distance);
  int   pop_stop(int distance);
  bool  gen_path(CityMap *city, int src, int dst, int delay);
  void  gen_stop(CityMap *city, stack<int> &rebalancing_set);
};

typedef std::vector<Cab> CabList;

inline Cab::Cab(int n, int cap, int m)
    : id(m), max_share(n), capacity(cap), occupancy(0), cur_pos(-1), speed(DEFAULT_SPEED),
      odometer(0), dist_revenue(0), dist_driven(0), last_update(-1),
      cur_index(-1), shareability_changed(true), state(1), prev_stop(0)
{
}

inline void Cab::set_max_share(int num_share)
{
  this->max_share = num_share;
  this->shareability_changed = true;
}
/////////////////////////////////////////////////////////////////////////////////////////////
inline int Cab::update_cab(CityMap *city, int time, stack<int> &rebalancing_set)
{
 
  // If this is the first update ever, making it like we're not moving
  if (this->last_update<0)
    this->last_update = time;
  
  // Update the timestamps
  int time_passed = time - this->last_update;
  this->last_update = time;
  
  // if we are not moving, no need to update  
  if(time_passed<=0 || (stops.size()==1 && stops.front().passengers==0 && stops.front().distance==0)){
    return this->cur_pos;
  }
  
  //cout<<"track 3"<<endl;
  // if we don't have a position (aka we could pop up anywhere),
  // teleport the cab to the first stop and reset the traveled
  // distance to 0 if possible
  if (this->cur_pos<0 && !this->stops.empty())
    this->cur_pos = this->pop_stop(0); ////// do we still need this ?
  
  // Now update the actual distance traveled
  int distance = this->speed*time_passed;
  int pop_time ;
  if(this->stops.front().distance!=0){
	pop_time = (this->stops.front().distance - this->dist_driven)/DEFAULT_SPEED ;
  }

  this->dist_driven += distance;
  if(this->id==debug)
    cout<<":::::: "<<distance<<" ::::: "<<this->dist_driven<<endl;

  if(this->id==debug)
    cout<<"stop distance"<<this->stops.front().distance<<endl;

  // Eliminate all reachable stops within the traveled distance
  while (!this->stops.empty() && this->dist_driven >= this->stops.front().distance){
    const int stop_distance = this->stops.front().distance;
    if(this->id==debug)
    cout<<"Front stop distance"<<stop_distance<<endl ;
    // if the cab is not hired, just drive back and forth
    
    // otherwise take out one stop at a time
    this->dist_driven -= stop_distance ;
    
    //if it is a pick up second pair we dont need to advance odometer
    
    if(this->stops.front().passengers<=0){
      if(this->prev_stop==-1){
        this->advance_odometer(stop_distance - DROPOFF_DELAY) ;
      }
      else if(this->prev_stop==1){
        this->advance_odometer(stop_distance - PICKUP_DELAY) ;
      }
      else{
        this->advance_odometer(stop_distance) ;
      }
    }
    //this->advance_odometer(stop_distance) ;
    cout<<"pop_time "<<this->last_update + pop_time<<endl ;
    this->cur_pos = this->pop_stop(this->dist_driven) ;
    distance = this->dist_driven ;
    if(!this->stops.empty()){
    	pop_time += this->stops.front().distance/DEFAULT_SPEED ;
    }
  }
  
  // the cab has passed all stops or has no position, make a new one
  if (this->stops.empty()) {
    this->gen_stop(city, rebalancing_set);
  }
  
  // otherwise...
  //we need to generate path for rebalancing and to-share states
  if (this->state==3 || (this->state==1 && this->stops.front().distance!=0)) {
    if(this->id==debug)    
      cout<<"state"<<this->state<<endl;
    int prev_delay = 0 ;
    if(this->prev_stop==1) prev_delay = PICKUP_DELAY ;
    else if(this->prev_stop==-1) prev_delay = DROPOFF_DELAY ;

    // find the actual path if we haven't done so
    if (this->pathIntersection.empty()){
      this->gen_path(city, this->cur_pos, this->stops.front().intersection, prev_delay);
    }
    
    //int temp_dis_driven = this->dist_driven - prev_delay ;
    while (this->dist_driven>=this->pathDistance.at(this->cur_index)) {
      --this->cur_index;
      if(this->cur_index==-1){
        cout<<"::-1"<<endl;
	cout<<this->id<<endl ;
      }

    }
    
  }

  return this->cur_pos;
}

inline void Cab::advance_odometer(int distance)
{
  if(this->id==debug)
    cout<<"ADVANCE ------"<<distance<<endl;
  this->odometer += distance;
  if (this->occupancy)
    this->dist_revenue += distance;
}

///////////////////////////////////////////////////////////////////////////////////////////
// Take out the first stop (aka we have arrived!) and use "distance" as
// the initial dist_driven value
inline int Cab::pop_stop(int distance)
{  
 //if(this->id ==0) cout<<"poped ::"<<this->stops.front().stop_wait<<endl;
  cout<<"pop_cabID "<<this->id<<endl;
  cout<<"pop_loc "<<this->stops.front().intersection<<endl;
  cout<<"pop_pass "<<this->stops.front().passengers<<endl;

  this->dist_driven = distance; 
  if (this->stops.empty())
    return -1;
  

  //update state
  //if we pop a rebalancing stop the state does not change so no need to update the state
  //if we pop a pickup stop
  if(this->stops.front().passengers>0){
    this->state = 3 ;
    this->prev_stop = 1 ;
  }
  else if(this->stops.front().passengers<0){
    this->prev_stop = -1 ;
      //if it is the last stop
    if(this->stops.size()==1){
      this->state=1 ;
      if(this->id==debug) cout<<"state1"<<endl ;
    }
    else{
      this->state=3 ;
      if(this->id==debug) cout<<"state3"<<endl ;
    }
  }
  else{ //if we pop a wait stop
    this->prev_stop = 0 ;
    if(this->stops.size()==1){
      this->state = 1  ;
      if(this->id==debug) cout<<"state1"<<endl ;
    }
    else{
      this->state = 2  ;
      if(this->id==debug) cout<<"state2"<<endl ;
    }
  }

  //if(this->stops.front().passengers!=0)
  this->shareability_changed = true;

  StopList::iterator stop = this->stops.begin();
  int loc = stop->intersection;
  
  // Update occupancy and trip information
  this->occupancy += stop->passengers;
  
  if (this->occupancy > this->capacity) {
    fprintf(stderr, "CAPACITY EXCEEDED!\n");
    fprintf(stderr, "id %d、occupancy = %d \n", this->id, this->occupancy);
    exit(EXIT_FAILURE);
  }
  if (stop->passengers>0) {
    stop->trip->set_picked(this->id);
    stop->trip->wait_time = (this->last_update-stop->distance/this->speed)-stop->trip->trip->pickup_time;
    stop->trip->dist_actual = this->odometer;
    //this->speed = stop->trip->speed; //commented for comparison
  }
  else if (stop->passengers<0) {
    stop->trip->set_complete(true);
    stop->trip->dist_actual = this->odometer - stop->trip->dist_actual ;
  }
  //else if(this->stops.size()>1){
  //  stop->trip->dist_actual = this->odometer ;
  //}
  // remove the stop and invalidate the current path
  this->stops.pop_front();
  this->pathIntersection.clear();
  this->pathDistance.clear();
  
  return loc;
}

//////////////////////////////////////////////////////////////////////////////////////
// Randomly select a place and making the cab go there
inline void Cab::gen_stop(CityMap *city, stack<int> &rebalancing_set)
{
  this->state = 1 ;//to hire

  if (this->cur_pos<0) {
      this->cur_pos = random()%city->numIntersections();
  }
  int location = -1;
  int distance = -1;
  if(!rebalancing_set.empty()){
    stack<int> temp ;
    while(distance<=0 && !rebalancing_set.empty()){
      if(location!=-1) temp.push(location);
      location = rebalancing_set.top();
      rebalancing_set.pop();
      distance = city->findShortestDistance(this->cur_pos, location);
    }
    while(!temp.empty()){
      rebalancing_set.push(temp.top());
      temp.pop();
    }
    if(distance>0){
      this->stops.push_front(Stop(0, location, 0, distance, 0, NULL));
      this->dist_driven = 0 ;
    }
    else {gen_stop_idle(city);}
  }
  else gen_stop_idle(city);
}

inline void Cab::gen_stop_idle(CityMap *city)
{
  this->state = 1 ;
  int location = this->cur_pos ;
  int distance = city->findShortestDistance(this->cur_pos, location);
       
  this->stops.push_front(Stop(0,location, 0, distance, 0, NULL)) ;
  this->pathIntersection.clear();
  this->dist_driven = 0 ;
}

//////////////////////////////////////////////////////////////////////////////////////

// Creating a path from src to dst and its computing accumulated
// distance, stored in a reverse order in pathIntersection and
// pathDistance
inline bool Cab::gen_path(CityMap *city, int src, int dst, int delay)
{
  CityMap::PathIterator path;
  city->findShortestPathIterator(src, dst, path);
  if (!path.isValid()) return false;
  int acc = path.totalDistance;
  this->pathDistance.clear();
  this->pathIntersection.clear();
  while (acc>0) {
    this->pathIntersection.push_back(path.current());
    this->pathDistance.push_back(acc);
    acc -= path.currentDistance;
    if (path.hasMore()) ++path;
    else break;
  }
  this->pathDistance.push_back(0);
  this->pathIntersection.push_back(src);
  
  this->cur_index = this->pathDistance.size()-1;
  
  if(delay>0){
    for(int i=0 ; i<pathDistance.size();i++)
      pathDistance[i] += delay ;
    
    this->pathDistance.push_back(0);
    this->pathIntersection.push_back(src);
  } 
  return true;
}
/////////////////////////////////////////////////////////////////////////////////////

inline int Cab::cost_to_share(CityMap *city, const CabTrip *trip, ServingOrder *so,
                              int pick_loc, int drop_loc, int walk_time, int &pick_wait,
                              int &cab_wait)
{
  int equ_walk_distance = walk_time * DEFAULT_SPEED ;
  int distance = city->findShortestDistance(pick_loc, drop_loc) + PICKUP_DELAY ;

  so->reset(-1, this->stops.size());
  
  int dis=0, prev, prev_delay=0 ;

  //if the cab is not moving cur_pos is its location
  if(this->state==1 && this->stops.front().distance==0){
    prev = this->cur_pos ;
    //prev_type = 0 ;
    dis = 0 ;
  }
  //otherwise we need to find its upcoming intersection
  else if(this->state==2){
      prev = this->cur_pos ;
      dis = this->stops.front().distance - this->dist_driven ;
  }
  else{
      prev = this->pathIntersection.at(this->cur_index);
      dis = this->pathDistance.at(this->cur_index) - this->dist_driven;
  }
 
  // if the cab is available for hire, pick up this trip
  if (this->state==1) {
    const int d = city->findShortestDistance(prev, pick_loc) ;    
    if(d<0 || d+dis > MAX_DELAY_DISTANCE || equ_walk_distance>MAX_DELAY_DISTANCE) return -1;
    
    //check if the cab needs to wait for the passenger
    if(d+dis < equ_walk_distance){
      cab_wait = equ_walk_distance - (d+dis) ;
    }
    so->pick = 0 ;
    so->drop = 1 ; //moti added
    
    pick_wait = (d+dis+cab_wait)/DEFAULT_SPEED ;
    return dis+d+distance;
  }
  
  // otherwise, do full calculation. There is a fixed cost of dropping
  // of this trip after the last dropoff location since the end of the list is definitly a dropoff
  // we need to add Dropoff Delay as well
  int drop_cost = city->findShortestDistanceReverse(this->stops.back().intersection, drop_loc) +  DROPOFF_DELAY ;
  // ... but if the path doesn't exist, bail out
  
  if (drop_cost - DROPOFF_DELAY <0) return -1;
  
  // Next, we need to determine how soon can we start take this trip
  // in given the max number of shares. Since we always put this trip
  // dropoff at the end, the number of shares will increase for all
  // trips that would be dropped off between the start of this trip to
  // the end. So we need to look pass the latest dropoff that already
  // has the maximum number of shares. If there's none, we start at
  // the beginning.
  StopList::iterator skip_it ;
  int vacancy = this->find_shareable_stop(skip_it, trip->trip->passengers);
  if(this->id==debug and 1==2){
    cout<<vacancy<<endl;
    cout<<skip_it->passengers<<endl ;
    cout<<":size: "<<this->stops.size()<<endl ;
    cout<<(skip_it!=this->stops.begin())<<endl;
  }
  
  if(this->cur_pos==this->stops.front().intersection && skip_it==this->stops.begin()){
    skip_it++ ;
    //skip_it++ ;
  }
  bool skip = skip_it->trip->shares+1 > this->max_share || vacancy<0 || (skip_it!=this->stops.begin()) ;
  //if (skip && (skip_it==--this->stops.end())) return -1;
  // Now we try all possible segment, only after the skipped trip
  int occ = this->occupancy;
  int order=0;
  int min_cost = INT_MAX;
  int pick_dist_prev = 0; // the distance from previous stop to the pick location
  int pick_dist = 0; // the distance from the pick location to the next stop
  int pick_total_dis = 0 ;
  int best_cab_wait = 0, cab_wait_ = 0 ;
  int prev_type = 0 ;
  int bestpick_spring_gap=0, bestdrop_spring_gap=0 ;
  int pick_max_extra = 0 ;
  
   StopList::iterator t = this->stops.begin();
   if(this->id==debug && 1==2)
    for(;t!=stops.end();t++){
     cout<<":"<<t->passengers<<"\t";
   }
   if(this->id==debug && 1==2)
  cout<<endl;
  
  for (StopList::iterator it=this->stops.begin(); it!=this->stops.end() && dis<MAX_DELAY_DISTANCE; ++it, ++order) {
    //int dist_left = 0 ;

    int dist_left = it->distance-((order==0 && this->cur_pos!=this->stops.front().intersection)?this->pathDistance.at(this->cur_index):0);
    
    if(it->passengers<=0){
      if(prev_type==1) prev_delay = PICKUP_DELAY ;
      else if(prev_type==-1) prev_delay = DROPOFF_DELAY ;
      else prev_delay = 0 ;

  //if(this->id == debug) cout<<"WOww 5"<<endl;
      
      // we are not yet at the stop that can accommodate shares
      if (skip)
        skip = it!=skip_it;

      // check if the passengers would fit in the cab
      //else if (it->wait_stop!=-1 && it->occ+trip->trip->passengers<=this->capacity) {
      else if (it->occ + trip->trip->passengers <= this->capacity) {
        // and the distance from here to when we pick them up should meet our condition
        int d = city->findShortestDistanceReverse(prev, pick_loc) + prev_delay ;
        
        if(d+dis < equ_walk_distance){
          cab_wait_ = equ_walk_distance - (d+dis) ;
        }
        else{
          cab_wait_ = 0 ;
        }

        //check if it OK for the trip
        if ((d>=0) && (dis+d+cab_wait_ <= MAX_DELAY_DISTANCE)) {
          int cost = d + cab_wait_ ; // ag
          pick_dist_prev = d ;
          d = city->findShortestDistance(pick_loc, it->intersection) + PICKUP_DELAY ;

          if (d>=0 && (d + it->dist_acc + drop_cost - distance) <= MAX_EXTRA_DISTANCE+it->spring_gap) {
            cost += d - dist_left;
            
            if ((cost+it->max_extra) <= MAX_EXTRA_DISTANCE+it->spring_gap && (cost+it->max_delay) <= MAX_DELAY_DISTANCE) {
              cost += drop_cost;
              if (cost<min_cost) {
                min_cost = cost;
                pick_dist = d ;
                pick_total_dis = pick_dist_prev + dis ;
                bestpick_spring_gap = it->spring_gap ;
                so->pick = order;
		pick_max_extra = it->max_extra ;
              }
            }
          }
        }
      }
    }
    prev = it->intersection;
    prev_type = it->passengers ;
    occ += it->passengers;
    dis += dist_left;
  }
  //if(this->id==debug && pick_loc==4175) cout<<"track 5"<<endl;
  { 
    if (order == this->stops.size()) {
      if(prev_type==1) prev_delay = PICKUP_DELAY ;
      else if(prev_type==-1) prev_delay = DROPOFF_DELAY ;
      else prev_delay = 0 ;
      int d = city->findShortestDistanceReverse(prev, pick_loc) + prev_delay ;
      if(dis+d<equ_walk_distance){
        cab_wait_ = equ_walk_distance - (d+dis);
      }
      else{
        cab_wait_ = 0 ;
      }
      if ((d>=0) && (dis+d+cab_wait_ <= MAX_DELAY_DISTANCE)) {
        int cost = distance + d;
	      if (cost<min_cost) {
	        min_cost = cost;
	        pick_dist = d;
          pick_total_dis = d+dis ; ///??? chera faghat d bud inja!?
          so->pick = order;
          bestpick_spring_gap = 0 ;
	        so->drop = so->pick;
        }
      }
    }
  }
  
  if(pick_total_dis <= equ_walk_distance){
    cab_wait = equ_walk_distance - pick_total_dis ;
  }
  
  const int pick_cost = min_cost - drop_cost;
  // After finding the pickup order, we'll optimize for the dropoff's
  if (so->pick>=0 && so->pick < this->stops.size()) {
    //const int pick_cost = min_cost - drop_cost;    
    int max_extra=0, max_delay = 0, last_dist=0, pass=2, spring_gap = 0;
    int next = -1;
    StopList::iterator it = --this->stops.end();
    for (order=this->stops.size()-1; ; --order, --it) {
      
      int intersection = (order<so->pick)?pick_loc:it->intersection;

      prev_type = it->passengers ;

      if(pass!=1){
      	if(prev_type==1) prev_delay = PICKUP_DELAY ;
      	else if(prev_type==-1) prev_delay = DROPOFF_DELAY ;
      	else prev_delay = 0 ;
      	if (next>=0) {
            int cost = city->findShortestDistance(drop_loc, next)+ DROPOFF_DELAY ;
            
          if (cost>=0) {
            int d = city->findShortestDistanceReverse(intersection, drop_loc) + prev_delay ;
          	
            if (d>=0) {	   
            	    cost += pick_cost + d -last_dist;
            	    //if ((cost+max_extra<MAX_EXTRA_DISTANCE) && cost<min_cost && equ_walk_distance<=cost+max_delay && cost+max_delay<MAX_DELAY_DISTANCE) {
                  if ((cost+max_extra<MAX_EXTRA_DISTANCE+spring_gap) && cost<min_cost && cost+max_delay<MAX_DELAY_DISTANCE) {
              		
                    min_cost = cost;
              		  so->drop = order+1;
                    bestdrop_spring_gap = spring_gap ;
            	    }
          	}
          }
        }
      }

      if (order<so->pick) break;
      next = intersection;
      last_dist = (order==so->pick)?pick_dist:it->distance;
      max_extra = it->max_extra;
      max_delay = it->max_delay;
      spring_gap = it->spring_gap ;
      pass = it->passengers ;
    }
  }
  //min_cost -= (bestpick_spring_gap - bestdrop_spring_gap) ;
   
  if(pick_cost - (bestpick_spring_gap - bestdrop_spring_gap) <= MAX_EXTRA_DISTANCE
     && pick_cost + pick_max_extra - (bestpick_spring_gap - bestdrop_spring_gap) <= MAX_EXTRA_DISTANCE) {
    min_cost -= (bestpick_spring_gap - bestdrop_spring_gap) ;
  }
  else{
    min_cost = -1 ;
  }
  pick_wait = pick_total_dis/DEFAULT_SPEED ;
  return min_cost;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// We need to determine how soon can we start take this trip in given
// the max number of shares. Since we always put this trip dropoff at
// the end, the number of shares will increase for all trips that
// would be dropped off between the start of this trip to the end. So
// we need to look pass the latest dropoff that already has the
// maximum number of shares. If there's none, we start at the
// beginning.
inline int Cab::find_shareable_stop(StopList::iterator &skip_it, int num_ppl)
{

  int vacancy;
  if (!this->shareability_changed && num_ppl == 1) {
    //if(this->id==debug) cout<<"AA!"<<endl;
    skip_it = this->saved_stop.first;
    vacancy = this->saved_stop.second;
    //if(this->id==debug) cout<<"AA!"<<endl;
  }
  else {
    //if(this->id==debug ) cout<<"BB!"<<endl;
    int max_extra = 0 ;
    int max_delay = 0 ;
    int dist_acc = 0 ;
    int spring_acc = 0 ;
    skip_it = --this->stops.end();
    vacancy = this->max_share;
    int MAX_DELAY_OFF, MAX_EXTRA_OFF ;
    while (1) {
      if (skip_it->passengers<0) {
        --vacancy;
        if (max_extra<skip_it->trip->dist_extra)
          max_extra = skip_it->trip->dist_extra;
      }
      else if (skip_it->passengers>0) {
	      if (max_delay<skip_it->trip->dist_delay)
	          max_delay = skip_it->trip->dist_delay;
      }
      else if(skip_it->stop_wait>0){
        //if (max_delay<skip_it->stop_wait)
        max_delay -= skip_it->stop_wait ;
        //max_extra -= skip_it->trip->dist_extra;
        //max_extra -= skip_it->trip->dist_extra;
        spring_acc += skip_it->stop_wait ;
      }

      skip_it->max_extra = max_extra ;
      skip_it->dist_acc = dist_acc ;
      dist_acc += skip_it->distance ;
      skip_it->max_delay = max_delay ;
      skip_it->spring_gap = spring_acc ;
      if (skip_it==this->stops.begin() || skip_it->trip->shares>=this->max_share || vacancy<0 || 
	               skip_it->occ+num_ppl > this->capacity){
        if(this->id==debug && 1==2){
          cout<<"M "<<(skip_it==this->stops.begin())<<endl ;
          cout<<"M "<<(skip_it->trip->shares>=this->max_share)<<endl ;
          cout<<"M "<<(vacancy<0)<<endl ;
          cout<<"M "<<(skip_it->occ+num_ppl > this->capacity)<<endl ;

        }
        break;
        //if(this->id==debug) cout<<"M"<<endl;
      }

      --skip_it;
    }
    if(this->id==debug & 1==2)
          cout<<"N "<<(skip_it==this->stops.begin())<<endl ;
    this->saved_stop.first = skip_it;
    this->saved_stop.second = vacancy;
    this->shareability_changed = false;
  }
  return vacancy;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline void Cab::add_trip(CityMap *city, CabTrip *trip, int pick_loc, int drop_loc,
                          int &cab_wait, const ServingOrder &so)
{
  
  bool idle = (this->stops.size()==1 && this->stops.front().passengers==0 && this->stops.front().distance==0) ;
  bool roaming = (this->stops.size()==1 && this->stops.front().passengers==0 && this->stops.front().distance!=0) ;

  //cout<<"add trip :: roaming "<<roaming<<", idle :: "<<idle<<endl ;


  int to_print ;
if(this->id==debug)  
  cout<<"add trip :: roaming "<<roaming<<", idle :: "<<idle<<endl ;
if(this->id==debug)
  cout<<"pickup "<<pick_loc<<", dropoff "<<drop_loc<< "cur_pos"<<this->cur_pos<<" original pickup "<<trip->pick_loc<<endl ;
if(this->id==debug)
  cout<<"serving order"<<so.pick<<"\t"<<so.drop<<endl ; 
/*
  StopList::iterator tt = this->stops.begin();
if(this->id==debug)
  for(;tt!=stops.end();tt++){
    cout<<":"<<tt->passengers<<"\t";
  }
if(this->id==debug)
  cout<<endl;
  */

  StopList::iterator it = this->stops.begin();

  std::advance(it, so.pick);
  StopList::iterator drop_it = it;
  if (so.drop<this->stops.size()){
    std::advance(drop_it, so.drop-so.pick);
  }
  else{
    drop_it = this->stops.end();
  }

  int occ_pick=-1, occ_drop=-1;
  StopList::iterator stop = it;
  if (so.pick == 0 && it!= this->stops.end() && it->passengers<=0) { // be nazar miad ke ghablesh darim add mikonim
    occ_pick = it->occ + trip->trip->passengers;
  } else if (so.pick == 0 && it!= this->stops.end() && it->passengers>0) { //added equal zero
    occ_pick = it->occ - it->passengers + trip->trip->passengers;
  } else if (so.pick == 0 || it == this->stops.end()) {
    occ_pick = trip->trip->passengers;
  } else {
    if (stop!=this->stops.begin()){
      --stop;
      if (stop->passengers < 0)
	       occ_pick = stop->occ + stop->passengers + trip->trip->passengers; //why we need  stop->passengers ?????

      else occ_pick = stop->occ + trip->trip->passengers;
        ++stop;
    }
  }
  // Increase the number of shares all trips that will be dropped and
  // increase the number of stops for only trips that have been
  // picked. For the trip being served, calculating the number of
  // shares based on pickup and dropoff.

  trip->stops = 0;
  trip->shares = 0;
  for (; stop!=drop_it; ++stop) {
    if (stop->passengers<0) {
      ++trip->shares;
      ++stop->trip->shares;
      //if (stop->trip->is_picked() && !stop->wait_stop)
      if (stop->trip->is_picked())
        ++stop->trip->stops;
    }
    if (stop->passengers)
      ++trip->stops;
    stop->occ += trip->trip->passengers; 
  }
 
  // increase additional stops for those after this dropoff and
  // increase shares only for trips picked before this dropped off
  if(!roaming && !idle){
    for (; stop!=this->stops.end(); ++stop) {
      if (stop->passengers>0)//add equality
        --stop->trip->shares; ///aslan motevajeh nemisham chera share ro kam mikone
      else if (stop->passengers<0)
        ++stop->trip->shares;
      if (stop->trip->is_picked())
        ++stop->trip->stops;
    }
  }

  StopList::iterator temp = drop_it;
  if (so.pick == so.drop){//it->passengers == 0) {//can be change to trip is idle or not

    occ_drop = occ_pick;
  } else if (this->stops.size()!=0) {

    --temp;
    if (temp->passengers < 0) {
      occ_drop = temp->occ + temp->passengers;
    } else {
      occ_drop = temp->occ ;
    }
  }
  //cout<<"add trip 5 "<<this->stops.size()<<endl ;
  
  int dist_extra = (it==this->stops.end())?0:it->distance;
  // Now inserting the trip. If this is the only trip
  if (it==this->stops.end()){
      it-- ;
      int distance = city->findShortestDistance(it->intersection, pick_loc) + DROPOFF_DELAY ;
      it++ ;
      this->stops.push_back(Stop(cab_wait, pick_loc, 0, distance, occ_pick, trip)) ;
      this->stops.push_back(Stop(PICKUP_DELAY, pick_loc, trip->trip->passengers,cab_wait, occ_pick, trip)) ;
      this->stops.push_back(Stop(DROPOFF_DELAY,drop_loc, -trip->trip->passengers, trip->distance, occ_drop, trip)) ;
      trip->dist_extra = 0;
  }
  else{   // otherwise, try to put it in between two other stops
    //cout<<"try to put it in between two other stops. "<<this->state<<endl;
    
    // update the distance of the trip if the taxi is not roaming
    if(!roaming && !idle){
      it->distance = city->findShortestDistance(pick_loc, it->intersection) + PICKUP_DELAY ;
    }  
    dist_extra = it->distance - dist_extra; //???????
    //cout<<"state"<<this->state<<" "<<(it==this->stops.begin())<<endl;
    // for the first stop, interpolate between cur_index and the first stop
    if (it==this->stops.begin()) {
      //if the cab is not idle updates its cur_pos
      if(!idle){
        //cout<<"ZERO chnage in add stop"<<endl;
        this->dist_driven -= this->pathDistance.at(this->cur_index);
        this->cur_pos = this->pathIntersection.at(this->cur_index);
      }
      else{
        //cout<<"ZERO add trip"<<endl;
        this->dist_driven = 0 ;
      }

      //cout<<"insert in the begining of list"<<this->stops.size()<<endl;
      const int distance = city->findShortestDistance(this->cur_pos, pick_loc);
      if(this->id==debug){
        cout<<"pick/drop"<<this->cur_pos<<", "<<pick_loc<<endl;
        cout<<"+++"<<distance<<endl;
      }
      const Stop wait_stop(cab_wait, pick_loc, 0, distance, occ_pick, trip);
      const Stop pickup_stop(PICKUP_DELAY, pick_loc, trip->trip->passengers, cab_wait, occ_pick, trip);
      //cout<<"to pick up distance ::"<<pick_loc<<","<<this->cur_pos<<","<<distance<<endl ;
      
      if(roaming || idle){
        this->stops.clear();
      }
      else{
        dist_extra += distance+this->pathDistance.at(this->cur_index);
      }

      this->stops.push_front(pickup_stop); 
      this->stops.push_front(wait_stop);
      if(roaming || idle){
        it = this->stops.begin();
        drop_it = this->stops.end();
      }
      this->gen_path(city, this->cur_pos, pick_loc, 0);
    }

    // otherwise, it wouldn't affect our current path, we just need to insert the stop
    //}
    else{
      //cout<<"insert the stop after all we have"<<this->stops.size()<<endl;
      StopList::iterator it2 = it;
      --it2;
      int prev_delay = 0 ;
      if(it2->passengers==1) prev_delay = PICKUP_DELAY ;
      else if(it2->passengers==-1) prev_delay = DROPOFF_DELAY ;

      const int distance = city->findShortestDistanceReverse(it2->intersection, pick_loc) + prev_delay ;
      this->stops.insert(it, Stop(cab_wait, pick_loc, 0, distance, occ_pick, trip));
      this->stops.insert(it, Stop(PICKUP_DELAY, pick_loc, trip->trip->passengers, cab_wait, occ_pick, trip));

      dist_extra += distance ;
    }

    //cout<<"track1"<<endl;
    dist_extra += cab_wait ;
    // add the (pickup) extra distance to the rest of the stops until the dropoff location  

    //cout<<"track2 "<<(it!=drop_it)<<","<<(it==this->stops.begin())<<","<<(it==this->stops.end())<<endl;
    
    //if(drop_it)
     //drop_it=this->stops.end();

    //if(!roaming && !idle){
      //cout<<"->"<<(it==this->stops.begin())<<endl;
    //if(1){
      if(!idle && !roaming){
      for (; it!=drop_it; ++it) {
        //cout<<"loop+"<<endl;
        //cout<<"ever !?!??"<<endl ;
        trip->dist_extra += it->distance ;
        //if(it->passengers==0){
        //  trip->dist_extra += it->stop_wait ;
        //}
        //cout<<"track3"<<endl;
        if (it->passengers<0){
          if(!it->trip->bet)
            it->trip->dist_extra += dist_extra ;
          else
            it->trip->bet = false ;
          //cout<<"track4"<<endl;

        }else if(it->passengers>0){
              //cout<<"track5"<<endl;

            //cout<<it->trip->pick_loc<<endl;
            //it->trip->dist_extra -= dist_extra ;
            it->trip->bet = true ;
            it->trip->dist_delay += dist_extra; 
              //cout<<"track5"<<endl;
        }
        else{
          if(it->stop_wait){
    //cout<<"track6"<<endl;

            if(dist_extra>= it->stop_wait){

              dist_extra -= it->stop_wait ;
              it->stop_wait = 0 ;
            }
            else{
    //cout<<"track7"<<endl;

              it->stop_wait -= dist_extra ;
              dist_extra = 0 ;
            }
            int updated_stopwait = it->stop_wait ;
            it++ ;
	    trip->dist_extra -= (it->distance - updated_stopwait) ;
            it->distance = updated_stopwait ;
            it-- ;
          }
        }
      }
    //}

    --it; //point to the element the position that drop off should be added
    }
    else{
      it++ ;
    }
    int prev_delay = 0 ;
    if(it->passengers==1) prev_delay = PICKUP_DELAY ;
    else if(it->passengers==-1) prev_delay = DROPOFF_DELAY ;
    
    const int distance = city->findShortestDistance(it->intersection, drop_loc) + prev_delay ;
    to_print = distance;
   
    this->stops.insert(drop_it, Stop(DROPOFF_DELAY, drop_loc, -trip->trip->passengers, distance, occ_drop, trip));
    
    if(!idle || !roaming){
      trip->dist_extra += distance-trip->distance;
    }
    
    //cout<<"track9"<<endl;
    
    // add the (dropoff) extra distance to the rest of the stops until the dropoff location
    if (drop_it!=this->stops.end()) {
      const int drop_cost = city->findShortestDistance(drop_loc, drop_it->intersection) + DROPOFF_DELAY ;
      int drop_extra = distance + drop_cost - drop_it->distance ;
      dist_extra += distance + drop_cost - drop_it->distance;
      drop_it->distance = drop_cost ;
      for (it=drop_it; it!=this->stops.end(); ++it) {
        if (it->passengers<0){
          if(it->trip->bet){
            it->trip->dist_extra += drop_extra;
            it->trip->bet = false ;
          }
          else if(it->trip->after){
            it->trip->dist_extra += 0;
            it->trip->after = false ;
          }
          else{
            it->trip->dist_extra += dist_extra;
          }
        } else if (it->passengers>0) {
            //it->trip->dist_extra -= dist_extra ; //why ???
	          it->trip->dist_delay += dist_extra ;
            it->trip->after = true ;
	      }
        else if(it->stop_wait){
          int change = 0 ;
          if(it->stop_wait >= dist_extra){
            it->stop_wait -= dist_extra ;
            dist_extra = 0 ;
            change = dist_extra ;
          }
          else{
            dist_extra -= it->stop_wait ;
            change = it->stop_wait;
            it->stop_wait = 0 ;
          }
          if(drop_extra!=0){
            drop_extra-=change ; 
            if(drop_extra<0)
              drop_extra = 0 ;
          }
          int updated_stopwait = it->stop_wait ;
          it++ ;
          it->distance = updated_stopwait ;
          it-- ;
        }
      }
    }
  }
  //cout<<"add trip 7 "<<this->stops.size()<<" distance "<<to_print<<endl ;
  
  StopList::iterator t = this->stops.begin();
if(this->id==debug)
  for(;t!=stops.end();t++){
    cout<<":"<<t->passengers<<"\t";
  }
if(this->id==debug)
  cout<<endl;
  
t = this->stops.begin();

  //cout<<"Ha !?"<<endl;


if(this->id==debug)    
  for(;t!=stops.end();t++){
    cout<<":"<<t->distance<<"\t";
  }
if(this->id==debug)    
  cout<<endl;

  //t = this->stops.begin();

  //cout<<"Ha !?"<<endl;

  //for(;t!=stops.end();t++){
  //  cout<<":"<<t->trip->dist_extra<<"\t";
  //}
  //cout<<endl;
  if(this->state==1)	this->state=3 ;
  this->shareability_changed = true ;
  //if(this->id==debug)
  //  cout<<"shareability_changed "<<this->shareability_changed<<endl ;

}

#endif
