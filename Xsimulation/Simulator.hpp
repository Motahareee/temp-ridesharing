#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <boost/thread/thread.hpp>
#include <boost/atomic.hpp>
#include <time.h>
#include "Cab.hpp"
#include <typeinfo>
#include <iostream>
#include <stack>
#include "CityMap.hpp"
using namespace std ;

#define DEFAULT_SPEED (17398400/3600)

#define MAX_DELAY_DISTANCE 1449866 //5 min
#define MAX_EXTRA_DISTANCE 1449866 //5 min
//#define MAX_DELAY_DISTANCE 933333
//#define MAX_EXTRA_DISTANCE 933333
class Simulator
{
public:
  static const int CABS_PER_CHUNK = 100;

  struct Parameters {
    CityMap * city;
    Cab    ** cabs;
    int       total_cabs;
    int       num_share;
    CabTrip * trip;
    int       num_trips;
    stack<int> rebalancing_set ;
  };
  struct Statistics {
    float avg_wait_overall = 0 ; //including unserved passangers
    float avg_wait = 0 ;
    float avg_travel_delay = 0 ;
    float avg_ = 0 ;
    float avg_occupancy = 0 ;
    float occ_counter = 0 ;
    float avg_total_occupancy = 0 ;
    float mean_odometer = 0 ; //average distance traveled (km)
    float avg_computation_time = 0 ;
    float average_total_walk = 0 ;
    float average_pickup_walk = 0 ;
    float average_dropoff_walk = 0 ;
    int zerocounter = 0 ;
    int under100 = 0 ;
    int under200 = 0 ;
    int under300 = 0 ;
    int under400 = 0 ;
    int tester = 0 ;
  };
  struct XPool {
    XPool() { this->reset(); }
    void reset() {
      this->best_pick = -1;
      this->best_drop = -1;
      this->cost = INT_MAX;
      this->walktoPick = -1;
      this->walkfromDrop = -1; 
      this->pick_wait = 0 ;
      this->cab_wait = 0 ;
      this->serving_order.reset(); 
    }
    int               best_pick;
    int               best_drop;
    int               cost;
    int               walktoPick;
    int               walkfromDrop;
    int               pick_wait;
    int               cab_wait;
    Cab::ServingOrder serving_order;
  };
  struct Solution {
    Solution() { this->reset(); }
    void reset() {
      this->min_cost = INT_MAX;
      this->min_cab_num = -1;
      this->min_serving_order.reset(); 
      this->X.reset();
    }

    bool operator<(const Solution &s) const {
      return (this->min_cost<s.min_cost) || (this->min_cost==s.min_cost && this->min_cab_num<s.min_cab_num);
    }

    int               min_cost;
    int               min_cab_num;
    Cab::ServingOrder min_serving_order;
    XPool             X;
  };
  
  Simulator(int n=0): shutdown(true) {
    this->start(n);
  }
  
  void start(int n) {
    if (!this->shutdown)
      return;
    this->shutdown = false;
    this->nThreads = n;
    this->workerBusy = this->nThreads;
    this->params.num_trips = 0;
    if (n<2)
      return;
    for (int i=0; i<this->nThreads; i++) {
      this->workerThreads.create_thread(boost::bind(&Simulator::worker, this, i));
    }
    this->waitForWorkers();
  }

  void stop() {
    this->shutdown = true;
    this->condWork.notify_all();
  }

  void init(CityMap *city, Cab **cabs, int total_cabs, int num_share) {
    this->params.city = city;
    this->params.cabs = cabs;
    this->params.total_cabs = total_cabs;
    this->params.num_share = num_share;
    this->numServed = 0;
    this->totalCost = 0;
    this->hourlyCost.resize(24, 0);
  }

  void dispatchToWorkers(CabTrip *trips, int cnt)
  {
    this->solution.reset();
    this->currentCab.store(0, boost::memory_order_release);
    this->workerBusy = this->nThreads;
    this->condWork.notify_all();
  }

  void waitForWorkers() {
    boost::mutex::scoped_lock lock(this->mutex);
    if (this->workerBusy>0)
      this->condDone.wait(lock);
  }

  void worker(int workerId) {
    Solution local;
    while (1) {
      {
        boost::mutex::scoped_lock lock(this->mutex);
        if (local<this->solution)
          this->solution = local;
        if (--this->workerBusy==0) {
          this->applySolution();
          this->workerBusy = this->nThreads;
          if (--this->params.num_trips>0) {
            ++this->params.trip;
            this->currentCab.store(0, boost::memory_order_release);
            this->solution.reset();
            this->condWork.notify_all();
          } else {
            this->condDone.notify_all();
            this->condWork.wait(lock);
          }
        }
        else {
          this->condWork.wait(lock);
        }
      }
      if (this->shutdown) break;

      local.reset();
      int index;
      while ((index=this->currentCab.fetch_add(CABS_PER_CHUNK, boost::memory_order_relaxed))<this->params.total_cabs) {
        this->updateCost(this->params.trip, index, std::min(index+CABS_PER_CHUNK, this->params.total_cabs), &local);
      }
    }
  }
  /*
  void update_avg_occupancy(){
    float average_onboard = 0 ;
    float average = 0 ;
    int counter = 0 ;
    for(int i=0 ; i<this->params.total_cabs ; i++){
      average += this->params.cabs[i]->occupancy ;
      if(this->params.cabs[i]->occupancy != 0){
        average_onboard += this->params.cabs[i]->occupancy ;
        this->stats.occ_counter++ ;
      }
    }
    this->stats.avg_occupancy += average_onboard ;
    this->stats.avg_total_occupancy += average ;
  }
  void update_avg_occupancy(CabTrip *cab_trips){
    this->params.trip = cab_trips ;
    float total = 0 ;
    for (int i=0;i<this->params.num_trips;i++){
      total += this->params.trip[i].dist_actual ;
    }
    this->stats.avg_total_occupancy = total/(this->stats.mean_odometer*this->params.total_cabs) ;
  }*/
  void update_avg_occupancy(CabTrip *cab_trips){
    //this->params.trip = cab_trips ;
    float total = 0 ;
    for (int i=0 ; i<this->params.total_cabs ; i++){
      total += this->params.cabs[i]->dist_revenue/1000000 ;
    }
    this->stats.avg_total_occupancy = total;
  }
 
  void update_mean_odometer(){
    float sum = 0 ;
    for(int i=0 ;i<this->params.total_cabs ;i++){
      sum += this->params.cabs[i]->odometer ;
    }
    this->stats.mean_odometer = (sum/1000000)/this->params.total_cabs ;
  }
  void update_waiting_times(CabTrip *cab_trips){
    this->params.trip = cab_trips ;
    float average_wait=0 ;
    float average_overall_wait=0 ;
    float average_travel_delay=0 ;
    //float default_speed = float(28000000)/3600 ;
    for (int i=0;i<this->params.num_trips;i++){
      if(this->params.trip[i].served){
        average_wait += //this->params.trip[i].pickup_wait + 
                          (this->params.trip[i].dist_delay/DEFAULT_SPEED) ; 
        if(this->params.trip[i].dist_delay > MAX_DELAY_DISTANCE){
          cout<<"NOOO !!!!"<<endl ;
        }     
        average_overall_wait += //this->params.trip[i].pickup_wait +
                                  (this->params.trip[i].dist_delay/DEFAULT_SPEED) ;
        average_travel_delay += (this->params.trip[i].dist_extra/DEFAULT_SPEED) ;
      }
      else{
         average_overall_wait += //this->params.trip[i].pickup_wait +
                                (this->params.trip[i].dist_delay/DEFAULT_SPEED) ;

      }
      
    }
    this->stats.avg_wait = average_wait/this->numServed ;
    this->stats.avg_wait_overall = average_overall_wait/this->params.num_trips ;
    this->stats.avg_travel_delay = average_travel_delay/this->numServed;
  }

  void update_pool_stats(){
    this->stats.average_total_walk /= this->numServed ;
    this->stats.average_pickup_walk /= this->numServed ;
    this->stats.average_dropoff_walk /= this->numServed ;
  }

  void updateCost(CabTrip *cab_trip, int start, int end, Solution *local)
  {
    int pick, drop, temp_cost ;
    int min_cost = INT_MAX, min_pick, min_drop ;
    int min_pickwalk, min_dropwalk ;
    int distance_to_pick = 0 ;
    Cab::ServingOrder best_serving_order ;
    int best_pick_wait=-1, pick_wait=0 ;
    int best_cab_wait=-1, cab_wait=0 ;
    int add_cost;

    for (int c=start; c<end; ++c) {
      this->params.cabs[c]->update_cab(this->params.city, cab_trip->trip->pickup_time, 
                              this->params.rebalancing_set);


      //Check if cab reach the passenger in best case
       CityMap::Location prev,des ;
      int dis=0 ;

      if(!this->params.cabs[c]->pathIntersection.empty()){
        //cout<<"NOT IDLE"<<endl; 

        if (this->params.cabs[c]->stops.size() == 1 
          && this->params.cabs[c]->stops.front().wait_stop == -1) {
          //printf("is an idle cab, shouldnt be\n");
        }
        if (this->params.cabs[c]->cur_pos == -1) {
          //printf("no cur_pos\n");
        }
        prev = this->params.city->intersections[this->params.cabs[c]->pathIntersection.at(this->params.cabs[c]->cur_index)] ;
        dis = this->params.cabs[c]->pathDistance.at(this->params.cabs[c]->cur_index)-this->params.cabs[c]->dist_driven;
      }
      else{
        //cout<<"IDLE"<<endl; 
        if (this->params.cabs[c]->cur_pos == -1) {
          //printf("no cur_pos\n");
        }
        prev = this->params.city->intersections[this->params.cabs[c]->cur_pos] ;
        dis=0 ;
      }
      //dis = this->pathDistance.at(this->cur_index)-this->dist_driven;
      des = this->params.city->intersections[cab_trip->pick_loc];
      double upperbound = ((MAX_DELAY_DISTANCE)/1000.0);//*0.000621371 ;
      
      if(this->params.city->distance(prev,des)+(dis*0.001) > upperbound){

      //if(this->params.city->distance(prev,des)+(dis*0.000621371) > upperbound){
        //this->stats.tester++ ;
        //cout<<"*"<<this->params.city->distance(prev,des)+(dis*0.000621371)<<", "<<upperbound<<endl ;
        //continue ;
      } else {
        //cout<<c<<" - reached here~"<<endl ;
      }



      //finding best pick up
      min_pick = 0, min_drop=0 , distance_to_pick = 0, min_cost = INT_MAX ;
      //min_pick = -1, min_drop=-1 , distance_to_pick = 0, min_cost = INT_MAX ;
      
      for(int j=0 ; j<this->params.city->poolNodes[cab_trip->pick_loc].size() ; j++){
        pick = this->params.city->poolNodes[cab_trip->pick_loc][j].intersection ;
        distance_to_pick = this->params.city->poolNodes[cab_trip->pick_loc][j].walk_distance;

        for(int k=0 ; k<this->params.city->poolNodes[cab_trip->drop_loc].size() ; k++){
          drop = this->params.city->poolNodes[cab_trip->drop_loc][k].intersection ;
          Cab::ServingOrder serving_order ;
          pick_wait = 0 ;
          cab_wait = 0 ;

          temp_cost = this->params.cabs[c]->cost_to_share(this->params.city, cab_trip,
                     &serving_order, pick, drop, float(distance_to_pick)/1.4 , pick_wait,
                     cab_wait) ;
          
          //cout<<"cost "<<temp_cost<<endl ;
          if(temp_cost!=-1 && temp_cost < min_cost){
            min_pick = pick ;
            min_drop = drop ;
            min_pickwalk = distance_to_pick ;
            min_dropwalk = this->params.city->poolNodes[cab_trip->drop_loc][k].walk_distance ;
            min_cost = temp_cost ;
            best_pick_wait = pick_wait ;
            best_cab_wait = cab_wait ;
            best_serving_order = serving_order ;
          }
        }
      }

      XPool best ;
      if(min_cost!=INT_MAX){
        best.best_pick = min_pick ;
        best.best_drop = min_drop ;
        best.cost = min_cost ;
        best.walktoPick = min_pickwalk ;
        best.walkfromDrop = min_dropwalk ;
        best.pick_wait = best_pick_wait ;
        best.cab_wait = best_cab_wait ;
        best.serving_order = best_serving_order ;
      }
      else{
        best.cost = -1 ;
      }

      //add_cost = this->params.cabs[i]->cost_to_share(this->params.city, cab_trip, &serving_order);

      if (best.cost!=-1 && best.serving_order.isValid() && (best.cost<local->min_cost)) {
        local->min_cost = best.cost ;
        local->min_cab_num = c ;
        local->min_serving_order = best.serving_order ;
        local->X = best ;

      }
    }
  }

  void applySolution() {
    if (this->solution.min_cab_num!=-1) {
      //cout<<":"<<this->solution.min_cab_num<<endl; 
      this->params.trip->distance = this->params.city->findShortestDistance(this->solution.X.best_pick, this->solution.X.best_drop) ; 
      this->params.cabs[this->solution.min_cab_num]->add_trip(this->params.city,
                                                              this->params.trip,
                                                              this->solution.X.best_pick,
                                                              this->solution.X.best_drop,
                                                              this->solution.X.cab_wait  ,
                                                              this->solution.min_serving_order);
      ////fc
      CityMap::Location prev,des ;
      int dis ;
      if(!this->params.cabs[this->solution.min_cab_num]->pathIntersection.empty()){
        prev = this->params.city->intersections[this->params.cabs[this->solution.min_cab_num]->pathIntersection.at(this->params.cabs[this->solution.min_cab_num]->cur_index)] ;
        dis = this->params.cabs[this->solution.min_cab_num]->pathDistance.at(this->params.cabs[this->solution.min_cab_num]->cur_index)-this->params.cabs[this->solution.min_cab_num]->dist_driven;
      }
      else{
        prev = this->params.city->intersections[this->params.cabs[this->solution.min_cab_num]->cur_pos] ;
        dis=0 ;
      }
      //dis = this->pathDistance.at(this->cur_index)-this->dist_driven;
      des = this->params.city->intersections[this->params.trip->pick_loc];
      double upperbound = ((MAX_DELAY_DISTANCE)/1000.0);//*0.000621371 ;
      //cout<<prev.lat<<","<<prev.lon<<endl ;
      //cout<<des.lat<<","<<des.lon<<endl ;
      //cout<<this->params.city->distance(prev,des)<<endl ;
      //cout<<"***"<<endl ;      


      if(this->params.city->distance(prev,des)+(dis*0.001) > upperbound){
      //if(this->params.city->distance(prev,des)+(dis*0.0000006) > upperbound){
        //cout<<"prev in applySolution "<<this->params.cabs[this->solution.min_cab_num]->cur_pos<<endl;
        //cout<<"des in applySolution "<<this->params.trip->pick_loc<<endl ;
        //cout<<"check:: " <<this->params.city->distance(prev,des)+(dis*0.0000006)<<" "<<upperbound<<endl;
        this->stats.tester++ ;
        //cout<<"values "<<this->params.city->distance(prev,des)*1609.34*1000<<endl ;
        //cout<<"the cab is : "<<this->solution.min_cab_num<<endl;
      }
      ////
      this->params.trip->served = true ;
      this->params.trip->pickup_wait += this->solution.X.pick_wait ;
      this->params.trip->dist_delay += this->params.trip->pickup_wait*DEFAULT_SPEED ;
      this->totalCost += this->solution.min_cost;
      time_t c_time = (time_t)this->params.trip->trip->pickup_time;
      struct tm *cur_time = localtime(&c_time);
      this->hourlyCost[cur_time->tm_hour] += this->solution.min_cost;
      ++this->numServed;

      this->stats.average_total_walk += this->solution.X.walktoPick + this->solution.X.walkfromDrop ;
      if(this->solution.X.walktoPick==0){
        this->stats.zerocounter += 1 ;
      }
      else{
        if(this->solution.X.walktoPick<=100){
          this->stats.under100++ ;
        } 
        else{
          if(this->solution.X.walktoPick<=200){
            this->stats.under200++ ;
          }
          else{
            if(this->solution.X.walktoPick<=300){
              this->stats.under300++ ;
            }
            else{
              this->stats.under400++ ;
            }
          }
        }
      }
      this->stats.average_pickup_walk += this->solution.X.walktoPick ;
      this->stats.average_dropoff_walk += this->solution.X.walkfromDrop ;
    }
    else{
      this->params.rebalancing_set.push(this->params.trip->pick_loc) ;
    }
  }

  void run(CabTrip *cab_trips, int cnt){
    this->params.trip = cab_trips ;
    this->params.num_trips = cnt ;
    time_t last_update = (time_t) this->params.trip->trip->pickup_time ;
    if (this->nThreads<2) {
      for (int i=0; i<cnt; ++i, ++this->params.trip) {
        cout<<"trip "<<i<<"--------------"<<endl ;
        this->solution.reset() ;
        this->updateCost(this->params.trip, 0, this->params.total_cabs, &this->solution);
        if(this->params.trip->trip->pickup_time > last_update + 30){
          last_update = this->params.trip->trip->pickup_time ;
          //this->params.rebalancing_set.clear();
          //stack<int>().swap(this->params.rebalancing_set); //clear the stack
          while(!this->params.rebalancing_set.empty()){
            this->params.rebalancing_set.pop();
          }
        }
        this->applySolution() ;
      }
      this->update_mean_odometer() ;
      this->update_pool_stats();
      cout<<"HHHHHHEY : "<<this->stats.tester<<endl; 
      return ;
    }
    this->dispatchToWorkers(cab_trips, cnt);
    this->waitForWorkers();
  }

  int count() const {
    return this->numServed;
  }
  
  float total_cost() const {
    return this->totalCost*1e-6f;
  }

  std::vector<float> hourly_cost() {
    std::vector<float> hourly(24, 0.0);
    for (int i=0; i<24; i++) {
      hourly[i] = this->hourlyCost[i]*1e-6f;
    }
    return hourly;
  }
public:
  Simulator::Statistics       stats;
  Simulator::Parameters       params;
  
private:

  boost::mutex                mutex;
  boost::condition_variable   condDone, condWork;
  boost::thread_group         workerThreads;
  Solution                    solution;
  bool                        shutdown;
  int                         nThreads;
  int                         workerBusy;
  boost::atomic<int>          currentCab;

  int                         numServed;
  int64_t                     totalCost;
  std::vector<int64_t>        hourlyCost;
};

#endif
