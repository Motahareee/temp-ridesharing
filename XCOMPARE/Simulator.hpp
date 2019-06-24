#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include</anaconda/include/boost/timer/timer.hpp>
#include <boost/thread/thread.hpp>
#include <boost/atomic.hpp>
#include <time.h>
#include "Cab.hpp"
#include <queue> //moti
#include <vector>  //moti
#include "bimatch.hpp"
#include <typeinfo>
#include <stack>

#define DEFAULT_SPEED (17398400/3600)
#define MAX_DELAY_DISTANCE 1449866 //5 min
#define MAX_EXTRA_DISTANCE 1449866 //5 min

//#define MAX_DELAY_DISTANCE 3266666
//#define MAX_EXTRA_DISTANCE 3266666

class Simulator
{
public:
  static const int CABS_PER_CHUNK = 100;

  struct Parameters {
    CityMap *   city;
    Cab    **   cabs;
    int         total_cabs;
    int         num_share;
    CabTrip *   trip;
    int         num_trips;
    int         interval ;//moti (time interval we do biparted matching in)
    stack<int>  rebalancing_set ;
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
    int               pick_wait ;
    int               cab_wait;
    Cab::ServingOrder serving_order;
  };

  struct Solution {
    Solution() { this->reset(); }
    void reset() {
      this->cost = INT_MAX;
      this->cab_num = -1;
      this->serving_order.reset(); 
      this->X.reset();
    }

    bool operator<(const Solution &s) const {
      return (this->cost<s.cost) || (this->cost==s.cost && this->cab_num<s.cab_num);
    }

    int               cost;
    int               cab_num;
    Cab::ServingOrder serving_order;
    XPool             X;
  };
  struct ValidCab {
    int               cost;
    int               cab_num;
    Cab::ServingOrder serving_order;
    XPool             X;


    bool operator<(const ValidCab &s) const {
      return (this->cost<s.cost) || (this->cost==s.cost && this->cab_num<s.cab_num);
    }   
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
    //--this ;
    boost::mutex::scoped_lock lock(this->mutex);
    if (this->workerBusy>0){
      this->condDone.wait(lock);
    }
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
        //this->updateCost(this->params.trip, index, std::min(index+CABS_PER_CHUNK, this->params.total_cabs), &local); //moti
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
        counter++ ;
      }
    }
    if(counter != 0)
      this->stats.avg_occupancy += (average_onboard/counter)/2880 ;
    else
      this->stats.avg_occupancy += 0 ;
    this->stats.avg_total_occupancy += (average/this->params.total_cabs)/2880 ;
  }*/
  /*void update_avg_occupancy(){
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
  }*/
  void update_avg_occupancy(CabTrip *cab_trips){
    this->params.trip = cab_trips ;
    float total = 0 ;
    for (int i=0;i<this->params.num_trips;i++){
      total += this->params.trip[i].dist_actual ;
    }
    this->stats.avg_total_occupancy = total/(this->stats.mean_odometer*this->params.total_cabs) ;
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
    float default_speed = float(28000000)/3600 ;
    for (int i=0;i<this->params.num_trips;i++){
      if(this->params.trip[i].served){
        //average_wait += (this->params.trip[i].dist_delay/default_speed) ;      
        average_wait += //this->params.trip[i].pickup_wait ;+ 
                           (this->params.trip[i].dist_delay/DEFAULT_SPEED) ;      
        average_overall_wait += //this->params.trip[i].pickup_wait;// +
                                  (this->params.trip[i].dist_delay/DEFAULT_SPEED) ;
        //average_overall_wait += (this->params.trip[i].dist_delay/default_speed) ;
        average_travel_delay += (this->params.trip[i].dist_extra/DEFAULT_SPEED) ;
      }
      else{
        average_overall_wait += //this->params.trip[i].pickup_wait +
                                (this->params.trip[i].dist_delay/DEFAULT_SPEED) ;
        //average_overall_wait += (this->params.trip[i].dist_delay/default_speed) ;
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

  XPool XPool_cost(int c, CabTrip *cab_trip){
    int pick, drop, temp_cost ;
    int min_cost , min_pick, min_drop ;
    int min_pickwalk, min_dropwalk ;
    int distance_to_pick = 0 , pick_wait = 0, best_pick_wait = 0;
    Cab::ServingOrder best_serving_order ;
    min_pick = 0, min_drop=0 , distance_to_pick = 0, min_cost = INT_MAX ;
    int best_cab_wait=-1, cab_wait=0 ;

  //Check if cab reach the passenger in best case
    CityMap::Location src,des ;
    if(!this->params.cabs[c]->pathIntersection.empty()){
      src = this->params.city->intersections[this->params.cabs[c]->pathIntersection.at(this->params.cabs[c]->cur_index)] ;
    }
    else{
      src = this->params.city->intersections[this->params.cabs[c]->cur_pos] ;
    }

    des = this->params.city->intersections[cab_trip->pick_loc];
    double upperbound = (MAX_DELAY_DISTANCE/1000 + 400)*0.0006 ;
    //double upperbound = (MAX_DELAY_DISTANCE + 2*400000) ;
    
    if(this->params.city->distance(src,des) > upperbound){
      //int a = 0 ;
      //continue ;
    }

    for(int j=0 ; j<this->params.city->poolNodes[cab_trip->pick_loc].size() ; j++){
      pick = this->params.city->poolNodes[cab_trip->pick_loc][j].intersection ;
      distance_to_pick = this->params.city->poolNodes[cab_trip->pick_loc][j].walk_distance;

      for(int k=0 ; k<this->params.city->poolNodes[cab_trip->drop_loc].size() ; k++){
        drop = this->params.city->poolNodes[cab_trip->drop_loc][k].intersection ;
        Cab::ServingOrder serving_order ;
        pick_wait = 0 ;
        temp_cost = this->params.cabs[c]->cost_to_share(this->params.city, cab_trip,
                     &serving_order, pick, drop, float(distance_to_pick)/1.5, pick_wait,
                      cab_wait) ;
        
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
    return best;
  }
  void updateCost(const vector<CabTrip*> &group, int start, int end, std::vector<ValidCab> &Solutions,
   vector<int> &pointer_to_solution, std::vector<int> &Sol_Cabnum, time_t time, bool rebalancing){
    int add_cost;
    ValidCab temp ;
    XPool best_pair ;
    for(int t=0 ; t<group.size() ; t++){
      //cout<<"trip number:"<<t<<" out of overal "<<group.size()<<"trips"<<endl ;
      pointer_to_solution[t] = Solutions.size() ;
      //if(rebalancing && group[t]->served) continue ;
      Cab::ServingOrder serving_order ;    
      for (int c=start ; c<end ; c++) { 
        this->params.cabs[c]->update_cab(this->params.city, time, this->params.rebalancing_set) ;
        best_pair = XPool_cost(c, group[t]) ;
        
        if (best_pair.cost!=-1 && best_pair.serving_order.isValid()) {
          temp.cost = best_pair.cost ;
          temp.cab_num = c ;
          temp.serving_order = best_pair.serving_order ;
          temp.X = best_pair ;
          Solutions.push_back(temp) ;
          Sol_Cabnum.push_back(c) ;
        }
      }
    }
  }

  void tripAssignment(std::vector<int> &pointers, std::vector<ValidCab> &Solutions, 
      vector<CabTrip*> &group, std::vector<int> &Sol_Cabnum){
    
    //get unique list of trips
    std::vector<int> cabs ;
    for(int i=0 ; i<Sol_Cabnum.size() ; i++) cabs.push_back(Sol_Cabnum[i]) ;
    std::sort(cabs.begin(),cabs.end()) ;
    std::vector<int>::iterator it;
    it = std::unique (cabs.begin(), cabs.end()) ; 
    cabs.resize( std::distance(cabs.begin(),it) );
    
    if(cabs.size()== 0 ) {
      return ;
    }
    std::map <int,int> assignment ;
    bimatch b(pointers, Sol_Cabnum, cabs) ;
    b.maxBPM(assignment) ;
    
    int point,size ;
    for(auto const& [cab, trip] : assignment){ 
      if(trip!=-1){
        point = pointers[trip] ;
        if(trip == pointers.size()-1)
          size = Solutions.size()-pointers[trip] ;
        else
          size = pointers[trip+1] - pointers[trip] ;

        for(int i=0 ; i<size ; i++){
          if(Sol_Cabnum[point+i] == cab){
              this->solution.reset() ;
              this->solution.cost = Solutions[point+i].cost ;
              this->solution.cab_num = Solutions[point+i].cab_num ;
              this->solution.serving_order = Solutions[point+i].serving_order ;
              this->solution.X = Solutions[point+i].X ;
              this->applySolution_assignment(group[trip]) ;
          }
        }
      }  
    }
  }

  void applySolution_assignment(CabTrip *trip_) {
    if (this->solution.cab_num!=-1) {;
      trip_->distance = this->params.city->findShortestDistance(this->solution.X.best_pick, this->solution.X.best_drop) ; 
      this->params.cabs[this->solution.cab_num]->add_trip(this->params.city,
                                                          trip_,
                                                          this->solution.X.best_pick,
                                                          this->solution.X.best_drop,
                                                          this->solution.X.cab_wait,
                                                          this->solution.serving_order);
      
      trip_->served = true ;
      trip_->pickup_wait += this->solution.X.pick_wait ;
      this->params.trip->dist_delay += this->params.trip->pickup_wait*DEFAULT_SPEED ;
      this->totalCost += this->solution.cost ;
      time_t c_time = (time_t)trip_->trip->pickup_time ;
      struct tm *cur_time = localtime(&c_time) ;
      this->hourlyCost[cur_time->tm_hour] += this->solution.cost ; 
      ++this->numServed;

      this->stats.average_total_walk += this->solution.X.walktoPick + this->solution.X.walkfromDrop ;
      //cout<<"----"<<this->solution.X.walktoPick<<endl ;
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
}

  void applySolution() {
    if (this->solution.cab_num!=-1) {
      this->params.cabs[this->solution.cab_num]->add_trip(this->params.city,
                                                          this->params.trip,
                                                          this->params.trip->pick_loc,
                                                          this->params.trip->drop_loc,
                                                          this->solution.X.cab_wait,
                                                          this->solution.serving_order);
      this->totalCost += this->solution.cost;
      
      time_t c_time = (time_t)this->params.trip->trip->pickup_time;
      struct tm *cur_time = localtime(&c_time);
      this->hourlyCost[cur_time->tm_hour] += this->solution.cost;
      ++this->numServed;
    }
  }

  void run(CabTrip *cab_trips, int cnt, int inter){
    this->params.trip = cab_trips ;
    this->params.num_trips = cnt ;
    this->params.interval = inter ;
    
    if (this->nThreads<2) {
      int max_cost = 0 ,time, counter = 0 ;
      time_t leader_pu_time = (time_t) this->params.trip->trip->pickup_time ; //leader pick up time  
      boost::timer::cpu_timer timer ;
      std::vector<CabTrip*> group ;
      while(counter<cnt){
        timer.start();
        cout<<"counter "<<counter<<"and total is "<<cnt<<endl ;
        while((counter<cnt) && 
          ((time_t) this->params.trip->trip->pickup_time - leader_pu_time < this->params.interval)){
        	counter++ ;
          this->params.trip->dist_delay += (leader_pu_time + this->params.interval - 
              this->params.trip->trip->pickup_time) * DEFAULT_SPEED ;
        	group.push_back(this->params.trip);
        	++this->params.trip ;
        }
        //cout<<"a"<<endl;

        leader_pu_time += this->params.interval ;
        CabTrip* remember_pointer = this->params.trip ; //check if it is true ghablesh ++ bud !!
        std::vector<int> pointer_to_solution(group.size(), -1) ;
        std::vector<ValidCab> Solutions ;
        std::vector<int> Sol_Cabnum ;
        //cout<<"b"<<endl;
        
        this->updateCost(group, 0, this->params.total_cabs, Solutions, pointer_to_solution , Sol_Cabnum, leader_pu_time, false) ;
        //cout<<"c"<<endl;
        
        while(!this->params.rebalancing_set.empty()){
            this->params.rebalancing_set.pop();
        }

        tripAssignment(pointer_to_solution,Solutions, group, Sol_Cabnum) ;
        
        //rebalancing
        //std::vector<int> pointer_to_solution1(group.size(), -1) ;
        //std::vector<ValidCab> Solutions1 ;
        //std::vector<int> Sol_Cabnum1 ;
        //this->updateCost(group, 0, this->params.total_cabs, Solutions1, pointer_to_solution1 , Sol_Cabnum1, leader_pu_time, true) ;
        //tripAssignment(pointer_to_solution1,Solutions1, group, Sol_Cabnum1) ;
        
        this->params.trip = remember_pointer ;

        //reconsidering unexpired unserved trips
        std::vector<CabTrip*> unserved_trips ;
        for(int i=0 ; i<group.size() ; i++)
          if(!group[i]->served){
            //if(group[i]->dist_delay+(inter*DEFAULT_SPEED) < MAX_DELAY_DISTANCE ){
            if((group[i]->pickup_wait+inter)*DEFAULT_SPEED < MAX_DELAY_DISTANCE ){
              //group[i]->dist_delay += this->params.interval * DEFAULT_SPEED ;
              group[i]->pickup_wait += this->params.interval ;
              unserved_trips.push_back(group[i]) ;
            }
            else{
              this->params.rebalancing_set.push(group[i]->pick_loc);
            }
          }
        group = unserved_trips ;
        timer.stop();
        //update statistics
        this->stats.avg_computation_time += timer.elapsed().wall*1e-9/2880 ;
        //this->update_avg_occupancy() ;
        //clearing the rebalancing stack
        
      }
      this->update_mean_odometer() ;
      this->update_pool_stats();
      //this->update_avg_occupancy() ;
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
    std::vector<float> hourly(24, 0.0) ;
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
//----------------------------------------------------------------------------------------
