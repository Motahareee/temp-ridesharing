#include <cstdio>
#include <cstring>
#include <ctime>
#include <boost/timer/timer.hpp>
#include <numeric>
#include <iostream>
#include <fstream>
#include "CityMap.hpp"
#include "KdTrip.hpp"
#include "Cab.hpp"
#include "Simulator.hpp"
// #include "tsharesim.hpp"

int main(int argc, char **argv){
  int nThreads = boost::thread::hardware_concurrency()/2;
  if (argc<7) {
    fprintf(stderr, 
            "Usage: %s  <IN_TAXI_TRIP_RECORDS_FILE> <manhattan_with_weights.txt> <paths.bin> <EDGES.GRA> <NUM_SHARE> <NUM_CABS> [NUM_TRIPS=-1/all] [NUM_THREADS=%d]\n", 
            argv[0], nThreads);
    return -1;
  }
  unsigned randomSeed = 0;//std::time(0);
  srandom(randomSeed);
  fprintf(stderr, "Random Seed: %d\n", randomSeed);
  boost::iostreams::mapped_file mfile(std::string(argv[1]),
                                      boost::iostreams::mapped_file::priv);
  int nTrip = mfile.size()/sizeof(KdTrip::Trip);
  KdTrip::Trip *trips = (KdTrip::Trip*)mfile.const_data();
  CityMap *city = new CityMap(argv[2]);
  if (city->loadPaths(argv[3]))
    fprintf(stderr, "Loaded precomputed shortest paths.\n");
  else {
    fprintf(stderr, "First time running or invalid precomputed paths. Generating shortest paths...\n");
    city->buildAllShortestPaths();
    city->savePaths(argv[3]);
  }
  // exit(0);
  int num_share  = atoi(argv[5]);
  int total_cabs = atoi(argv[6]);
  int total_trips;
  if (argc > 7) {
    total_trips= atoi(argv[7]);
    if (total_trips<0)
      total_trips = nTrip;
  } else {
    total_trips = nTrip;
  }
  total_trips = std::min(total_trips, nTrip);

  time_t rawtime;
  struct tm * timeinfo;
  if (argc>11) {
  // get current timeinfo and modify it 
    time ( &rawtime );
    timeinfo = localtime ( &rawtime );
    timeinfo->tm_year = atoi(argv[8]) - 1900;
    timeinfo->tm_mon = atoi(argv[9]) - 1;
    timeinfo->tm_mday = atoi(argv[10]);
    timeinfo->tm_hour =atoi(argv[11]);
    timeinfo->tm_min = 0;
    timeinfo->tm_sec = 0;
  
    rawtime = mktime ( timeinfo );

  } else {
    rawtime = -1;
  }
  // Generate and sort cab trips using result obtained above.
  CabTripList cab_trips;
  const float min_speed = 3.f;
  gen_cabtrips(city, trips, nTrip, total_trips, cab_trips, rawtime, 3600,  min_speed);
  total_trips = std::min(total_trips, (int)cab_trips.size());

  if (argc>8)
    nThreads = std::min(atoi(argv[8]), (int)boost::thread::hardware_concurrency());
  fprintf(stderr, "Using %d threads\n", nThreads);
  Simulator *sim = new Simulator(nThreads);

  //load driver data
  std::vector<int> taxi_initialloc ;
  fstream file;
  file.open("/home/Motahare/Documents/Github/ride-sharing/Data/driverData-RS-1000.csv");
  string line ;
  while (getline( file, line,'\n')){
    istringstream templine(line) ;
    string data ;
    getline(templine, data,',') ;//index
    getline(templine, data,',') ;//index of cab
    taxi_initialloc.push_back(std::atoi(data.c_str())) ;
  }
 
  // Generate cabs.
  std::vector<Cab*> cabs(total_cabs);
  for (int i=0; i<total_cabs; i++) {
    Cab *cab = new Cab(num_share);
    cab->id = i;
    cab->cur_pos = taxi_initialloc[i]-1 ;
    cab->gen_stop_idle(city); 
    cabs[i] = cab; 
  }
  fprintf(stderr, "%d trips generated\n", total_trips);
  fprintf(stderr, "%lu cabs generated\n", cabs.size());

  if (!boost::filesystem::exists(argv[4])) {
    fprintf(stderr, "Creating %s...\n", argv[4]);
    city->createEdgesFile(argv[4]);
  }
  // tsharesim::init(city, argv[4], &cabs[0], cabs.size(), num_share);
  // tsharesim::init(city, argv[4], &cabs[0], cabs.size(), num_share);
  sim->init(city, &cabs[0], cabs.size(), num_share);

  // Simulating ride-sharing
  float add_cost, total_cost = 0;
  int count = 0;
  std::vector<float> hourly_cost;
  boost::timer::cpu_timer timer;
  timer.start();
  sim->init(city, &cabs[0], cabs.size(), num_share);
  sim->run(&cab_trips[0], cab_trips.size());
  sim->update_waiting_times(&cab_trips[0]) ;
  sim->update_avg_occupancy(&cab_trips[0]) ;  
  count = sim->count();
  total_cost = sim->total_cost();
  hourly_cost = sim->hourly_cost();

  // Make incomplete trips complete.
  float total_dist_revenue = 0;
  for (int j=0; j<cabs.size(); j++) {
    // allowing the cab to travel 200 miles more
    cabs[j]->update_cab(city, cabs[j]->last_update+200e6/cabs[j]->speed,
                         sim->params.rebalancing_set);
    total_dist_revenue += cabs[j]->dist_revenue*1e-6f;
  }
  timer.stop();
  fprintf(stderr, " %f seconds took for ridesharing simulation\n", timer.elapsed().wall*1e-9);

  // // For debugging purposes
  // for (int i=0; i<total_trips; i++) {
  //   if (cab_trips[i].is_picked()) {
  //     if (!cab_trips[i].is_complete()) {
  //       fprintf(stderr, "FOUND AN INCOMPLETE TRIP %d %d %d %d %d %.9f\n", i, cab_trips[i].picked_id, cab_trips[i].shares, cab_trips[i].stops, cab_trips[i].speed, cab_trips[i].trip->distance*0.01);
  //       int id = cab_trips[i].picked_id;
  //       Cab::StopList::iterator it = cabs[id].stops.begin();
  //       fprintf(stderr, "CABS: %d %d %d %d\n", cabs[id].odometer, cabs[id].dist_driven, cabs[id].speed, cabs[id].cur_pos);
  //       while (it!=cabs[id].stops.end()) {
  //         fprintf(stderr, "STOP: %d %d %d %d %.9f\n", it->intersection, it->passengers, it->trip->pick_loc, it->trip->drop_loc, it->distance*0.01);
  //         ++it;
  //       }

  //       fprintf(stderr, "PUSH 10000s more\n");
  //       cabs[id].update_cab(city, cabs[id].last_update+10000);
  //       fprintf(stderr, "CABS: %d %d %d %d\n", cabs[id].odometer, cabs[id].dist_driven, cabs[id].speed, cabs[id].cur_pos);
  //       while (it!=cabs[id].stops.end()) {
  //         fprintf(stderr, "STOP: %d %d %d %d %.9f\n", it->intersection, it->passengers, it->trip->pick_loc, it->trip->drop_loc, it->distance*0.01);
  //         ++it;
  //       }

  //       exit(0);
  //     }
  //   }
  // }  

  const int MAX_SHARE = num_share;
  const int MAX_STOP  = num_share*3;
  std::vector<int> share(MAX_SHARE+1);
  std::vector<int> stop(MAX_STOP+1);
  int val, val2;
  for (int i=0; i<total_trips; i++) {

    if (!cab_trips[i].is_complete() && cab_trips[i].is_picked()) {
      fprintf(stderr, "-------------> WARNING: not complete trips:%d\n", i);
      exit(EXIT_FAILURE);
    } else if (!cab_trips[i].is_picked()) {

      continue;
    }

    val = cab_trips[i].shares;
    val2 = cab_trips[i].stops;

    if (val<=MAX_SHARE)
      ++share[val];
    else
      fprintf(stderr, "Unexpected sharing number! cab %d, %d\n", i, val);

    if (val2<=MAX_STOP)
      stop[val2]++;
    else {
      if (val2<=num_share*2)
        stop[MAX_STOP]++;
      else
        fprintf(stderr, "Unexpected stop number! cab %d, %d\n", i, val2);
    }
  }

  int total_complete_trips = std::accumulate(share.begin(), share.end(), 0);
  if (total_complete_trips != count) {
    fprintf(stderr, "The number of complete trips does not match that of pick-uped trips %d %d\n", total_complete_trips, count);
    exit(EXIT_FAILURE);
  }
 
  fprintf(stderr, "%f miles cost and %f miles with revenue\n", total_cost, total_dist_revenue);

  fprintf(stderr, "%d trips served\n", count);

  fprintf(stderr, "%f %% are not shared.\n", (float)share[0]/count*100);
  for (int i=1; i<=MAX_SHARE; ++i)
    fprintf(stderr, "%f %% are shared with %d trips\n", (float)share[i]/count*100, i);

  fprintf(stderr, "%f %% no extra stops.\n", (float)stop[0]/count*100);
  for (int i=1; i<MAX_STOP; ++i)
    fprintf(stderr, "%f %% %d extra stops.\n", (float)stop[i]/count*100, i);
  fprintf(stderr, "%f %% %d extra stops or more.\n", (float)stop[MAX_STOP]/count*100, MAX_STOP);

  float avg_trip = 0, avg_actual_trip = 0, avg_extra = 0, original, extra;
  int NUM_BINS = 11, bin;
  float BIN = 0.2;
  std::vector<int> distribution_extra(NUM_BINS, 0);
  std::vector<float> hourly_orig_cost(24, 0);
  time_t c_time;
  struct tm * cur_time;
  int pf = 0;
  uint64_t total_wait_time = 0;
  uint32_t total_wait_trip = 0;
  for (int i=0; i<total_trips; i++) {
    if (cab_trips[i].is_complete()) {
      //cout<<i<<endl;

      ++total_wait_trip;
      total_wait_time += cab_trips[i].wait_time;
      avg_actual_trip += cab_trips[i].actual_dist();
      extra = cab_trips[i].extra_dist();
      original = cab_trips[i].actual_dist()-extra;
      if (cab_trips[i].extra_dist()>MAX_EXTRA_DISTANCE+0.1)
        fprintf(stderr, "extra distance EXCEEDED: trip=%d extra=%.9f original=%.9f\n", i, extra, original);
      bin = (int)(extra/BIN);
      if (bin < 1) {
        bin = 0;
      }
      if (bin<NUM_BINS)
      	distribution_extra[bin]++;
      c_time = (time_t)cab_trips[i].trip->pickup_time;
      cur_time = localtime(&c_time);
      hourly_orig_cost[cur_time->tm_hour]+=original;
      avg_extra += extra;
      avg_trip += original;
     
      if (extra < 0.1)
        pf++;
    } else if (cab_trips[i].actual_dist() == 0 && !cab_trips[i].is_complete() && cab_trips[i].is_picked()) {
      fprintf(stderr, "not complete trip = %d\n", i);
    }
  }
  
  float total = avg_trip;
  avg_trip/=count;
  avg_actual_trip/=count;
  avg_extra/=count;
  
  fprintf(stderr, "average trip distance = %f, average extra distance = %f\n", avg_trip, avg_extra);
  fprintf(stderr, "saving is %f %%\n", (total - total_dist_revenue)/total*100);
  fprintf(stderr, "average occupancy = %f \n", sim->stats.avg_occupancy/sim->stats.occ_counter);
  fprintf(stderr, "average total occupancy = %f \n", sim->stats.avg_total_occupancy);///(2880*total_cabs));
  //fprintf(stderr, "average occupancy = %f \n", sim->stats.avg_occupancy);
  //fprintf(stderr, "average total occupancy = %f \n", sim->stats.avg_total_occupancy);
  fprintf(stderr, "mean distance traveled = %f \n", sim->stats.mean_odometer);
  fprintf(stderr, "average computation time per iteration = %f \n", sim->stats.avg_computation_time);
  fprintf(stderr, "average waiting time w/o(before pickup_time) = %f \n", sim->stats.avg_wait);
  fprintf(stderr, "average waiting time all(before pickup_time) = %f \n", sim->stats.avg_wait_overall);
  fprintf(stderr, "average in car delay = %f \n", sim->stats.avg_travel_delay);
  fprintf(stderr, "average total travel delay = %f \n", sim->stats.avg_travel_delay+sim->stats.avg_wait);
  fprintf(stderr, "---- Xpress Pool --------------------------------\n");
  fprintf(stderr, "average total walk = %f \n", sim->stats.average_total_walk);
  fprintf(stderr, "average pick up walk = %f \n", sim->stats.average_pickup_walk);
  fprintf(stderr, "average drop off walk = %f \n", sim->stats.average_dropoff_walk);
  fprintf(stderr, "-------------------------------------------------\n");
  fprintf(stderr, "zero walk = %d \n", sim->stats.zerocounter);
  fprintf(stderr, "lower than 100 walk = %d \n", sim->stats.under100);
  fprintf(stderr, "lower than 200 walk = %d \n", sim->stats.under200);
  fprintf(stderr, "lower than 300 walk = %d \n", sim->stats.under300);
  fprintf(stderr, "up to 400 walk = %d \n", sim->stats.under400);
  fprintf(stderr, "-------------------------------------------------\n");



  


  // Print to stdout, so we can pipe it differently from stderr
  std::cout << total_trips << " " << count << " "  << total_dist_revenue << " " << total
     << " " << avg_trip << " " << avg_extra;
  for (int i=0; i<=MAX_STOP; ++i)
    std::cout << " " << stop[i]; 

  if (MAX_SHARE < 5) {
    for (int i=0; i < 10-MAX_SHARE*2; i++){
      std::cout << " " << 0.0;
    }
  }

  for (int i=0; i<=MAX_SHARE; i++)
    std::cout << " " << share[i];

  if (MAX_SHARE < 5) {
    for (int i=0; i < 5-MAX_SHARE; i++){
      std::cout << " " << 0.0;
    }
  }

  //added my me
  //cout<<" !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl ;
  for(int i=0 ; i<share.size() ; i++) cout<<share[i]<<" " ; 
  cout<<endl ;
  int count2 = 0;
  for (int i=0; i<NUM_BINS; i++){
    //std::cout << " " << distribution_extra[i];
    count2 += distribution_extra[i];
  }

  if (count != count2) {
    fprintf(stderr, "count != count2 %d != %d %d\n", count, count2);
  }


  for (int i=0; i<24; i++) {
    std::cout << " " << hourly_cost[i];

  }

  for (int i=0; i<24; i++) {
    std::cout << " " << hourly_orig_cost[i];

  }
  std::cout << std::endl;

  fprintf(stderr, "serving rate = %f %%\n", (float)count/total_trips*100);
  cout<<"::"<<total_wait_trip<<endl ;
  fprintf(stderr, "average wait time for %u trips is %lu seconds\n", total_wait_trip, total_wait_time/total_wait_trip);
  return 0;
}
 
