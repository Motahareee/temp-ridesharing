//#include "tsharesim.hpp"
#include "CityMap.hpp"
#include <stdio.h>

//#include "tsharesim/includes.h"
//#include "tsharesim/taxitree.h"
//#include "tsharesim/customerqueue.h"
//#include "tsharesim/vertex.h"
//#include "tsharesim/shortestPath.h"
//#include "tsharesim/taxilinklist.h"
//#include "tsharesim/oneTaxiPath.h"

TSHARESIM_BEGIN

//queue of customer requests waiting to be processed
static customerqueue customers;

//set of taxis; hopefully 50000 >= MAX_PASSENGERS
static taxiTree taxi[50000];

//array of vertices in the graph
static vector<vertex *> vertices;

//shortest path interface
static ShortestPath *shortestPath;

static CityMap * city;
static ::Cab  ** cabs;

void init(CityMap *cm, const char *edgeFile, ::Cab **pcabs, int total_cabs, int num_share)
{
  tsharesim::city = cm;
  tsharesim::cabs = pcabs;
  MAX_TAXIS = total_cabs;
  MAX_PASSENGERS = num_share;
  
  for(int i = 0; i < city->numIntersections(); ++i) {
    vertex *newVertex = new vertex;
    newVertex->x = city->getIntersection(i).lon;
    newVertex->y = city->getIntersection(i).lat;
    newVertex->id = i;		
    tsharesim::vertices.push_back(newVertex);
  }

  for(int i = 0; i < city->numStreets(); ++i) {
    const CityMap::Street &street = city->getStreet(i);
    tsharesim::vertices[street.first]->neighbors.push_back(tsharesim::vertices[street.second]);
    // tsharesim::vertices[street.second]->neighbors.push_back(tsharesim::vertices[street.first]);
  }

  // CityMap::Path path;
  // fprintf(stderr, ">>>>>>>> %d\n",
  //         city->computeShortestPath(6202, 3354, path));
  // exit(0);

  tsharesim::shortestPath = new ShortestPath(tsharesim::vertices, edgeFile);

  for(int i = 0; i < MAX_TAXIS; ++i) {
    //select a random vertex to place this taxi
    int randIndex = rand() % vertices.size();
    tsharesim::taxi[i].setInitialPosition(tsharesim::shortestPath, tsharesim::vertices[randIndex]);
  }
}

Cab::Cab(int num_share, int cap, int id)
    : ::Cab(num_share, cap, id)
{
}

int Cab::update_cab(CityMap *city, int time)
{
  // If this is the first update ever, making it like we're not moving
  if (this->last_update<0)
    this->last_update = time;

  // Update the timestamps
  taxiTree *self = &tsharesim::taxi[this->id];
  int time_passed = time - this->last_update;
  this->last_update = time;
  if (this->lastValue>-2)
    self->cancel();
  while (time_passed-->0)
    self->updateLocation();
}

int Cab::cost_to_share(CityMap *city, const CabTrip *trip, ServingOrder *so)
{
  taxiTree *self = &tsharesim::taxi[this->id];
  // if (self->passengers+trip->trip->passengers<=this->capacity) {
  if (self->passengers<this->capacity) {
    so->reset(0);
    this->lastValue = self->value(tsharesim::vertices[trip->pick_loc],
                                  tsharesim::vertices[trip->drop_loc])*1e3;
  }
  else {
    so->reset(-1);
    this->lastValue = -2;
  }
  return this->lastValue;
}

void Cab::add_trip(CityMap *city, CabTrip *trip, const ServingOrder &order)
{
  taxiTree *self = &tsharesim::taxi[this->id];
  // self->passengers += trip->trip->passengers;
  self->passengers += 1;
  self->push();
}

TSHARESIM_END
