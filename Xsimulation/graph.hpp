#ifndef GRAPH
#define GRAPH

#include <iostream>
#include <vector>
//#include <unordered_set> 
#include <fstream>
#include <set>
#include <string>
using namespace std ;

struct Vertex
{
  int vertex ;
  float coordinates[2] = {-1,-1} ;
  std::vector <int> neighbours ;
  std::vector <float> weight ;
  set <int>  path_parents ; 
};
class Graph_
{
public:
  int V ;
  int E ;
  std::vector<Vertex> graph ;
  struct poolnodes {
    int intersection = -1 ;
    float walk_distance = -1 ;
  };
  Graph_ (string inputaddress){
    ifstream infile ;
    infile.open(inputaddress);
    infile >> V >> E ;

      for (int i=0 ; i < V ; i++){
        Vertex temp ;
        temp.vertex = i ;
        infile >> temp.coordinates[0] >> temp.coordinates[1] ;
        graph.push_back(temp) ;
      }
      int from, to;
      float _w_ ;
      
      std::vector< std::vector<float> > weight ;
      for(int i=0 ; i<V ; i++){
        std::vector<float> temp ;
        for(int j=0 ; j<V ; j++){
          temp.push_back(-1) ;
        }
        weight.push_back(temp) ;
      }

      for (int i=0 ; i < E ; i++){
        infile >> from >> to >> _w_ ;
        weight[from][to] = _w_ ;
      }

      for(int i=0 ; i<V ; i++){
        for(int j=0 ; j<V ; j++){
          if(weight[i][j]!=-1 && weight[j][i]==-1){
            weight[j][i] = weight[i][j] ;
          }
          if(weight[i][j]==-1 && weight[j][i]!=-1){
            weight[i][j] = weight[j][i] ;
          }
        }
      }

      for(int i=0 ; i<V ; i++){
        for(int j=0 ; j<V ; j++){
          if(weight[i][j]!=-1){
            graph[i].neighbours.push_back(j) ;
            graph[i].weight.push_back(weight[i][j]) ;
          }
        }
      }
      infile.close() ;
  }

  int minDistance(std::vector<float> &dist, std::vector<bool> &visited){ 
      int min = INT_MAX ;
      int min_index = -1 ;
      for (int v = 0; v < this->V; v++){ 
        if (visited[v] == false && dist[v] < min){ 
            min = dist[v] ;
            min_index = v ; 
        }
      }
      return min_index; 
  } 
  std::vector<poolnodes> Dijkstra_shortestpath(int source, float walk_threshold, float start_dis){
    std::vector<poolnodes> result ;
    std::vector<float> dist;
    std::vector<bool> visited ;
    int v ;
    for (int i = 0; i < V; i++){
        dist.push_back(INT_MAX) ;
        visited.push_back(false) ;
    }
    
    dist[source] = start_dis ; 
    for (int count = 0; count < V-1; count++){ 
      int u = minDistance(dist, visited) ;
      if(dist[u] <= walk_threshold){
        poolnodes temp ;
        temp.intersection = u ;
        temp.walk_distance = dist[u] ;
        result.push_back(temp) ;
      }
      else{
        return (result) ;
      }

      visited[u] = true ;
      for (int i = 0 ; i < graph[u].neighbours.size() ; i++){
        v = graph[u].neighbours[i] ;
        if (!visited[v] && dist[u] + graph[u].weight[i] < dist[v]){
              dist[v] = dist[u] + graph[u].weight[i] ; 
        }
      }
    }
    return result ;
  }
};
#endif
