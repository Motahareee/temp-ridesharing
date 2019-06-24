#include <iostream>
#include <time.h>
using namespace std ;

#include <iostream>
#include <string>
#include <cstring>
#include <ctime>

class HistoryCache
{
public:
   static std::string getTimeStamp(time_t epochTime, const char* format = "%Y-%m-%d %H:%M:%S")
   {
      char timestamp[64] = {0};
      strftime(timestamp, sizeof(timestamp), format, localtime(&epochTime));
      return timestamp;
   }

   static time_t convertTimeToEpoch(const char* theTime, const char* format = "%Y-%m-%d %H:%M:%S")
   {
      std::tm tmTime;
      memset(&tmTime, 0, sizeof(tmTime));
      strptime(theTime, format, &tmTime);
      return mktime(&tmTime);
   }
};

int main()
{
   // get current epoch time
   const time_t curTime = time(0);

   // convert current time to a string
   std::string curTimeStr = HistoryCache::getTimeStamp(curTime);

   // convert string time to an epoch time
   const time_t curTime2 = HistoryCache::convertTimeToEpoch(curTimeStr.c_str());

   // display results
   std::cout << "Epoch Time: " << curTime    << "\n"
             << "As string : " << curTimeStr << "\n"
             << "Epoch Time: " << curTime2
             << std::endl;
}