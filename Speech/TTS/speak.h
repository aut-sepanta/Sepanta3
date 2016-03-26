#include "ros/ros.h"
#include <string>
#include <stdlib.h>
using namespace std; 
class speak{
public:
static void say(char* sentence)
{
	system((string("rosrun sound_play say.py \"")+string(sentence)+string("\"")).c_str());
}
};

