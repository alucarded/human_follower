#include "HumanFollower.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_follower");

  HumanFollower::getInstance()->navigate();

  return 0;
}
