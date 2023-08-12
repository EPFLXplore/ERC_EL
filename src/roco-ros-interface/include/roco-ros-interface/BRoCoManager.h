#ifndef BROCO_MANAGER
#define BROCO_MANAGER

#include "BRoCoPublisher.h"
#include "BRoCoSubscriber.h"

#include "BRoCo/CanSocketDriver.h"

class BRoCoManager : public rclcpp::Node {
public:
  BRoCoManager();
  ~BRoCoManager();
private:
    CanSocketDriver* driver;
    // IOBus* bus;
    CANBus* bus;
    BRoCoPublisher* pub;
    BRoCoSubscriber* sub;
};

#endif /* BROCO_MANAGER */