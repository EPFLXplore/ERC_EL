#ifndef BROCO_MANAGER
#define BROCO_MANAGER

#include "BRoCoPublisher.h"
#include "BRoCoSubscriber.h"

#include "BRoCo/IODriver.h"
#include "BRoCo/IOBus.h"

class BRoCoManager : public rclcpp::Node {
public:
  BRoCoManager();

private:
    CanSocketDriver* driver;
    // IOBus* bus;
    std::shared_ptr<CANBus> bus;
    BRoCoPublisher* pub;
    BRoCoSubscriber* sub;
};

#endif /* BROCO_MANAGER */