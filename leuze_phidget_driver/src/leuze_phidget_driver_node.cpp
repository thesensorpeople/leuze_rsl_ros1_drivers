#if defined(RSL200)
    #include "leuze_phidget_driver_rsl200.hpp"
#elif defined(RSL400)
    #include "leuze_phidget_driver_rsl400.hpp"
#endif

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "leuze_phidgets_driver", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    sleep(3); //Wait for phidgets_ik_node to come up and spawn its topics
    
#if defined(RSL200)
    //std::cout << "RSL200" << std::endl;
    LeuzePhidgetDriver *driver = new LeuzePhidgetDriverRsl200(&n);
#elif defined(RSL400)
    //std::cout << "RSL400" << std::endl;
    LeuzePhidgetDriver *driver = new LeuzePhidgetDriverRsl400(&n);
#endif

    ros::spin();
    return 0;
}
