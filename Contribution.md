# How to contribute to the VDA-5050-Connector
## Coding guideline
* We are using clang-format for our project formatting.
* The .clang-format file includes the format settings.
* Make sure you have clang-format server installed using the following command : 
```
sudo apt-get install clang-format
```
* Check for your IDE on how to enable the clang-format for the project.

## Documenting the code
We use Doxygen for generating the code documentation. Please refer to the documentation for syntax rules. Note that the first sentence (i.e. the words until the first full stop) of each documentation block should be a concise wrap-up of the following text.
* Variables are documented in the following line(s). The documentation starts with an additional tab followed by the sequence `/**<`:
```C++
    ros::Subscriber orderCancelSub;
        /**< ordinary order actions from order_daemon to action_daemon */

    ros::Publisher orderActionPub;
        /**< cancelled actions from action_daemon to order_daemon */
```

* Methods are documented in the above line(s). See the following example for the standards on the level of details as well as the indentations:
```C++
    /**
     * Read in the user-specified topic names. The user can specify names for
     * the topics that contain the needed information. For mapping the contents
     * to the topic names, this method scans the parameter server.
     * 
     * @param nh              Pointer to the ROS node handle.
     * @param paramTopicName  Name of the param family to scan through.
     * 
     * @return                Map from topic descriptor to user-defined topic
     *                        name.
     */
    std::map<std::string,std::string> ReadTopicParams(ros::NodeHandle *nh,
        std::string paramTopicName);
```
