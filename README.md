cfe_sbn_bridge_plugin
======================

This bridge allows to share information between cFE and ROS2 systems
using SBN. 

To run:

```
ros2 launch cfe_sbn_plugin cfe_sbn_bridge.launch.py
```

This launch file accepts as argument **cfe_sbn_config** . This file contains 
configuration information for the bridge. Most of the parameters can be used
with its default values. You'll only want to modify the parameters that 
define the location of your gsw machine, specifically:

- peer_ip
- peer_port
- peer_processor_id

Currently, we have available 2 configuration files to use, both
located in cfe_sbn_plugin/config:

- cfe_sbn_config.yaml: File applicable for a single-host setup. This is the default.
- cfe_sbn_config_multihost.yaml:  File applicable for a multihost setup, such as the one used by brash docker.

You can create your own file using these files as reference.
