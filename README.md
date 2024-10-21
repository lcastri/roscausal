# <img src="https://github.com/lcastri/roscausal/blob/main/images/roscausal.png" width="35"> ROS-Causal
![](https://github.com/lcastri/roscausal/blob/main/images/intro.png "ROS-Causal")

A ROS-Based Causal Analysis Framework for Human-Robot Interaction Applications. The library facilitates onboard data collection and causal discovery in human-robot spatial interactions. Due to its modular design, it allows the integration of new causal discovery methods and functionalities. The framework has been incorporated into a Docker container named [ROS-Causal_HRISim](https://github.com/lcastri/ROS-Causal_HRISim). ROS-Causal_HRISim represents a Gazebo-based simulator for HRI scenarios involving a TIAGo robot and multiple pedestrians.

## Features
### Data Merging
This block is composed by two ROS nodes **roscausal_robot** and **roscausal_human** and its aim is to merge robot and human data from various topics into custom ROS messages defined in the ROS-Causal framework. In detail, the nodes **roscausal_robot** and **roscausal_human** extract the position, orientation, velocities and target positions of the robot and the human, respectively. These data are retrieved from ROS topics/params relative to the robotic platform and need to be configured within the framework. Then, the two nodes merge the acquired data into the ROS messages **RobotState** and **HumanState** published on the predefined topics **/roscausal/robot** and **/roscausal/human**.

**roscausal_robot** parameters:
* odom_topic
* pose_topic
* goal_topic
* source_frame
* target_frame

**roscausal_human** parameters:
* people_topic
* goal_param
* source_frame
* target_frame

### Data Collection and Post-processing
The data collection and post-processing block takes input from the previous block's topics to create a data batch for the causal discovery node. The **roscausal_data** node subscribes to the topics **/roscausal/robot** and **/roscausal/human**, collecting data in a CSV file. Once the desired time-series length, configurable in the system as a ROS parameter, is reached, the node provides the option to post-process the data and finally saves it into a designated folder. Parameters list:
* data_dir: Data directory - default = roscausal_data/data_pool/
* pp_data_dir: Post-processed data directory - default = roscausal_data/pp_pool/
* pp_script_dir: Post-processing scripts directory - default = roscausal_data/pp_scripts/
* pp_script: Post-processing script (optional)
* ts_length: Desired time-series length - default = 150 [s] 
* subsampling: Subsampling bit - default = False 
* dt: Subsampling step. Ignored if subsampling=False - default = 0.1 [s]
* id_format: Filename ID format - default = "%Y%m%d_%H%M%S" 
* csv_prefix: Filename prefix - default = "data_"

If you want to integrate your post-processing script, you need to add a Python script to the **pp_script_dir** folder.

### Causal Discovery
The **roscausal_discovery** ROS node performs causal discovery analysis on the collected data. Specifically, the ROS node continuously checks for the presence of a CSV file in the designated folder. Upon locating a file, it initiates the causal analysis on that specific data batch. If multiple CSV files are present, it prioritises the oldest one for the analysis. It is important to note that the **roscausal_data** and **roscausal_discovery** ROS nodes operate asynchronously, allowing the simultaneous execution of causal analysis on one dataset while continuing the collection of another. Parameters list:
* data_dir: Data directory - default = roscausal_data/pp_pool/
* res_dir: Result directory. If not specified, the causal model is not saved but still published - default = roscausal_discovery/cm_pool/
* cd_method: Causal discovery algorithm [fpcmci, pcmci]- default = fpcmci
* sig_alpha: Significance level - default = 0.05
* min_lag: Minimum time lag for the causal analysis 
* max_lag: Maximum time lag for the causal analysis
* id_format: Filename ID format - default = "%Y%m%d_%H%M%S" 
* csv_prefix: Filename prefix - default = "data_"

If you want to integrate new causal discovery methods, you need to add a Python script to the **roscausal_discovery/scripts/causal_discovery_methods** folder that includes a method called **run**. The latter must take the following parameters as input:
* csvpath: path to the csv to analyse 
* csvname: csv name
* alpha: significance level
* minlag: minimum time lag for causal analysis 
* maxlag: maximum time lag for causal analysis
* resdir: result directory
  
Moreover, it must output the list of features and the three matrices for the CausalModel msg (causal_structure, val_matrix, pval_matrix).

## ROS messages
**RobotState** and **HumanState** messages currently share the same structure. However, two distinct messages were created to accommodate potential modifications. Given the differences between agents, it might be convenient to have distinct information and, consequently, different message structures. The structures are as follows:
* Header header
* geometry_msgs/Pose2D pose2D
* geometry_msgs/Twist twist
* geometry_msgs/Point goal

The **CausalModel** message stores a list of variables, three matrices derived from the causal discovery analysis, and their original shapes, which are useful to reconstruct the matrices in other nodes subscribing to this message. The following is the message structure:
* Header header
* string[] features
* std_msgs/Int32MultiArray causal_structure
* std_msgs/Float32MultiArray val_matrix
* std_msgs/Float32MultiArray pval_matrix
* int32[] original_shape

## Requirements
The ROS-Causal library requires the [F-PCMCI](https://github.com/lcastri/fpcmci) for the causal discovery analysis.

## Citation

If you found this useful for your work, please cite this papers:
```
@inproceedings{castri2024exp,
  title={Experimental Evaluation of ROS-Causal in Real-World Human-Robot Spatial Interaction Scenarios},
  author={Castri, Luca and Beraldo, Gloria and Mghames, Sariah and Hanheide, Marc and Bellotto, Nicola},
  booktitle={33nd IEEE International Conference on Robot and Human Interactive Communication (RO-MAN)},
  pages={},
  year={2024},
  organization={IEEE}
}
```

## Recent changes
| Version | Changes |
| :---: | ----------- |
| 1.1.0 | roscausal_human new topic: "/roscausal/humans"<br>roscausal_discovery new topics: "/roscausal/dag" and "/roscausal/tsdag" for RViz visualisation<br>improved robustness in roscausal_discovery|
| 1.0.0 | package released|
