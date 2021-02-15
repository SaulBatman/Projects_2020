bag=rosbag('2020-05-23-00-41-33.bag')
G_message_2=select(bag,'MessageType','nav_msgs/OccupancyGrid')
map_data_G2=readMessages(G_message_2)