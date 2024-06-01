Device A = Charger, Device B = Battery

Device A uses the RS-485 protocol, while Device B uses the CAN protocol. To enable communication between these devices without altering their configurations, an RS-485 to CAN converter is being introduced. This converter will seamlessly capture CAN data from Device B (Battery) and send it to Device A (Charger). 

Purpose of the Converter:
OEMs source various components like chargers, batteries, and instrument clusters from different vendors. To ensure these components work together, they must adhere to a common communication standard, typically CAN or RS-485 in EVs. Implementing this standard in every node is time-consuming, and vendors often do not guarantee timely implementation, causing production delays. An RS-485 to CAN converter addresses this issue by bridging the communication gap, thereby accelerating the vehicle production cycle.
