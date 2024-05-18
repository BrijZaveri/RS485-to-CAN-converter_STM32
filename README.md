Device A = Charger,
Device B = Battery

Device-A contains RS-485 protocol and Device-B contains CAN protocol, in order to make them communicate without changing their matrix, an RS485<->CAN converter is being intoduced which will capture 
CAN data from Device-B (battery) and send it to Device-A (Charger) seamlessly.

PURPOSE OF THIS CONVERTER:-
OEM sources various components from different vendors like, charger, battery, instrument cluster etc. In order to make them work with each other they have to follow one standard CAN or RS485 matrix (common in EVs). 
Implementing the common matrix is time consuming as vendors does not guarantee or fulfill the implementation time, which cause delay in production of vehicle. 
So RS485<->CAN converter will take care of this problem and boost the production cycle of vehicle 

