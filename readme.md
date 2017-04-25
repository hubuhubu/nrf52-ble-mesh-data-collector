nrf52-BLE-MESH-DATA-COLLECTOR
==================

 This repository contains code examples that show nRF52 RBC Mesh demo.
 
Requirements
------------
- nRF5 SDK version 13.0.0
- nRF52840-PDK

To compile it, clone the repository or extract the zip file in the \nRF5_SDK_13\examples\. 

How it works
------------

-	Node on booting will requests a Handle ID from Gateway (commissioning handle = handle 0). If it is assigned a handleID it will start normal operation of sending 23 byte every seconds on that handleID. If no button press it will simply send 1 and 0 in the first byte (always send 23 bytes). If Button 1 or Button 2 pressed it will fix the first byte value to 1 or 2. Press Button 3 bring the LED toggle back. 
-	Gateway can commission the handle ID to a node and then store the Node_ID table to flash. If you hold button 4 on the gateway and reset, it will erase the table
- 	Pressing button 4 on the Gateway also set the network into Reinitialize state and all node will receive the re-initialize command continuouly in the next 10 seconds
-	If there is something on UART + Enter, Node will update that value to the payload (+ the first byte is the LED value) and send to gateway.
-	Gateway print out current data of active nodes every second on UART. After 10 seconds of non-activity, node is marked inactive and won’t display value. 
-	Gateway can be connected by phone and the phone can read the nodes sensor data. Mesh doesn’t work if connection interval = 7.5ms 
-   Default output power is +8dBm.

About this project
------------------
This application is one of several applications that has been built by the support team at Nordic Semiconductor, as a demo of some particular feature or use case. It has not necessarily been thoroughly tested, so there might be unknown issues. It is hence provided as-is, without any warranty. 

The application is built to be used with the official nRF5 SDK, that can be downloaded from http://developer.nordicsemi.com/

Please post any questions about this project on https://devzone.nordicsemi.com.