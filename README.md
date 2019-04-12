_[ note: this readme is in jest, but captures the spirit of the project in lieu of actual documentation ]_

## embedded interface
Embedded Interface aims to reduce the complexity of starting a new mechatronic project. From reconciling sensor communication protocols, to convincing the hardware team that you _only need one more_ wire routed through a chassis, a lot of time is wasted reinventing a communications stack that essentially turns raw sensor data into a packet that can be digested somewhere else. Why can’t we just plug in a sensor and start focusing on what it is we are trying to build, instead of rewriting a service discovery protocol for the n-th time when we just wanted to get a UART module into ROS? Taking a page from both ROS and distributed system design, these building blocks aim to push the complexity into reusable edge nodes that allow you to prototype faster, spend more time focusing on your use-case, and ship a more reliable product.

## how it works:
Each integration is composed of a sensor board, a reusable communications stack, and a power bus convertor.

[ image showing squares with examples of each ]

The sensor board contains the discrete circuitry required for your sensing application. Currently, we are developing these reference designs:
* temperature controller: couples a thermo couple with a pid controlled solid state relay. [done]
* Stepper driver: a high resolution stepper driver based off of the TMC2130 [ done ]
* VESC driver: regenerative braking and full system diagnostics come from this brushless motor controller.
* Camera Controller: create full resolution RTP streams on your network, or sample single images at will.
* Battery monitor: Watch your cell voltages change under load and make sure you still have enough juice to get home.

The communications stack is responsible for service discovery, network configuration, firmware updates, and mitigating communication between your bus and the sensor board. Found out that you can’t use wifi in your new aluminum chassis? Swap it out for the ethernet version and continue with your day. On the roadmap there are these designs:
* mqtt / wifi: configure your wireless network through bluetooth, then let mDNS configure the mqtt broker. [done]
* mqtt / ethernet: similar to the wifi version, but hardwired and configured with DHCP.
* EtherCAT: For you latency sensitive users.
* USB: sometimes you just want to test something.

The power bus convertor allows you to run a single power bus through your application, and converts that line voltage into whatever your sensor requires, all while providing reversed polarity and short circuit protection. Need to change the battery voltage to drive more torque out of your brushless motors? Just swap out the power bus convertors on your sensors, and plug in that bigger battery!
* These come in all shapes and sizes, but I needed a bullet point for consistency.

## example application
A wireless reflow oven with an automatic door

[ insert image of a temp controller, stepper driver, and mqtt broker ]

