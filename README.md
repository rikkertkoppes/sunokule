# working electronics

ESP D18 ----- SW1
ESP GND ----- SW2

ESP GND ----- CAP-
ESP VCC ----- CAP+

ESP VCC ----- BUS1
ESP D26 ----- BUS2
ESP GND ----- BUS3

ESP D16 ----- DIP6
ESP D17 ----- DIP5
ESP D21 ----- DIP4
ESP D32 ----- DIP3
ESP D22 ----- DIP2
ESP D25 ----- DIP1


# flashing

run `. ~/esp/esp-idf/export.sh` to set up the environment




# UDP Client example

(See the README.md file in the upper level 'examples' directory for more information about examples.)

The application creates UDP socket and sends message to the predefined port and IP address. After the server's reply, the application prints received reply as ASCII text, waits for 2 seconds and sends another message.

## How to use example

In order to create UDP server that communicates with UDP Client example, choose one of the following options.

There are many host-side tools which can be used to interact with the UDP/TCP server/client. 
One command line tool is [netcat](http://netcat.sourceforge.net) which can send and receive many kinds of packets. 
Note: please replace `192.168.0.167 3333` with desired IPV4/IPV6 address (displayed in monitor console) and port number in the following commands.

In addition to those tools, simple Python scripts can be found under sockets/scripts directory. Every script is designed to interact with one of the examples.

### Send UDP packet via netcat
```
echo "Hello from PC" | nc -w1 -u 192.168.0.167 3333
```

### Receive UDP packet via netcat
```
echo "Hello from PC" | nc -w1 -u 192.168.0.167 3333
```

### UDP server using netcat
```
nc -u -l 192.168.0.167 3333
```

### Python scripts
Script example_test.py could be used as a counter part to the udp-client application, ip protocol name (IPv4 or IPv6) shall be stated as argument. Example:

```
python example_test.py IPv4
```
Note that this script is used in automated tests, as well, so the IDF test framework packages need to be imported;
please add `$IDF_PATH/tools/ci/python_packages` to `PYTHONPATH`.


## Hardware Required

This example can be run on any commonly available ESP32 development board.

## Configure the project

```
idf.py menuconfig
```

Set following parameter under Serial Flasher Options:

* Set `Default serial port`.

Set following parameters under Example Configuration Options:

* Set `IP version` of example to be IPV4 or IPV6.

* Set `IPV4 Address` in case your chose IP version IPV4 above.

* Set `IPV6 Address` in case your chose IP version IPV6 above.

* Set `Port` number that represents remote port the example will send data and receive data from.

Configure Wi-Fi or Ethernet under "Example Connection Configuration" menu. See "Establishing Wi-Fi or Ethernet Connection" section in [examples/protocols/README.md](../../README.md) for more details.


## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.


## Troubleshooting

Start server first, to receive data sent from the client (application).
