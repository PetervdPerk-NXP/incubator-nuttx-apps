Example application in NuttX for libuavcanv0

To build you've to manually generate dsdlc files to do this

Install libuavcan_dsdlc tool and run following command in
the apps/examples/libuavcanv0/libuavcan folder

libuavcan_dsdlc dsdl/uavcan/

this will generate the apps/examples/libuavcanv0/libuavcan/dsdlc_generated folder
