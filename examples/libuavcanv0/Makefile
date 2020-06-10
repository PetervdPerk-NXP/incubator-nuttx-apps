############################################################################
# apps/examples/netpkt/Makefile
#
#   Copyright (C) 2014 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs

# Network packet socket example

PROGNAME = libuavcanv0_example
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = 20480
MODULE = $(CONFIG_EXAMPLES_LIBUAVCANV0_EXAMPLE)

CXXEXT = .cpp

CXXFLAGS += -DUAVCAN_CPP_VERSION=UAVCAN_CPP03 -DUAVCAN_NO_ASSERTIONS -DUAVCAN_PLATFORM=generic -std=c++11

CXXFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(APPDIR)/examples/libuavcanv0/libuavcan/dsdlc_generated"}

CXXFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(APPDIR)/examples/libuavcanv0/libuavcan/libuavcan/include"}

CXXFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(APPDIR)/examples/libuavcanv0/platform_specific_components/nuttx/libuavcan/include"}

MAINSRC   = uavcan_main.cpp
CXXSRCS  += libuavcan/libuavcan/src/node/uc_scheduler.cpp
CXXSRCS  += libuavcan/libuavcan/src/node/uc_service_client.cpp
CXXSRCS  += libuavcan/libuavcan/src/node/uc_timer.cpp
CXXSRCS  += libuavcan/libuavcan/src/node/uc_generic_subscriber.cpp
CXXSRCS  += libuavcan/libuavcan/src/node/uc_generic_publisher.cpp
CXXSRCS  += libuavcan/libuavcan/src/node/uc_global_data_type_registry.cpp
CXXSRCS  += libuavcan/libuavcan/src/transport/uc_dispatcher.cpp
CXXSRCS  += libuavcan/libuavcan/src/transport/uc_transfer_sender.cpp
CXXSRCS  += libuavcan/libuavcan/src/transport/uc_transfer_receiver.cpp
CXXSRCS  += libuavcan/libuavcan/src/transport/uc_frame.cpp
CXXSRCS  += libuavcan/libuavcan/src/transport/uc_transfer_buffer.cpp
CXXSRCS  += libuavcan/libuavcan/src/transport/uc_can_acceptance_filter_configurator.cpp
CXXSRCS  += libuavcan/libuavcan/src/transport/uc_transfer_listener.cpp
CXXSRCS  += libuavcan/libuavcan/src/transport/uc_crc.cpp
CXXSRCS  += libuavcan/libuavcan/src/transport/uc_transfer.cpp
CXXSRCS  += libuavcan/libuavcan/src/transport/uc_outgoing_transfer_registry.cpp
CXXSRCS  += libuavcan/libuavcan/src/transport/uc_can_io.cpp
CXXSRCS  += libuavcan/libuavcan/src/uc_error.cpp
CXXSRCS  += libuavcan/libuavcan/src/driver/uc_can.cpp
CXXSRCS  += libuavcan/libuavcan/src/uc_data_type.cpp
CXXSRCS  += libuavcan/libuavcan/src/uc_dynamic_memory.cpp
CXXSRCS  += libuavcan/libuavcan/src/protocol/uc_dynamic_node_id_client.cpp
CXXSRCS  += libuavcan/libuavcan/src/protocol/uc_node_status_provider.cpp
CXXSRCS  += libuavcan/libuavcan/src/marshal/uc_scalar_codec.cpp
CXXSRCS  += libuavcan/libuavcan/src/marshal/uc_float_spec.cpp
CXXSRCS  += libuavcan/libuavcan/src/marshal/uc_bit_stream.cpp
CXXSRCS  += libuavcan/libuavcan/src/marshal/uc_bit_array_copy.cpp

include $(APPDIR)/Application.mk
