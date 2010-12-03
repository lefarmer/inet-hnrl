# Source files

SRC_OMNET =  debug.c dymo_generic.c dymo_re.c dymo_socket.c dymo_timeout.c \
         dymo_uerr.c dymo_rerr.c rtable.c timer_queue.c pending_rreq.c \
         icmp_socket.c blacklist.c dymo_nb.c dymo_hello.c


INCLUDE_PATH= -I. -I../../../../Transport/Contract -I../../../Contract -I../../../../NetworkInterfaces/Contract -I../../../../Base -I../../../../Util -I../../../IPv4 -I../../../Contract -I../../../AutoRouting -I../../../Queue  -I../../../../Transport/UDP -I../../../../NetworkInterfaces -I../../../../Nodes/INET -I../. -I$(OMNETPP_INCL_DIR)

# Object files
SRC_OMNET_CPP =	
OBJS_OMNET = $(SRC_OMNET:%.c=%.o)
OBJS_OMNET_CPP = 


# Kernel stuff
KERNEL=$(shell uname -r)
KERNEL_DIR=/lib/modules/$(KERNEL)/build
KERNEL_INC=$(KERNEL_DIR)/include

# Compiler options
CC= gcc
CPP= g++
C_OPTS= -Wall -O3
CPP_OPTS= -Wall

export CC

DEBUG= -g -O0 -DDEBUG
XDEFS=
DEFS=#-DCONFIG_GATEWAY #-DLLFEEDBACK
CFLAGS=$(OPTS) $(DEBUG) $(DEFS) $(XDEFS)
LD_OPTS=


OMNET_DEFS= -DOMNETPP -DNS_PORT

# Set extra DEFINES here. Link layer feedback is now a runtime option.
EXTRA_OMNET_DEFS=

include ../../../../inetconfig

# Note: OPTS is overridden by OMNET Makefile
OMNET_CFLAGS=$(OPTS) $(CPP_OPTS) $(DEBUG) $(OMNET_DEFS) $(EXTRA_OMNET_DEFS)

OMNET_INC= $(INCLUDE_PATH) # 

OMNET_TARGET=libdymo-um.a


# Archiver and options
AR=ar
AR_FLAGS=rc

.PHONY: default clean install uninstall depend kdymo

# Targets
default: $(OMNET_TARGET)

$(OBJS): %.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJS_NS): %-ns.o: %.c Makefile
	$(CPP) $(NS_CFLAGS) $(NS_INC) -c -o $@ $<

$(OBJS_NS_CPP): %-ns.o: %.cc Makefile
	$(CPP) $(NS_CFLAGS) $(NS_INC) -c -o $@ $<

$(NS_TARGET): $(OBJS_NS_CPP) $(OBJS_NS)
	$(AR) $(AR_FLAGS) $@ $(OBJS_NS_CPP) $(OBJS_NS) > /dev/null

$(OBJS): %.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJS_OMNET): %.o: %.c Makefile
	$(CPP) $(OMNET_CFLAGS) $(OMNET_INC) -c -o $@ $<

$(OBJS_OMNET_CPP): %-ns.o: %.cc Makefile
	$(CPP) $(OMNET_CFLAGS) $(OMNET_INC) -c -o $@ $<


$(OMNET_TARGET): $(OBJS_OMNET_CPP) $(OBJS_OMNET) 
	$(AR) $(AR_FLAGS) $@ $(OBJS_OMNET_CPP) $(OBJS_OMNET) > /dev/null


dymod: $(OBJS) Makefile
	$(CC) $(CFLAGS) -o $@ $(OBJS) $(LD_OPTS)

kdymo:
	$(MAKE) -C lnx KERNEL_DIR=$(KERNEL_DIR) KCC=$(CC)

depend:
	@echo "Updating Makefile dependencies..."
	@makedepend -Y./ -- $(DEFS) -- $(SRC) &>/dev/null
	@makedepend -a -Y./ -- $(KDEFS) kdymo.c &>/dev/null

install: default
	install -s -m 755 dymod /usr/sbin/dymod
	
	@if [ ! -d /lib/modules/$(KERNEL)/dymo ]; then \
		mkdir /lib/modules/$(KERNEL)/dymo; \
	fi
	@echo "Installing kernel module in /lib/modules/$(KERNEL)/dymo/";
	@if [ -f ./kdymo.ko ]; then \
		install -m 644 kdymo.ko /lib/modules/$(KERNEL)/dymo/kdymo.ko; \
	else \
		install -m 644 kdymo.o /lib/modules/$(KERNEL)/dymo/kdymo.o; \
	fi
	/sbin/depmod -a

uninstall:
	rm -f /usr/sbin/dymod
	rm -rf /lib/modules/$(KERNEL)/dymo

clean:
	rm -f dymod *~ *.o *.ko core *.log $(OMNET_TARGET) ns/*.o ns/*~
	cd lnx && $(MAKE) clean

generateheaders: # target invoked by OMNeT++-generated uplevel Makefile.vc; does nothing for now