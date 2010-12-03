
include ../../../../inetconfig

SRC=dsr-module.c dsr-pkt.c dsr-dev.c dsr-io.c dsr-opt.c dsr-rreq.c dsr-rrep.c dsr-rerr.c dsr-ack.c dsr-srt.c send-buf.c debug.c neigh.c maint-buf.c

OMNET_SRC=dsr-pkt.c dsr-io.c dsr-opt.c dsr-rreq.c dsr-rrep.c dsr-rerr.c dsr-ack.c dsr-srt.c send-buf.c neigh.c maint-buf.c link-cache.c path-cache.c

OMNET_SRC_CPP=

OBJS_OMNET = $(OMNET_SRC:%.c=%.o)
OBJS_OMNET_CPP = $(OMNET_SRC_CPP:%.cc=%.o)

DEFS=-DDEBUG 

MODNAME=dsr
RTC_TRG=linkcache
RTC_SRC=link-cache.c


INCLUDE_PATH= -I. -I../../../Transport/Contract  -I../../../NetworkInterfaces/Contract -I../../../Base -I../../../Util -I../../IPv4 -I../../Contract -I../../AutoRouting -I../../Queue  -I../../../Transport/UDP -I../../../NetworkInterfaces -I../../../Nodes/INET -I$(OMNETPP_INCL_DIR)

INCLUDE_PATH= -I. -I../../../../Transport/Contract -I../../../Contract -I../../../../NetworkInterfaces/Contract -I../../../../Base -I../../../../Util -I../../../IPv4 -I../../../Contract -I../../../AutoRouting -I../../../Queue  -I../../../../Transport/UDP -I../../../../NetworkInterfaces -I../../../../Nodes/INET -I../. -I$(OMNETPP_INCL_DIR)


ifneq (,$(findstring 2.6,$(KERNELRELEASE)))


else

export-objs := link-cache.o


CC=gcc
CPP=g++

MIPS_CC=mipsel-linux-gcc
MIPS_LD=mipsel-linux-ld

# NS2
OBJS_OMNET=$(OMNET_SRC:%.c=%.o)
OBJS_OMNET_CPP=$(OMNET_SRC_CPP:%.cc=%.o)

NS_DEFS= # DON'T CHANGE (overridden by NS Makefile)

OMNET_DEFS= -DOMNETPP 

EXTRA_OMNET_DEFS=-DDEBUG -g3 -pg

OMNET_CFLAGS=$(OPTS) $(CPP_OPTS) $(DEBUG) $(OMNET_DEFS) $(EXTRA_OMNET_DEFS)

OMNET_INC= $(INCLUDE_PATH) # 

OMNET_TARGET=libdsr-uu.a


# Set extra DEFINES here. Link layer feedback is now a runtime option.

# Archiver and options
AR=ar
AR_FLAGS=rc

#######
VERSION=$(shell if [ ! -d $(KERNEL_DIR) ]; then echo "No linux source found!!! Check your setup..."; exit; fi; grep ^VERSION $(KERNEL_DIR)/Makefile | cut -d' ' -f 3)
PATCHLEVEL=$(shell grep ^PATCHLEVEL $(KERNEL_DIR)/Makefile | cut -d' ' -f 3)
SUBLEVEL=$(shell grep ^SUBLEVEL $(KERNEL_DIR)/Makefile | cut -d' ' -f 3)
#######

ifeq ($(PATCHLEVEL), 6)
MODPREFIX=ko
else
MODPREFIX=o
endif

.PHONY: mips default depend clean ns clean-2.4 clean-2.6 indent

# Check for kernel version
#ifeq ($(PATCHLEVEL),6)
#default: $(MODNAME).ko $(RTC_TRG).ko TODO
#clean: clean-2.6
#else 
# Assume kernel 2.4
#default: $(MODNAME).o $(RTC_TRG).o TODO
#clean: clean-2.4
#endif



#$(OMNET_TARGET): $(OBJS_OMNET_CPP) $(OBJS_OMNET) *.h
#$(AR) $(AR_FLAGS) $@ $(OBJS_NS_CPP) $(OBJS_NS) 
#	$(LD) -r -o $@ $(OBJS_OMNET_CPP) $(OBJS_OMNET)  


default: $(OMNET_TARGET)


$(OBJS_OMNET): %.o: %.c Makefile
	$(CPP) $(OMNET_CFLAGS) $(OMNET_INC) -c -o $@ $<

$(OBJS_OMNET_CPP): %.o: %.cc Makefile
	$(CPP) $(OMNET_CFLAGS) $(OMNET_INC) -c -o $@ $<

$(OMNET_TARGET): $(OBJS_OMNET_CPP) $(OBJS_OMNET) *.h 
	$(AR) $(AR_FLAGS) $@ $(OBJS_OMNET_CPP) $(OBJS_OMNET) > /dev/null


TODO:
	grep -n "TODO:" *.c *.h > TODO
	cat TODO

TAGS: *.c *.h
	etags $(SRC) *.h

indent:
	indent -kr -i8 -ts8 -sob -l80 -ss -ncs *.c *.h


clean:
	rm -rf Makefile.bak TAGS TODO $(OBJS_OMNET_CPP) $(OBJS_OMNET)  *~ $(OMNET_TARGET)



endif
# DO NOT DELETE

dsr-module.o: dsr.h  dsr-pkt.h timer.h dsr-dev.h dsr-io.h debug.h
dsr-module.o: neigh.h dsr-rreq.h dsr-rrep.h dsr-srt.h maint-buf.h send-buf.h
dsr-module.o: link-cache.h tbl.h list.h
dsr-pkt.o: debug.h dsr-opt.h dsr.h  dsr-pkt.h timer.h
dsr-dev.o: debug.h dsr.h  dsr-pkt.h timer.h neigh.h dsr-opt.h
dsr-dev.o: dsr-rreq.h link-cache.h tbl.h list.h dsr-srt.h dsr-ack.h
dsr-dev.o: send-buf.h maint-buf.h dsr-io.h
dsr-io.o: dsr.h  dsr-pkt.h timer.h dsr-rreq.h dsr-rrep.h dsr-srt.h
dsr-io.o: debug.h dsr-ack.h dsr-rtc.h maint-buf.h neigh.h dsr-opt.h
dsr-io.o: link-cache.h tbl.h list.h send-buf.h
dsr-opt.o: debug.h dsr.h  dsr-pkt.h timer.h dsr-opt.h dsr-rreq.h
dsr-opt.o: dsr-rrep.h dsr-srt.h dsr-rerr.h dsr-ack.h
dsr-rreq.o: debug.h dsr.h  dsr-pkt.h timer.h tbl.h list.h
dsr-rreq.o: dsr-rrep.h dsr-srt.h dsr-rreq.h dsr-opt.h link-cache.h send-buf.h
dsr-rreq.o: neigh.h
dsr-rrep.o: dsr.h  dsr-pkt.h timer.h debug.h tbl.h list.h
dsr-rrep.o: dsr-rrep.h dsr-srt.h dsr-rreq.h dsr-opt.h link-cache.h send-buf.h
dsr-rerr.o: dsr.h  dsr-pkt.h timer.h dsr-rerr.h dsr-opt.h debug.h
dsr-rerr.o: dsr-srt.h dsr-ack.h link-cache.h tbl.h list.h maint-buf.h
dsr-ack.o: tbl.h list.h debug.h dsr-opt.h dsr.h  dsr-pkt.h timer.h
dsr-ack.o: dsr-ack.h link-cache.h neigh.h maint-buf.h
dsr-srt.o: dsr.h  dsr-pkt.h timer.h dsr-srt.h debug.h dsr-opt.h
dsr-srt.o: dsr-ack.h link-cache.h tbl.h list.h neigh.h dsr-rrep.h
send-buf.o: tbl.h list.h send-buf.h dsr.h  dsr-pkt.h timer.h
send-buf.o: debug.h link-cache.h dsr-srt.h
debug.o: debug.h dsr.h  dsr-pkt.h timer.h
neigh.o: tbl.h list.h neigh.h dsr.h dsr-pkt.h timer.h debug.h
maint-buf.o: dsr.h dsr-pkt.h timer.h debug.h tbl.h list.h neigh.h
maint-buf.o: dsr-ack.h link-cache.h dsr-rerr.h dsr-dev.h dsr-srt.h dsr-opt.h
maint-buf.o: maint-buf.h ../dsr-uu-omnetpp.h send-buf.h dsr-rreq.h dsr-rrep.h
maint-buf.o: ../dsr-pkt_omnet.h dsr-io.h


generateheaders: # target invoked by OMNeT++-generated uplevel Makefile.vc; does nothing for now

depend:	# target invoked by OMNeT++-generated uplevel Makefile.vc; does nothing for now
