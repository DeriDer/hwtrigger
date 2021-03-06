################################################################################
# Trigger Makefile
################################################################################
PROJECT_ROOT=../../
OPT_INC = ${PROJECT_ROOT}/common/make/common_spin.mk
-include ${OPT_INC}

################################################################################
# Key paths and settings
################################################################################
ifeq ($(wildcard ${OPT_INC}),)
CFLAGS += -std=c++11 -pthread
LDFLAGS = -pthread
CXX = g++ ${CFLAGS}
ODIR  = .obj/build${D}
SDIR  = .
MKDIR = mkdir -p
endif

OUTPUTNAME = Trigger${D}
OUTDIR = ../../bin

################################################################################
# Dependencies
################################################################################
# Spinnaker deps
SPINNAKER_LIB = -L../../lib -lSpinnaker${D} ${SPIN_DEPS}

################################################################################
# Master inc/lib/obj/dep settings
################################################################################
_OBJ = Trigger.o roboclaw.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))
INC = -I../../include -I/usr/include/spinnaker  -I../../lib/Scheduler
ifneq ($(OS),mac)
LIB += -Wl,-Bdynamic ${SPINNAKER_LIB}
LIB += -Wl,-rpath-link=../../lib
else
LIB += -rpath ../../lib
LIB += ${SPINNAKER_LIB} -lpthread
endif
################################################################################
# Rules/recipes
################################################################################
# Final binary
${OUTPUTNAME}: ${OBJ}
	${CXX} ${LDFLAGS} -o ${OUTPUTNAME} ${OBJ} ${LIB}
	mv ${OUTPUTNAME} ${OUTDIR}

# Intermediate object files
${OBJ}: ${ODIR}/%.o : ${SDIR}/%.cpp
	@${MKDIR} ${ODIR}
	${CXX} ${CFLAGS} ${INC} -Wall -D LINUX -c $< -o $@

# Clean up intermediate objects
clean_obj:
	rm -f ${OBJ}
	@echo "intermediate objects cleaned up!"

# Clean up everything.
clean: clean_obj
	rm -f ${OUTDIR}/${OUTPUTNAME}
	@echo "all cleaned up!"
