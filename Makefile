#
# qrouter Makefile
#

# Main compiler arguments
CFLAGS += -g -O2
CPPFLAGS =  -m64 -fPIC
DEFS = -DPACKAGE_NAME=\"\" -DPACKAGE_TARNAME=\"\" -DPACKAGE_VERSION=\"\" -DPACKAGE_STRING=\"\" -DPACKAGE_BUGREPORT=\"\" -DPACKAGE_URL=\"\" -DSTDC_HEADERS=1 -DHAVE_SETENV=1 -DHAVE_PUTENV=1 -DHAVE_VA_COPY=1 -DHAVE___VA_COPY=1 -DQROUTER_PATH=\"/usr/local/share/qrouter\" -DHAVE_LIBXT=1 -DTCL_QROUTER=1 -DLINUX=1 -DSYSV=1 -DVERSION=\"1.3\" -DREVISION=\"0\"
LIBS = -lXt 
LDFLAGS += 
LDDL_FLAGS = -shared -Wl,-soname,$@ -Wl,--version-script=symbol.map
LD_RUN_PATH = 
SHLIB_CFLAGS = -fPIC
LIB_SPECS =  -ltk8.5 -ltcl8.5
INC_SPECS = 
TCL_LIB_DIR = /usr/lib64
TK_LIB_DIR = /usr/lib64
EXTRA_LIB_SPECS = -ldl
INSTALL = /usr/bin/install -c
SHDLIB_EXT = .so
EXEEXT = 
X_LIBS = 
X_EXTRA_LIBS = 
X_PRE_LIBS =  -lSM -lICE
LIBINSTALL = /usr/local/share/qrouter
WISH_EXE = /usr/bin/wish
VERSION = 1.3
REVISION = 0
prefix = /usr/local

INSTALL_TARGET := install-tcl
ALL_TARGET := tcl

SOURCES = qrouter.c maze.c node.c qconfig.c lef.c def.c
OBJECTS := $(patsubst %.c,%.o,$(SOURCES))

SOURCES2 = graphics.c tclqrouter.c tkSimple.c
OBJECTS2 := $(patsubst %.c,%.o,$(SOURCES2))

SOURCES3 = qrouterexec.c
OBJECTS3 := $(patsubst %.c,%.o,$(SOURCES3))

SOURCES4 = main.c
OBJECTS4 := $(patsubst %.c,%.o,$(SOURCES4))

BININSTALL = ${prefix}/bin

all: $(ALL_TARGET)

install: $(INSTALL_TARGET)

nointerp: qrouter$(EXEEXT)

tcl: qrouter.sh qrouter.tcl qrouter$(SHDLIB_EXT) qrouterexec$(EXEEXT)

qrouter.tcl: qrouter.tcl.in
	sed -e '/LIBDIR/s#LIBDIR#${LIBINSTALL}#' \
	    -e '/VERSION/s#VERSION#${VERSION}#' \
	    -e '/REVISION/s#REVISION#${REVISION}#' \
		qrouter.tcl.in > $@

qrouter.sh: qrouter.sh.in
	sed -e '/WISH_EXE/s#WISH_EXE#${WISH_EXE}#' \
	    -e '/LIBDIR/s#LIBDIR#${LIBINSTALL}#' \
		qrouter.sh.in > $@
	chmod 0755 $@

qrouter$(EXEEXT): $(OBJECTS) $(OBJECTS4)
	$(CC) $(LDFLAGS) $(OBJECTS) $(OBJECTS4) -o $@ $(LIBS) -lm

qrouter$(SHDLIB_EXT): $(OBJECTS) $(OBJECTS2)
	$(RM) qrouter$(SHDLIB_EXT)
	$(CC) ${CFLAGS} ${SHLIB_CFLAGS} -o $@ \
		${LDDL_FLAGS} $(OBJECTS) $(OBJECTS2) \
		${LDFLAGS} -lc ${LIBS} ${X_PRE_LIBS} -lX11 ${X_LIBS} \
		${X_EXTRA_LIBS} ${LIB_SPECS} ${EXTRA_LIB_SPECS} -lm

qrouterexec$(EXEEXT): $(OBJECTS3)
	$(RM) qrouterexec$(EXEEXT)
	$(CC) ${CFLAGS} ${CPPFLAGS} ${DEFS} \
		${SOURCES3} ${INC_SPECS} -o $@  ${LIB_SPECS} \
		${LD_RUN_PATH} ${LDFLAGS} ${X_PRE_LIBS} -lX11 ${X_LIBS} \
		${X_EXTRA_LIBS} ${LIBS} ${EXTRA_LIB_SPECS} -lm

install-nointerp:
	@echo "Installing qrouter"
	$(INSTALL) -d $(DESTDIR)${BININSTALL}
	$(INSTALL) qrouter $(DESTDIR)${BININSTALL}

install-tcl: qrouter.sh qrouter.tcl qrouter$(SHDLIB_EXT) qrouterexec$(EXEEXT)
	@echo "Installing qrouter"
	$(INSTALL) -d $(DESTDIR)${BININSTALL}
	$(INSTALL) -d $(DESTDIR)${LIBINSTALL}
	$(INSTALL) qrouter.sh $(DESTDIR)${BININSTALL}/qrouter
	$(INSTALL) qrouter$(SHDLIB_EXT) $(DESTDIR)${LIBINSTALL}
	$(INSTALL) qrouterexec$(EXEEXT) $(DESTDIR)${LIBINSTALL}
	$(INSTALL) console.tcl $(DESTDIR)${LIBINSTALL}
	$(INSTALL) tkcon.tcl $(DESTDIR)${LIBINSTALL}
	$(INSTALL) qrouter.tcl $(DESTDIR)${LIBINSTALL}

uninstall:
	$(RM) $(DESTDIR)${BININSTALL}/qrouter

clean:
	$(RM) $(OBJECTS)
	$(RM) $(OBJECTS2)
	$(RM) $(OBJECTS3)
	$(RM) $(OBJECTS4)
	$(RM) qrouterexec$(EXEEXT)
	$(RM) qrouter$(EXEEXT)
	$(RM) qrouter$(SHDLIB_EXT)
	$(RM) qrouter.tcl
	$(RM) qrouter.sh

veryclean:
	$(RM) $(OBJECTS)
	$(RM) $(OBJECTS2)
	$(RM) $(OBJECTS3)
	$(RM) $(OBJECTS4)
	$(RM) qrouterexec$(EXEEXT)
	$(RM) qrouter$(EXEEXT)
	$(RM) qrouter$(SHDLIB_EXT)
	$(RM) qrouter.tcl
	$(RM) qrouter.sh

.c.o:
	$(CC) $(CFLAGS) $(CPPFLAGS) $(SHLIB_CFLAGS) $(DEFS) $(INC_SPECS) -c $< -o $@
