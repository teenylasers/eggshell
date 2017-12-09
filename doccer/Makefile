
APP=doccer.exe

GENERATED_FILES=doccer_lexxer.cc  doccer_parser.cc  doccer_parser.h

CFLAGS=-Wall -Wno-sign-compare -g -MMD

all: $(APP)

doccer.exe: doccer.o doccer_lexxer.o doccer_parser.o doccer_util.o
	g++ $(CFLAGS) -o $@ $(OBJS) $^

%.o: %.cc
	g++ -c $(CFLAGS) $<

doccer_lexxer.cc: doccer.lex
	flex -s -8 --outfile=$@ --bison-bridge $<

doccer_parser.cc doccer_parser.h: doccer.y
	bison --defines=doccer_parser.h $< -o doccer_parser.cc

doccer.o: doccer_parser.h

clean:
	rm -f *.o *.d $(GENERATED_FILES)

realclean: clean
	rm -f $(APP)

-include *.d
