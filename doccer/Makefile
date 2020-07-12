
APP=doccer.exe

GENERATED_FILES=doccer_lexxer.cc  doccer_parser.cc  doccer_parser.h

CFLAGS=-Wall -Wno-sign-compare -g -MMD -std=c++11

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

# An example of how to use doccer to make an HTML file from .doc.
#
# We write doccer output to temporary output rather than directly to the output
# file otherwise restarting this rule after an error will always succeed, as
# the output file will have been updated.
MATHJAX_SUBST=https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.5/MathJax.js?config=TeX-MML-AM_CHTML
%.html: %.doc
	./doccer.exe -t template.html $< | sed 's@__MATHJAX__@$(MATHJAX_SUBST)@' > __temporary_output.html
	mv __temporary_output.html $@


-include *.d
