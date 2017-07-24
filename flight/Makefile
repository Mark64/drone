# drivers Makefile
# by Mark Hill

include ../rules.mk

sources := 
objects := $(patsubst %.c, %.o, $(patsubst %.cpp, %.o, $(sources)))

sensors-built-in.o: $(objects)

.PHONY: clean

clean:
	$(RM) -rf $(DEPDIR)/*.d $(objects)

include $(wildcard $(patsubst %, $(DEPDIR)/%.d, $(sources)))



