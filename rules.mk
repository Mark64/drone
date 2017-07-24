# dependency generation from:
# http://make.mad-scientist.net/papers/advanced-auto-dependency-generation/

DEPDIR := .d
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEPDIR)/$(subst %/,, $*).Td

COMPILE.c = $(CC) $(DEPFLAGS) $(CFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -c
COMPILE.cxx = $(CXX) $(DEPFLAGS) $(CXXFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -c
POSTCOMPILE = @$(MV) -f $(DEPDIR)/$(subst /,, $*).Td $(DEPDIR)/$(subst /,, $*).d && touch $@

%.o : %.c
%.o : %.c $(DEPDIR)/%.d
	$(COMPILE.c) $<
	$(POSTCOMPILE)

%.o : %.cc
%.o : %.cc $(DEPDIR)/%.d
	$(COMPILE.cxx) $<
	$(POSTCOMPILE)

%.o : %.cpp
%.o : %.cpp $(DEPDIR)/%.d
	$(COMPILE.cxx) $<
	$(POSTCOMPILE)

$(DEPDIR)/%.d: | $(DEPDIR)
.PRECIOUS: $(DEPDIR)/%.d

$(DEPDIR):
	@$(MKDIR) -p $(DEPDIR)



