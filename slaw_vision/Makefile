include $(shell rospack find mk)/cmake.mk

.PHONY: check-syntax
check-syntax: 
	gcc -Wall -fsyntax-only $(shell rospack export --lang=cpp --attrib=cflags) $(CHK_SOURCES)

