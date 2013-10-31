bin_PROGRAMS = demo_sine \
			   demo_active_set\
			   demo_heterogeneous_noise \
			   demo_large_dataset \
			   spatial_example

demo_active_set_SOURCES  = demo_active_set.cpp 
demo_active_set_LDADD    = $(top_builddir)/src/libgptk.la
demo_active_set_CPPFLAGS = -I$(top_srcdir)/src

demo_heterogeneous_noise_SOURCES = demo_heterogeneous_noise.cpp
demo_heterogeneous_noise_LDADD    = $(top_builddir)/src/libgptk.la
demo_heterogeneous_noise_CPPFLAGS = -I$(top_srcdir)/src

demo_large_dataset_SOURCES = demo_large_dataset.cpp
demo_large_dataset_LDADD = $(top_builddir)/src/libgptk.la
demo_large_dataset_CPPFLAGS = -I$(top_srcdir)/src

spatial_example_SOURCES = SpatialExample.cpp
spatial_example_LDADD = $(top_builddir)/src/libgptk.la
spatial_example_CPPFLAGS = -I$(top_srcdir)/src

demo_sine_SOURCES = demo_sine.cpp
demo_sine_LDADD = $(top_builddir)/src/libgptk.la
demo_sine_CPPFLAGS = -I$(top_srcdir)/src
