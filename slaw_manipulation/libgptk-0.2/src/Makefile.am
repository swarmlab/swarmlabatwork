GPTK_RELEASE=1:0:0
GPTK_LIBRARY_VERSION=0:2:0

# The final library
lib_LTLIBRARIES = libgptk.la

# The sources (none, as they are all in subdirectories and will be
# compiled as sub-libraries)
libgptk_la_SOURCES = 

# The headers
nobase_include_HEADERS = covariance_functions/CovarianceFunction.h \
                         covariance_functions/ConstantCF.h \
                         covariance_functions/StationaryCF.h \
                         covariance_functions/GaussianCF.h \
                         covariance_functions/ExponentialCF.h \
                         covariance_functions/Matern3CF.h \
                         covariance_functions/Matern5CF.h \
                         covariance_functions/NeuralNetCF.h \
                         covariance_functions/WhiteNoiseCF.h \
                         covariance_functions/SumCF.h \
                         design/Design.h \
                         design/MaxMinDesign.h \
                         design/GreedyMaxMinDesign.h \
                         design/MinMaxDesign.h \
                         gaussian_processes/ForwardModel.h \
                         gaussian_processes/GaussianProcess.h \
                         gaussian_processes/PSGP.h \
                         io/csvstream.h \
                         itppext/itppext.h \
                         likelihood_models/LikelihoodType.h \
                         likelihood_models/GaussianLikelihood.h \
                         likelihood_models/LikelihoodType.h \
                         optimisation/ModelTrainer.h \
                         optimisation/Optimisable.h \
                         optimisation/SCGModelTrainer.h \
                         parameter_transforms/Transform.h \
                         parameter_transforms/LogTransform.h \
                         parameter_transforms/IdentityTransform.h \
                         plotting/GraphPlotter.h
                         

# The sub-libraries
libgptk_la_LIBADD = gaussian_processes/libgp.la \
                    covariance_functions/libcovf.la \
                    design/libdesign.la \
                    optimisation/liboptim.la \
                    likelihood_models/liblikmod.la \
                    parameter_transforms/libtransforms.la \
                    itppext/libitppext.la \
                    io/libio.la \
                    plotting/libplot.la

# Some package information (version)
libgptk_la_LDFLAGS= -version-info $(GPTK_LIBRARY_VERSION) -release $(GPTK_RELEASE)

# Also run make in the following sub-directories 
SUBDIRS = gaussian_processes \
          covariance_functions \
          design \
          parameter_transforms \
          likelihood_models \
          optimisation \
          io \
          itppext \
          plotting
           
