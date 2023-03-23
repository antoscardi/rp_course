CXX=g++ -O3 -fPIC -I/usr/include/eigen3

OBJS    = nn.o nn_optimizer.o nn_gd.o nn_gn.o neuron_factory.o
HEADERS = activation_function.h  neuron.h  neuron_factory.h  neuron_impl.h  nn.h  nn_gd.h  nn_gn.h  nn_impl.h  nn_optimizer.h

TARGETS = libsimplenn.so nn_gd_test nn_gn_test nn_make_net nn_make_train nn_optimize

.phony:	clean all


all:	  $(TARGETS)


clean:		
	rm -rf *.o *~ $(TARGETS)

%.o:		%.cpp 
		$(CXX)  -c $< 

nn_make_net:	nn_make_net.o libsimplenn.so
		$(CXX) -o $@ $< -Wl,-rpath=. libsimplenn.so	

nn_make_train:	nn_make_train.o libsimplenn.so
		$(CXX) -o $@ $< -Wl,-rpath=. libsimplenn.so	

nn_optimize:	nn_optimize.o libsimplenn.so
		$(CXX) -o $@ $< -Wl,-rpath=. libsimplenn.so	

nn_gd_test:	nn_gd_test.o libsimplenn.so
		$(CXX) -o $@ $< -Wl,-rpath=. libsimplenn.so	

nn_gn_test:	nn_gn_test.o libsimplenn.so
		$(CXX) -o $@ $< -Wl,-rpath=. libsimplenn.so 

libsimplenn.so: $(OBJS) $(HEADERS)
		$(CXX)  -fPIC -shared -o $@ $(OBJS)
