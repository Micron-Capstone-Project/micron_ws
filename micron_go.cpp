#include <iostream>
#include <csignal>

#include <micron/micron.hpp>

micron::Micron m;

void sigHandler(int sig){
    (void)sig;
    m.stop();
}

int main(int argc, char** argv){
    signal(SIGINT, sigHandler);
    m.go();

    return 0;
}
