// main.cpp
#include <catch2/catch_session.hpp>

int main(int argc, char* argv[]) {
    // Optional: do any custom setup here (e.g., logging, config)

    Catch::Session session;

    int result = session.run(argc, argv);



    return result;
}