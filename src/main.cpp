#include "engine.h"

int main(int argc, char* argv[])
{
    Engine engine = Engine();

    engine.setupImgui();
    engine.launch();
}