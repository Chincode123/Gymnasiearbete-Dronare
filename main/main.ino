#define DRONE true
#define RECIEVER false

// substitute when uploading code
#if DRONE

#include "src/Drone/drone.ino"

#else

#include "scr/Communication/receiver.ino"

#endif