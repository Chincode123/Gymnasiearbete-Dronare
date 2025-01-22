#define DRONE true
#define RECIEVER false

// substitute when uploading code
#if DRONE

#include "src/drone/Drone.ino"

#else

#include "src/Communication/Receiver.ino"

#endif