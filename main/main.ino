#define DRONE true
#define RECIEVER false

// substitute when uploading code
#if DRONE

#include "main/src/Drone/Drone.ino"

#else

#include "main/src/Communication/receiver.ino"

#endif