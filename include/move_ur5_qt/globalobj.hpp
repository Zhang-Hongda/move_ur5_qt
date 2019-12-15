#ifndef GLOBALOBJ_H
#define GLOBALOBJ_H
#include "logger.hpp"
#include "stringbuffer.hpp"
#include "tf_listener.hpp"
#include "collision_objects_mannager.hpp"
#include "qnode.hpp"
class Tf_listener;
class Collision_Objects_Mannager;
class QNode;

extern Logger logger;
extern Tf_listener listener;
extern Collision_Objects_Mannager collision_objects_mannager;
extern QNode qnode;
#endif  // GLOBALOBJ_H
