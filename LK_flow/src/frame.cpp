#include "frame.h"

Frame::Frame(unsigned long id) : id_(id), isKeyFrame_(false) {}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;
    Frame::Ptr new_frame(new Frame(factory_id++));
    return new_frame;
}