#include <online_learning/dataset.h>
#include <online_learning/instance_serializer.h>

CustomSerializer::Ptr Instance::custom_serializer_ = CustomSerializer::Ptr(new PCDSerializer);
