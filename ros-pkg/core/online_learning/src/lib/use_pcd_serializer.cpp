#include <online_learning/dataset.h>

CustomSerializer::Ptr Instance::custom_serializer_ = CustomSerializer::Ptr(new PCDSerializer);
