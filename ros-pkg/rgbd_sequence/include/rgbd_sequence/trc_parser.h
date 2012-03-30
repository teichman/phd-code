#ifndef TRC_PARSER_H
#define TRC_PARSER_H

#include <rgbd_sequence/stream_sequence.h>

namespace rgbd
{

  class TRCParser : public Serializable
  {
  public:
    std::vector<rgbd::Cloud::Ptr> frames_;

    TRCParser();
    rgbd::Cloud::Ptr getFrame(double ts, double tol) const;
    void deserialize(std::istream& in);
    void serialize(std::ostream& out) const;
    void explode(const std::string& line, std::vector<double>* fields) const;
  };
  
}

#endif // TRC_PARSER_H
